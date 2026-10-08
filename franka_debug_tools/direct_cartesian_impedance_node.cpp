#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <franka_msgs/msg/franka_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "examples_common.h"
#include "joint_limit_guard.hpp"

namespace {
constexpr double kCollisionThreshold = 100.0;
constexpr auto kJointStatePeriod = std::chrono::milliseconds(4);

}  // namespace

class DirectCartesianImpedance final : public rclcpp::Node {
 public:
  explicit DirectCartesianImpedance(const rclcpp::NodeOptions &options)
      : Node("direct_cartesian_impedance", options),
        robot_ip_(declare_parameter<std::string>("robot_ip", "172.16.0.2")) {
    auto read_array = [this](const char *name) {
      const auto values=declare_parameter<std::vector<double>>(name, std::vector<double>{});
      if (values.size()!=7) throw std::invalid_argument(std::string(name)+" requires 7 values");
      std::array<double,7> result;
      std::copy(values.begin(), values.end(), result.begin());
      return result;
    };
    guard_.lower=read_array("limit_lower");
    guard_.upper=read_array("limit_upper");
    guard_.stiffness=read_array("limit_stiffness");
    guard_.damping=read_array("limit_damping");
    guard_.torque_cap=read_array("limit_torque_cap");
    guard_.activation=declare_parameter("limit_activation_deg",10.0)*M_PI/180.0;
    guard_.reserve=declare_parameter("limit_reserve_deg",3.0)*M_PI/180.0;
    guard_.prediction=declare_parameter("limit_prediction_seconds",0.1);
    guard_.validate();
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "franka/joint_states", rclcpp::QoS(10));
    joint_command_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "franka/joint_commands", rclcpp::QoS(10));
    robot_state_publisher_ = create_publisher<franka_msgs::msg::FrankaRobotState>(
        "franka/robot_state", rclcpp::QoS(10));
    joint_state_timer_ = create_wall_timer(kJointStatePeriod, [this] { publishJointState(); });
    toggle_service_ = create_service<std_srvs::srv::Trigger>(
        "/panda/joint_impedance/toggle",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          toggle(response);
        });
    worker_thread_ = std::thread([this] { controlWorker(); });
  }

  ~DirectCartesianImpedance() override {
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      shutdown_requested_ = true;
      stop_requested_.store(true);
    }
    control_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

 private:
  enum class Mode { kIdle, kZero };

  void toggle(const std::shared_ptr<std_srvs::srv::Trigger::Response> &response) {
    std::unique_lock<std::mutex> lock(control_mutex_);
    if (transition_in_progress_) {
      response->success = false;
      response->message = "Direct Cartesian impedance transition already in progress.";
      return;
    }

    // A previous libfranka exception stops the active session.  A subsequent
    // request is the explicit retry after the operator has cleared the robot
    // fault; if the fault is still present, the new session will fail again
    // and remain stopped.
    last_error_.clear();

    requested_mode_ = active_mode_ == Mode::kZero ? Mode::kIdle : Mode::kZero;
    ++requested_generation_;
    const auto generation = requested_generation_;
    transition_in_progress_ = true;
    stop_requested_.store(true);
    control_cv_.notify_all();

    const bool started = control_cv_.wait_for(lock, std::chrono::seconds(10), [this, generation] {
      return started_generation_ >= generation || shutdown_requested_ || !last_error_.empty();
    });
    if (!started || shutdown_requested_) {
      response->success = false;
      response->message = "Timed out while switching direct Cartesian impedance.";
      return;
    }
    if (!last_error_.empty()) {
      response->success = false;
      response->message = "Direct Cartesian impedance failed: " + last_error_;
      return;
    }

    response->success = true;
    response->message = requested_mode_ == Mode::kZero
                            ? "Zero Cartesian impedance started."
                            : "Direct Cartesian impedance stopped; state monitoring remains active.";
  }

  void controlWorker() {
    uint64_t generation = 0;
    while (rclcpp::ok()) {
      Mode mode;
      {
        std::unique_lock<std::mutex> lock(control_mutex_);
        if (shutdown_requested_) {
          return;
        }
        mode = requested_mode_;
        generation = requested_generation_;
        stop_requested_.store(false);
        last_error_.clear();
      }

      try {
        runSession(mode, generation);
      } catch (const std::exception &exception) {
        std::unique_lock<std::mutex> lock(control_mutex_);
        last_error_ = exception.what();
        active_mode_ = Mode::kIdle;
        requested_mode_ = Mode::kIdle;
        stop_requested_.store(false);
        transition_in_progress_ = false;
        control_cv_.notify_all();
        RCLCPP_ERROR(get_logger(), "Direct Cartesian impedance stopped: %s. Falling back to continuous state monitoring.", exception.what());
        // Keep publishing ROS 2 state while the failed control session is stopped.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }

      std::unique_lock<std::mutex> lock(control_mutex_);
      if (shutdown_requested_) {
        return;
      }
      control_cv_.wait(lock, [this, generation] {
        return shutdown_requested_ || requested_generation_ > generation;
      });
    }
  }

  void runSession(Mode mode, uint64_t generation) {
    franka::Robot robot(robot_ip_, franka::RealtimeConfig::kIgnore);

    auto mark_started = [this, mode, generation]() {
      std::lock_guard<std::mutex> lock(control_mutex_);
      active_mode_ = mode;
      started_generation_ = generation;
      transition_in_progress_ = false;
      control_cv_.notify_all();
    };
    if (mode == Mode::kIdle) {
      mark_started();
      RCLCPP_INFO(get_logger(), "Starting continuous direct Panda state-monitoring session.");
      while (!stop_requested_.load() && !shutdown_requested_) {
        try {
          robot.read([this](const franka::RobotState &robot_state) {
            publishRealtimeState(robot_state);
            return !stop_requested_.load() && !shutdown_requested_;
          });
        } catch (const franka::Exception &e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Panda state-monitoring read exception: %s. Retrying in 500ms...", e.what());
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      }
      return;
    }

    try {
      robot.automaticErrorRecovery();
    } catch (...) {}

    const auto initial=robot.readOnce();
    if (static_cast<bool>(initial.current_errors) || initial.robot_mode != franka::RobotMode::kIdle)
      throw std::runtime_error("Robot must be idle with current errors cleared before starting");
    for (size_t i=0; i<7; ++i)
      if (!guard_.inside(i,initial.q[i]))
        throw std::runtime_error("Joint outside configured physical limits; recover position before retry");
    setDefaultBehavior(robot);
    robot.setCollisionBehavior(
        {{kCollisionThreshold, kCollisionThreshold, kCollisionThreshold, kCollisionThreshold,
          kCollisionThreshold, kCollisionThreshold, kCollisionThreshold}},
        {{kCollisionThreshold, kCollisionThreshold, kCollisionThreshold, kCollisionThreshold,
          kCollisionThreshold, kCollisionThreshold, kCollisionThreshold}},
        {{kCollisionThreshold, kCollisionThreshold, kCollisionThreshold, kCollisionThreshold,
          kCollisionThreshold, kCollisionThreshold}},
        {{kCollisionThreshold, kCollisionThreshold, kCollisionThreshold, kCollisionThreshold,
          kCollisionThreshold, kCollisionThreshold}});
    franka::Model model = robot.loadModel();
    RCLCPP_INFO(get_logger(), "Starting direct zero Cartesian impedance session.");
    bool first=true;
    robot.control([this, &model, &first, &mark_started](const franka::RobotState &robot_state,
                                 franka::Duration) -> franka::Torques {
      if (static_cast<bool>(robot_state.current_errors))
        throw std::runtime_error("Robot current_errors active; control stopped");
      for (size_t i=0; i<7; ++i)
        if (!guard_.inside(i,robot_state.q[i]) || !std::isfinite(robot_state.dq[i]))
          throw std::runtime_error("Physical joint limit reached or invalid state; control stopped");
      if (first) { mark_started(); first=false; }
      const std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      const Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      std::array<double, 7> tau_d_array{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = coriolis;
      for (size_t i=0; i<7; ++i)
        tau_d_array[i] += guard_.torque(i, robot_state.q[i], robot_state.dq[i]);
      publishRealtimeState(robot_state, &tau_d_array);
      if (stop_requested_.load()) {
        return franka::MotionFinished(franka::Torques(tau_d_array));
      }
      return tau_d_array;
    }, true, 100.0);
  }

  void publishRealtimeState(const franka::RobotState &state, const std::array<double, 7> *tau_d = nullptr) {
    for (size_t index = 0; index < 7; ++index) {
      position_[index].store(state.q[index], std::memory_order_relaxed);
      velocity_[index].store(state.dq[index], std::memory_order_relaxed);
      effort_[index].store(state.tau_J[index], std::memory_order_relaxed);
    }
    {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      latest_telemetry_state_ = state;
      if (tau_d) {
        latest_telemetry_tau_d_ = *tau_d;
        has_telemetry_tau_d_ = true;
      } else {
        has_telemetry_tau_d_ = false;
      }
    }
    has_state_.store(true, std::memory_order_release);
  }

  void publishJointState() {
    if (!has_state_.load(std::memory_order_acquire)) {
      return;
    }
    const auto stamp = now();
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    for (size_t index = 0; index < 7; ++index) {
      message.name.emplace_back("panda_joint" + std::to_string(index + 1));
      message.position.emplace_back(position_[index].load(std::memory_order_relaxed));
      message.velocity.emplace_back(velocity_[index].load(std::memory_order_relaxed));
      message.effort.emplace_back(effort_[index].load(std::memory_order_relaxed));
    }
    joint_state_publisher_->publish(message);

    franka_msgs::msg::FrankaRobotState franka_msg;
    franka_msg.header.stamp = stamp;
    franka_msg.header.frame_id = "panda_link0";

    sensor_msgs::msg::JointState cmd_msg;
    cmd_msg.header.stamp = stamp;

    {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      const auto &s = latest_telemetry_state_;
      for (size_t i = 0; i < 7; ++i) {
        franka_msg.q[i] = s.q[i];
        franka_msg.q_d[i] = s.q_d[i];
        franka_msg.dq[i] = s.dq[i];
        franka_msg.dq_d[i] = s.dq_d[i];
        franka_msg.ddq_d[i] = s.ddq_d[i];
        franka_msg.theta[i] = s.theta[i];
        franka_msg.dtheta[i] = s.dtheta[i];
        franka_msg.tau_j[i] = s.tau_J[i];
        franka_msg.dtau_j[i] = s.dtau_J[i];
        const double tau_cmd = has_telemetry_tau_d_ ? latest_telemetry_tau_d_[i] : s.tau_J_d[i];
        franka_msg.tau_j_d[i] = tau_cmd;
        franka_msg.tau_ext_hat_filtered[i] = s.tau_ext_hat_filtered[i];

        cmd_msg.name.emplace_back("panda_joint" + std::to_string(i + 1));
        cmd_msg.position.emplace_back(s.q_d[i]);
        cmd_msg.velocity.emplace_back(s.dq_d[i]);
        cmd_msg.effort.emplace_back(tau_cmd);
      }
      for (size_t i = 0; i < 6; ++i) {
        franka_msg.o_f_ext_hat_k[i] = s.O_F_ext_hat_K[i];
        franka_msg.k_f_ext_hat_k[i] = s.K_F_ext_hat_K[i];
      }
      for (size_t i = 0; i < 16; ++i) {
        franka_msg.o_t_ee[i] = s.O_T_EE[i];
        franka_msg.o_t_ee_d[i] = s.O_T_EE_d[i];
        franka_msg.o_t_ee_c[i] = s.O_T_EE_c[i];
        franka_msg.f_t_ee[i] = s.F_T_EE[i];
        franka_msg.ee_t_k[i] = s.EE_T_K[i];
      }
      franka_msg.time = s.time.toSec();
      franka_msg.control_command_success_rate = s.control_command_success_rate;
      franka_msg.robot_mode = static_cast<uint8_t>(s.robot_mode);
    }
    robot_state_publisher_->publish(franka_msg);
    joint_command_publisher_->publish(cmd_msg);
  }

  const std::string robot_ip_;
  JointLimitGuard guard_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::Publisher<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_service_;
  std::array<std::atomic<double>, 7> position_{};
  std::array<std::atomic<double>, 7> velocity_{};
  std::array<std::atomic<double>, 7> effort_{};
  std::atomic_bool has_state_{false};
  std::atomic_bool stop_requested_{false};
  std::thread worker_thread_;
  std::mutex control_mutex_;
  std::condition_variable control_cv_;
  std::mutex telemetry_mutex_;
  franka::RobotState latest_telemetry_state_{};
  std::array<double, 7> latest_telemetry_tau_d_{};
  bool has_telemetry_tau_d_{false};
  Mode active_mode_{Mode::kIdle};
  Mode requested_mode_{Mode::kIdle};
  uint64_t requested_generation_{0};
  uint64_t started_generation_{0};
  bool transition_in_progress_{false};
  bool shutdown_requested_{false};
  std::string last_error_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectCartesianImpedance>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
