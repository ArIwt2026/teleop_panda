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

#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "examples_common.h"

namespace {
constexpr double kCollisionThreshold = 100.0;
constexpr auto kJointStatePeriod = std::chrono::milliseconds(4);
}  // namespace

class DirectCartesianImpedance final : public rclcpp::Node {
 public:
  explicit DirectCartesianImpedance(const rclcpp::NodeOptions &options)
      : Node("direct_cartesian_impedance", options),
        robot_ip_(declare_parameter<std::string>("robot_ip", "172.16.0.2")) {
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "franka/joint_states", rclcpp::QoS(10));
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
      } catch (const franka::Exception &exception) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        last_error_ = exception.what();
        active_mode_ = Mode::kIdle;
        requested_mode_ = Mode::kIdle;
        stop_requested_.store(true);
        transition_in_progress_ = false;
        control_cv_.notify_all();
        RCLCPP_ERROR(get_logger(), "Direct Cartesian impedance stopped: %s", exception.what());
        // Keep the node and service alive, but do not start another session
        // until a new toggle request explicitly retries after fault recovery.
        std::unique_lock<std::mutex> retry_lock(control_mutex_);
        control_cv_.wait(retry_lock, [this, generation] {
          return shutdown_requested_ || requested_generation_ > generation;
        });
        if (shutdown_requested_) {
          return;
        }
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

    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      active_mode_ = mode;
      started_generation_ = generation;
      transition_in_progress_ = false;
    }
    control_cv_.notify_all();
    if (mode == Mode::kIdle) {
      RCLCPP_INFO(get_logger(), "Starting direct Panda state-monitoring session.");
      robot.read([this](const franka::RobotState &robot_state) {
        publishRealtimeState(robot_state);
        return !stop_requested_.load();
      });
      return;
    }

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
    robot.control([this, &model](const franka::RobotState &robot_state,
                                 franka::Duration) -> franka::Torques {
      publishRealtimeState(robot_state);
      const std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      const Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      std::array<double, 7> tau_d_array{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = coriolis;
      if (stop_requested_.load()) {
        return franka::MotionFinished(franka::Torques(tau_d_array));
      }
      return tau_d_array;
    });
  }

  void publishRealtimeState(const franka::RobotState &state) {
    for (size_t index = 0; index < 7; ++index) {
      position_[index].store(state.q[index], std::memory_order_relaxed);
      velocity_[index].store(state.dq[index], std::memory_order_relaxed);
      effort_[index].store(state.tau_J[index], std::memory_order_relaxed);
    }
    has_state_.store(true, std::memory_order_release);
  }

  void publishJointState() {
    if (!has_state_.load(std::memory_order_acquire)) {
      return;
    }
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    for (size_t index = 0; index < 7; ++index) {
      message.name.emplace_back("panda_joint" + std::to_string(index + 1));
      message.position.emplace_back(position_[index].load(std::memory_order_relaxed));
      message.velocity.emplace_back(velocity_[index].load(std::memory_order_relaxed));
      message.effort.emplace_back(effort_[index].load(std::memory_order_relaxed));
    }
    joint_state_publisher_->publish(message);
  }

  const std::string robot_ip_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
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
