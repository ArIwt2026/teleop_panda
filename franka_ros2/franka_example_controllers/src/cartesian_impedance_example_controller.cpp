// Copyright (c) 2023 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_example_controllers/cartesian_impedance_example_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(kNumberOfJoints);
  for (int i = 1; i <= kNumberOfJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumberOfJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(name);
  }
  return config;
}

CallbackReturn CartesianImpedanceExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {300.0, 300.0, 300.0, 30.0, 30.0, 30.0});
    auto_declare<std::vector<double>>("d_gains", {35.0, 35.0, 35.0, 5.0, 5.0, 5.0});
    auto_declare<double>("nullspace_stiffness", 0.5);
    auto_declare<double>("nullspace_damping", 1.4142135623730951);
    auto_declare<double>("nullspace_max_torque", 3.0);
    auto_declare<double>("joint_limit_margin", 0.15);
    auto_declare<double>("joint_limit_avoidance_torque", 1.0);
    // Panda limits from franka_description/robots/panda_arm.xacro. Other
    // Franka models must supply their own limits through these parameters.
    auto_declare<std::vector<double>>("joint_lower_limits",
        {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973});
    auto_declare<std::vector<double>>("joint_upper_limits",
        {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973});
    auto_declare<bool>("test_sinusoid", false);
    auto_declare<double>("sinusoid_amplitude", 0.03);
    auto_declare<double>("sinusoid_frequency", 0.2);
    auto_declare<std::string>("sinusoid_axis", "z");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.size() != 6 || d_gains.size() != 6) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains and d_gains must have exactly 6 values.");
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < 6; ++i) {
    k_gains_(i) = k_gains[i];
    d_gains_(i) = d_gains[i];
  }

  nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
  nullspace_damping_ = get_node()->get_parameter("nullspace_damping").as_double();
  nullspace_max_torque_ = get_node()->get_parameter("nullspace_max_torque").as_double();
  joint_limit_margin_ = get_node()->get_parameter("joint_limit_margin").as_double();
  joint_limit_avoidance_torque_ = get_node()->get_parameter("joint_limit_avoidance_torque").as_double();
  const auto lower = get_node()->get_parameter("joint_lower_limits").as_double_array();
  const auto upper = get_node()->get_parameter("joint_upper_limits").as_double_array();
  if (!std::isfinite(nullspace_stiffness_) || nullspace_stiffness_ < 0.0 ||
      !std::isfinite(nullspace_damping_) || nullspace_damping_ < 0.0 ||
      !std::isfinite(nullspace_max_torque_) || nullspace_max_torque_ <= 0.0 ||
      !std::isfinite(joint_limit_margin_) || joint_limit_margin_ < 0.0 ||
      !std::isfinite(joint_limit_avoidance_torque_) || joint_limit_avoidance_torque_ < 0.0 ||
      lower.size() != kNumberOfJoints || upper.size() != kNumberOfJoints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid nullspace gains, torque bound, or joint limits.");
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < kNumberOfJoints; ++i) {
    if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
        upper[i] - lower[i] <= 2.0 * joint_limit_margin_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid limits/margin for joint %d.", i + 1);
      return CallbackReturn::FAILURE;
    }
    joint_lower_limits_[i] = lower[i];
    joint_upper_limits_[i] = upper[i];
  }
  test_sinusoid_ = get_node()->get_parameter("test_sinusoid").as_bool();
  sinusoid_amplitude_ = get_node()->get_parameter("sinusoid_amplitude").as_double();
  sinusoid_frequency_ = get_node()->get_parameter("sinusoid_frequency").as_double();
  sinusoid_axis_ = get_node()->get_parameter("sinusoid_axis").as_string();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/" + k_robot_model_interface_name,
      arm_id_ + "/" + k_robot_state_interface_name);

  joint_overlay_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_overlay", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        jointOverlayCallback(msg);
      });

  admittance_sub_ = get_node()->create_subscription<control_msgs::msg::AdmittanceControllerState>(
      "admittance_controller/status", rclcpp::QoS(10),
      [this](const control_msgs::msg::AdmittanceControllerState::SharedPtr msg) {
        admittanceCallback(msg);
      });

  tau_last_.setZero();
  has_new_overlay_.store(false);

  RCLCPP_INFO(get_node()->get_logger(),
              "CartesianImpedanceExampleController configured as ChainableControllerInterface.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
CartesianImpedanceExampleController::on_export_reference_interfaces() {
  reference_interfaces_.resize(kNumberOfJoints, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(kNumberOfJoints);

  for (size_t i = 0; i < static_cast<size_t>(kNumberOfJoints); ++i) {
    reference_interfaces.emplace_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        arm_id_ + "_joint" + std::to_string(i + 1) + "/" + hardware_interface::HW_IF_POSITION,
        &reference_interfaces_[i]));
  }
  return reference_interfaces;
}

bool CartesianImpedanceExampleController::on_set_chained_mode(bool chained_mode) {
  RCLCPP_INFO(get_node()->get_logger(),
              "CartesianImpedanceExampleController: chained mode set to %s",
              chained_mode ? "true (preceded by controller)" : "false (external subscriber mode)");
  return true;
}

CallbackReturn CartesianImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  updateJointStates();

  q_initial_ = q_;
  q_pose_reference_ = q_initial_;
  for (int i = 0; i < kNumberOfJoints; ++i) {
    // Prefer the entry posture, with clearance if activated close to a limit.
    q_null_target_[i] = std::clamp(q_initial_[i], joint_lower_limits_[i] + joint_limit_margin_,
                                joint_upper_limits_[i] - joint_limit_margin_);
  }
  tau_last_.setZero();
  has_new_overlay_.store(false);

  // Initialize reference interfaces with current joint angles so no sudden jumps occur
  for (size_t i = 0; i < static_cast<size_t>(kNumberOfJoints) && i < reference_interfaces_.size(); ++i) {
    reference_interfaces_[i] = q_(i);
  }

  // Compute initial Cartesian pose once
  std::array<double, 16> pose_array =
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(pose_array.data()));
  position_target_ = initial_transform.translation();
  position_d_ = position_target_;
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());

  start_time_ = get_node()->now();
  max_update_ns_ = max_reference_fk_ns_ = max_model_ns_ = 0;
  updates_over_budget_ = 0;

  RCLCPP_INFO(get_node()->get_logger(),
              "Activated. Equilibrium TCP pose: [%.3f, %.3f, %.3f]. Chained mode: %s",
              position_target_.x(), position_target_.y(), position_target_.z(),
              is_in_chained_mode() ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(),
              "Nullspace posture captured on activation: stiffness %.2f Nm/rad, "
              "damping %.2f Nm*s/rad, torque bound %.2f Nm, joint-limit margin %.2f rad.",
              nullspace_stiffness_, nullspace_damping_, nullspace_max_torque_, joint_limit_margin_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(),
              "Cartesian timing: max update %.1f us, max reference FK %.1f us, "
              "max measured model %.1f us, updates over 1 ms %llu",
              max_update_ns_ / 1000.0, max_reference_fk_ns_ / 1000.0,
              max_model_ns_ / 1000.0, static_cast<unsigned long long>(updates_over_budget_));
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceExampleController::updateJointStates() {
  for (int i = 0; i < kNumberOfJoints; ++i) {
    q_(i) = state_interfaces_.at(2 * i).get_value();
    dq_(i) = state_interfaces_.at(2 * i + 1).get_value();
  }
}

void CartesianImpedanceExampleController::jointOverlayCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < static_cast<size_t>(kNumberOfJoints)) {
    return;
  }
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  for (int i = 0; i < kNumberOfJoints; ++i) {
    q_overlay_(i) = msg->position[i];
  }
  has_new_overlay_.store(true);
}

void CartesianImpedanceExampleController::admittanceCallback(
    const control_msgs::msg::AdmittanceControllerState::SharedPtr msg) {
  if (msg->joint_state.position.size() < static_cast<size_t>(kNumberOfJoints)) {
    return;
  }
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  for (int i = 0; i < kNumberOfJoints; ++i) {
    q_overlay_(i) = msg->joint_state.position[i];
  }
  has_new_overlay_.store(true);
}

controller_interface::return_type
CartesianImpedanceExampleController::update_reference_from_subscribers() {
  if (has_new_overlay_.load()) {
    std::unique_lock<std::mutex> lock(overlay_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return controller_interface::return_type::OK;
    }
    for (size_t i = 0; i < static_cast<size_t>(kNumberOfJoints) && i < reference_interfaces_.size(); ++i) {
      reference_interfaces_[i] = q_overlay_(i);
    }
    has_new_overlay_.store(false);
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type
CartesianImpedanceExampleController::update_and_write_commands(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  const auto update_started = std::chrono::steady_clock::now();
  updateJointStates();

  const double dt = period.seconds() > 0.0 ? period.seconds() : 0.001;

  // 1. Process reference interfaces (written by preceding controller in chained mode,
  // or by update_reference_from_subscribers in non-chained mode)
  if (reference_interfaces_.size() >= static_cast<size_t>(kNumberOfJoints)) {
    bool has_valid_ref = true;
    for (int i = 0; i < kNumberOfJoints; ++i) {
      if (!std::isfinite(reference_interfaces_[i])) {
        has_valid_ref = false;
        break;
      }
    }
    if (has_valid_ref) {
      Vector7d q_ref;
      for (int i = 0; i < kNumberOfJoints; ++i) {
        q_ref(i) = reference_interfaces_[i];
      }
      // Only recalculate FK if the reference changed
      if ((q_ref - q_pose_reference_).norm() > 1e-4) {
        const auto fk_started = std::chrono::steady_clock::now();
        q_pose_reference_ = q_ref;
        std::array<double, 7> q_arr;
        Eigen::Map<Vector7d>(q_arr.data()) = q_ref;
        std::array<double, 16> pose_arr =
            franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector, q_arr);
        Eigen::Affine3d t(Eigen::Matrix4d::Map(pose_arr.data()));
        position_target_ = t.translation();
        orientation_d_ = Eigen::Quaterniond(t.rotation());
        max_reference_fk_ns_ = std::max<int64_t>(max_reference_fk_ns_,
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - fk_started).count());
      }
    }
  }

  // 2. Sinusoidal TCP testing overlay
  position_d_ = position_target_;
  if (test_sinusoid_) {
    const double elapsed = (get_node()->now() - start_time_).seconds();
    const double ramp = std::min(1.0, elapsed / 3.0);
    const double wave = ramp * sinusoid_amplitude_ * std::sin(2.0 * M_PI * sinusoid_frequency_ * elapsed);

    int axis_idx = 2;  // 'z'
    if (sinusoid_axis_ == "x") axis_idx = 0;
    else if (sinusoid_axis_ == "y") axis_idx = 1;

    position_d_(axis_idx) += wave;
  }

  // 3. Current robot Cartesian state and dynamics
  const auto model_started = std::chrono::steady_clock::now();
  // libfranka already supplies the measured end-effector transform in this state.
  const auto& current_pose_arr = franka_robot_model_->getRobotState().O_T_EE;
  Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(current_pose_arr.data()));
  Eigen::Vector3d position_curr = current_transform.translation();
  Eigen::Quaterniond orientation_curr(current_transform.rotation());

  std::array<double, 42> jacobian_arr =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_arr.data());

  std::array<double, 7> coriolis_arr = franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<const Vector7d> coriolis(coriolis_arr.data());
  max_model_ns_ = std::max<int64_t>(max_model_ns_,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - model_started).count());

  // 4. Exact Franka Cartesian Error Formulation
  Vector6d error;
  error.head<3>() = position_curr - position_d_;

  if (orientation_d_.coeffs().dot(orientation_curr.coeffs()) < 0.0) {
    orientation_curr.coeffs() << -orientation_curr.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation_curr.inverse() * orientation_d_);
  error.tail<3>() << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail<3>() = -current_transform.rotation() * error.tail<3>();

  // 5. Cartesian Spring-Damper Wrench
  Vector6d wrench = -k_gains_.cwiseProduct(error) - d_gains_.cwiseProduct(jacobian * dq_);

  // Clamp task wrench for safety
  const double max_force = 40.0;   // 40 N
  const double max_torque = 10.0;  // 10 Nm
  if (wrench.head<3>().norm() > max_force) {
    wrench.head<3>() = wrench.head<3>().normalized() * max_force;
  }
  if (wrench.tail<3>().norm() > max_torque) {
    wrench.tail<3>() = wrench.tail<3>().normalized() * max_torque;
  }

  // 6. Joint Torques from Task Space
  Vector7d tau_task = jacobian.transpose() * wrench;

  // 7. Preferred posture and damping, independent of the moving TCP reference.
  Vector7d tau_null = Vector7d::Zero();
  if (nullspace_stiffness_ > 0.0 || nullspace_damping_ > 0.0 ||
      (joint_limit_margin_ > 0.0 && joint_limit_avoidance_torque_ > 0.0)) {
    if (!jacobian.allFinite() || !q_.allFinite() || !dq_.allFinite()) {
      return controller_interface::return_type::ERROR;
    }
    // Fixed-size SVD uses no heap storage. Include only numerical null directions;
    // damped inversion would also admit task directions near singularities.
    nullspace_svd_.compute(jacobian, Eigen::ComputeFullV);
    if (nullspace_svd_.info() != Eigen::Success) {
      return controller_interface::return_type::ERROR;
    }
    const auto& V = nullspace_svd_.matrixV();
    Eigen::Matrix<double, 7, 7> N = V.col(6) * V.col(6).transpose();
    const double rank_tolerance = std::max(1e-9, 1e-6 * nullspace_svd_.singularValues()[0]);
    for (int i = 0; i < 6; ++i) {
      if (nullspace_svd_.singularValues()[i] <= rank_tolerance) {
        N.noalias() += V.col(i) * V.col(i).transpose();
      }
    }
    Vector7d posture_torque =
        nullspace_stiffness_ * (q_null_target_ - q_) - nullspace_damping_ * dq_;
    if (joint_limit_margin_ > 0.0 && joint_limit_avoidance_torque_ > 0.0) {
      for (int i = 0; i < kNumberOfJoints; ++i) {
        const double near_lower = std::clamp(
            (joint_lower_limits_[i] + joint_limit_margin_ - q_[i]) / joint_limit_margin_, 0.0, 1.0);
        const double near_upper = std::clamp(
            (q_[i] - joint_upper_limits_[i] + joint_limit_margin_) / joint_limit_margin_, 0.0, 1.0);
        // Bounded, continuous repulsion within the margin, zero elsewhere.
        posture_torque[i] += joint_limit_avoidance_torque_ *
                            (near_lower * near_lower - near_upper * near_upper);
      }
    }
    tau_null.noalias() = N * posture_torque;
    // Scale the complete vector: per-joint clipping would break the projection.
    const double largest_torque = tau_null.cwiseAbs().maxCoeff();
    if (largest_torque > nullspace_max_torque_) {
      tau_null *= nullspace_max_torque_ / largest_torque;
    }
  }

  // 8. Total Commanded Torques with Coriolis Compensation
  Vector7d tau_cmd = tau_task + tau_null + coriolis;

  // 9. Slew rate limit and absolute torque saturation
  const Vector7d tau_max = (Vector7d() << 80.0, 80.0, 80.0, 80.0, 25.0, 25.0, 25.0).finished();
  const double max_rate = 1000.0 * dt;  // 1000 Nm/s slew rate
  for (int i = 0; i < kNumberOfJoints; ++i) {
    double delta = std::clamp(tau_cmd(i) - tau_last_(i), -max_rate, max_rate);
    tau_cmd(i) = std::clamp(tau_last_(i) + delta, -tau_max(i), tau_max(i));
  }
  tau_last_ = tau_cmd;

  // 10. Command to Hardware Interfaces
  for (int i = 0; i < kNumberOfJoints; ++i) {
#ifdef HW_HAS_SET_NODISCARD
    if (!command_interfaces_[i].set_value(tau_cmd(i))) {
      return controller_interface::return_type::ERROR;
    }
#else
    command_interfaces_[i].set_value(tau_cmd(i));
#endif
  }

  const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - update_started).count();
  max_update_ns_ = std::max<int64_t>(max_update_ns_, elapsed_ns);
  if (elapsed_ns > 1000000) {
    ++updates_over_budget_;
  }
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ChainableControllerInterface)
