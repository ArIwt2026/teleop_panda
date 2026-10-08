// Copyright (c) 2026
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

#include "franka_example_controllers/joint_impedance_chainable_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceChainableController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(kNumberOfJoints);
  for (int i = 1; i <= kNumberOfJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/" +
                           hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceChainableController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(2 * kNumberOfJoints + 2);
  for (int i = 1; i <= kNumberOfJoints; ++i) {
    const auto joint = arm_id_ + "_joint" + std::to_string(i);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  config.names.push_back(arm_id_ + "/" + k_robot_state_interface_name);
  config.names.push_back(arm_id_ + "/" + k_robot_model_interface_name);
  return config;
}

CallbackReturn JointImpedanceChainableController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>(
        "k_gains", {250.0, 250.0, 250.0, 200.0, 125.0, 75.0, 40.0});
    auto_declare<std::vector<double>>(
        "d_gains", {20.0, 20.0, 20.0, 16.0, 10.0, 6.0, 3.0});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceChainableController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  const auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  const auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  if (k_gains.size() != static_cast<size_t>(kNumberOfJoints) ||
      d_gains.size() != static_cast<size_t>(kNumberOfJoints)) {
    RCLCPP_ERROR(get_node()->get_logger(), "k_gains and d_gains must each contain %d values.",
                 kNumberOfJoints);
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < kNumberOfJoints; ++i) {
    if (!std::isfinite(k_gains[i]) || k_gains[i] < 0.0 ||
        !std::isfinite(d_gains[i]) || d_gains[i] < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Impedance gains must be finite and non-negative.");
      return CallbackReturn::FAILURE;
    }
    k_gains_(i) = k_gains[i];
    d_gains_(i) = d_gains[i];
  }

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/" + k_robot_model_interface_name,
      arm_id_ + "/" + k_robot_state_interface_name);

  joint_overlay_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_overlay", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        jointOverlayCallback(msg);
      });

  dq_filtered_.setZero();
  tau_last_.setZero();
  has_new_overlay_.store(false);

  RCLCPP_INFO(get_node()->get_logger(),
              "JointImpedanceChainableController configured successfully.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
JointImpedanceChainableController::on_export_reference_interfaces() {
  reference_interfaces_.resize(kNumberOfJoints, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kNumberOfJoints);

  for (size_t i = 0; i < static_cast<size_t>(kNumberOfJoints); ++i) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        arm_id_ + "_joint" + std::to_string(i + 1) + "/" + hardware_interface::HW_IF_POSITION,
        &reference_interfaces_[i]));
  }
  return interfaces;
}

bool JointImpedanceChainableController::on_set_chained_mode(bool chained_mode) {
  RCLCPP_INFO(get_node()->get_logger(),
              "JointImpedanceChainableController: chained mode set to %s",
              chained_mode ? "true (preceded by controller)" : "false (external subscriber mode)");
  return true;
}

CallbackReturn JointImpedanceChainableController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  updateJointStates();

  for (int i = 0; i < kNumberOfJoints; ++i) {
    q_d_(i) = q_(i);
    reference_interfaces_[i] = q_(i);
  }
  dq_filtered_.setZero();
  tau_last_.setZero();
  has_new_overlay_.store(false);

  RCLCPP_INFO(get_node()->get_logger(),
              "JointImpedanceChainableController activated. Chained mode: %s",
              is_in_chained_mode() ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceChainableController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void JointImpedanceChainableController::updateJointStates() {
  for (int i = 0; i < kNumberOfJoints; ++i) {
    q_(i) = state_interfaces_.at(2 * i).get_value();
    dq_(i) = state_interfaces_.at(2 * i + 1).get_value();
  }
}

void JointImpedanceChainableController::jointOverlayCallback(
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

controller_interface::return_type
JointImpedanceChainableController::update_reference_from_subscribers() {
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
JointImpedanceChainableController::update_and_write_commands(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  updateJointStates();
  const double dt = period.seconds() > 0.0 ? period.seconds() : 0.001;

  // Process reference interfaces
  if (reference_interfaces_.size() >= static_cast<size_t>(kNumberOfJoints)) {
    bool has_valid_ref = true;
    for (int i = 0; i < kNumberOfJoints; ++i) {
      if (!std::isfinite(reference_interfaces_[i])) {
        has_valid_ref = false;
        break;
      }
    }
    if (has_valid_ref) {
      for (int i = 0; i < kNumberOfJoints; ++i) {
        q_d_(i) = reference_interfaces_[i];
      }
    }
  }

  // Coriolis compensation from Franka model
  std::array<double, 7> coriolis_arr = franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<const Vector7d> coriolis(coriolis_arr.data());

  // Filter measured velocity
  constexpr double kVelocityFilterAlpha = 0.2;
  dq_filtered_ = (1.0 - kVelocityFilterAlpha) * dq_filtered_ + kVelocityFilterAlpha * dq_;

  // Joint spring-damper torques
  Vector7d tau_task = Vector7d::Zero();
  for (int i = 0; i < kNumberOfJoints; ++i) {
    tau_task(i) = k_gains_(i) * (q_d_(i) - q_(i)) - d_gains_(i) * dq_filtered_(i);
  }

  // Total commanded torque with Coriolis compensation
  Vector7d tau_cmd = tau_task + coriolis;

  // Slew rate limiting and torque saturation
  const Vector7d tau_max = (Vector7d() << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0).finished();
  const double max_rate = 1000.0 * dt;  // 1000 Nm/s slew rate
  for (int i = 0; i < kNumberOfJoints; ++i) {
    double delta = std::clamp(tau_cmd(i) - tau_last_(i), -max_rate, max_rate);
    tau_cmd(i) = std::clamp(tau_last_(i) + delta, -tau_max(i), tau_max(i));
  }
  tau_last_ = tau_cmd;

  // Write to hardware
  for (int i = 0; i < kNumberOfJoints; ++i) {
#ifdef HW_HAS_SET_NODISCARD
    if (!command_interfaces_[i].set_value(tau_cmd(i))) {
      return controller_interface::return_type::ERROR;
    }
#else
    command_interfaces_[i].set_value(tau_cmd(i));
#endif
  }

  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceChainableController,
                       controller_interface::ChainableControllerInterface)
