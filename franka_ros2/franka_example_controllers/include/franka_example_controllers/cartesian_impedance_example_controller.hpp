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

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/chainable_controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <control_msgs/msg/admittance_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * High-performance, real-time safe Cartesian impedance controller.
 * Inherits from ChainableControllerInterface to export reference joint position interfaces
 * for preceding controllers (such as admittance_controller) while accepting external overlays.
 */
class CartesianImpedanceExampleController : public controller_interface::ChainableControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;
  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::string arm_id_{"panda"};
  static constexpr int kNumberOfJoints = 7;

  // Joint state vectors
  Vector7d q_;
  Vector7d dq_;
  Vector7d q_initial_;
  Vector7d q_null_target_;
  Vector7d tau_last_;

  // Cartesian pose state (cached, RT safe)
  Eigen::Vector3d position_target_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

  // Impedance gains
  Vector6d k_gains_;
  Vector6d d_gains_;
  double nullspace_stiffness_{0.5};

  // Sinusoidal TCP test overlay
  bool test_sinusoid_{false};
  double sinusoid_amplitude_{0.03};   // 3 cm
  double sinusoid_frequency_{0.2};    // 0.2 Hz (5s period)
  std::string sinusoid_axis_{"z"};

  rclcpp::Time start_time_;

  // Franka robot model semantic component
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  const std::string k_robot_model_interface_name{"robot_model"};
  const std::string k_robot_state_interface_name{"robot_state"};

  // Joint overlay subscriber
  std::mutex overlay_mutex_;
  std::atomic<bool> has_new_overlay_{false};
  Vector7d q_overlay_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_overlay_sub_;
  rclcpp::Subscription<control_msgs::msg::AdmittanceControllerState>::SharedPtr admittance_sub_;

  void updateJointStates();
  void jointOverlayCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void admittanceCallback(const control_msgs::msg::AdmittanceControllerState::SharedPtr msg);
};

}  // namespace franka_example_controllers
