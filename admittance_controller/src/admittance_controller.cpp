// Copyright (c) 2022, PickNik, Inc.
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
//
/// \authors: Denis Stogl, Andy Zelenak, Paul Gesel

#include "admittance_controller/admittance_controller.hpp"

#include <tinyxml2.h>
#include <algorithm>
#include <chrono>
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "admittance_controller/admittance_rule_impl.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rcutils/logging_macros.h"
#include "tf2_ros/buffer.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace admittance_controller
{
controller_interface::CallbackReturn AdmittanceController::on_init()
{
  // initialize controller config
  try
  {
    parameter_handler_ = std::make_shared<admittance_controller::ParamListener>(get_node());
    admittance_ = std::make_unique<admittance_controller::AdmittanceRule>(parameter_handler_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // number of joints in controllers is fixed after initialization
  num_joints_ = admittance_->parameters_.joints.size();

  // allocate dynamic memory
  last_reference_.positions.assign(num_joints_, 0.0);
  last_reference_.velocities.assign(num_joints_, 0.0);
  last_reference_.accelerations.assign(num_joints_, 0.0);
  last_commanded_ = last_reference_;
  reference_ = last_reference_;
  reference_admittance_ = last_reference_;
  joint_state_ = last_reference_;

  // robot_description passed as a parameter by the controller manager
  std::string robot_description;
  if (!get_node()->get_parameter("robot_description", robot_description))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'robot_description' parameter not set.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'robot_description' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  tinyxml2::XMLDocument doc;
  if (!doc.Parse(robot_description.c_str()) && doc.Error())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to parse robot description XML from parameter "
      "'robot_description': %s",
      doc.ErrorStr());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (doc.Error())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Error parsing robot description XML from parameter "
      "'robot_description': %s",
      doc.ErrorStr());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration()
  const
{
  std::vector<std::string> command_interfaces_config_names;
  for (const auto & interface : admittance_->parameters_.command_interfaces)
  {
    for (const auto & joint : command_joint_names_)
    {
      auto full_name = joint + "/" + interface;
      command_interfaces_config_names.push_back(full_name);
    }
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration()
  const
{
  std::vector<std::string> state_interfaces_config_names;
  for (size_t i = 0; i < admittance_->parameters_.state_interfaces.size(); ++i)
  {
    const auto & interface = admittance_->parameters_.state_interfaces[i];
    for (const auto & joint : admittance_->parameters_.joints)
    {
      auto full_name = joint + "/" + interface;
      state_interfaces_config_names.push_back(full_name);
    }
  }

  auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
  state_interfaces_config_names.insert(
    state_interfaces_config_names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}

std::vector<hardware_interface::CommandInterface>
AdmittanceController::on_export_reference_interfaces()
{
  // create CommandInterface interfaces that other controllers will be able to chain with
  if (!admittance_)
  {
    return {};
  }

  std::vector<hardware_interface::CommandInterface> chainable_command_interfaces;
  const auto num_chainable_interfaces =
    admittance_->parameters_.chainable_command_interfaces.size() *
    admittance_->parameters_.joints.size();

  // allocate dynamic memory
  chainable_command_interfaces.reserve(num_chainable_interfaces);
  reference_interfaces_.resize(num_chainable_interfaces, std::numeric_limits<double>::quiet_NaN());
  position_reference_ = {};
  velocity_reference_ = {};

  // assign reference interfaces
  auto index = 0ul;
  for (const auto & interface : admittance_->parameters_.chainable_command_interfaces)
  {
    for (const auto & joint : admittance_->parameters_.joints)
    {
      if (hardware_interface::HW_IF_POSITION == interface)
        position_reference_.emplace_back(reference_interfaces_[index]);
      else if (hardware_interface::HW_IF_VELOCITY == interface)
      {
        velocity_reference_.emplace_back(reference_interfaces_[index]);
      }
      const auto full_name = joint + "/" + interface;
      chainable_command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          std::string(get_node()->get_name()), full_name, reference_interfaces_.data() + index));

      index++;
    }
  }

  return chainable_command_interfaces;
}

controller_interface::CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto parameters = parameter_handler_->get_params();
  teach_mode_enabled_ = parameters.teach_mode.enabled;
  return_max_joint_velocity_ = parameters.teach_mode.return_max_joint_velocity_rad_s;
  if (!std::isfinite(parameters.joint_command_state.publish_rate) ||
      parameters.joint_command_state.publish_rate <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joint_command_state.publish_rate must be positive.");
    return CallbackReturn::FAILURE;
  }
  joint_command_state_publish_period_sec_ = 1.0 / parameters.joint_command_state.publish_rate;
  if (teach_mode_enabled_)
  {
    const auto & home = parameters.teach_mode.home_joint_positions_rad;
    if (num_joints_ != home_joint_positions_.size() || home.size() != home_joint_positions_.size())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "teach_mode requires exactly seven joints and seven home_joint_positions_rad values.");
      return CallbackReturn::FAILURE;
    }
    if (!std::isfinite(return_max_joint_velocity_) || return_max_joint_velocity_ <= 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "return_max_joint_velocity_rad_s must be positive.");
      return CallbackReturn::FAILURE;
    }
    for (size_t i = 0; i < home_joint_positions_.size(); ++i)
    {
      if (!std::isfinite(home[i]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Home joint positions must be finite.");
        return CallbackReturn::FAILURE;
      }
      home_joint_positions_[i] = home[i];
    }
    teach_mode_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/teach_mode/toggle",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        const auto requested = requested_teach_mode_.load(std::memory_order_acquire);
        if (requested == TeachMode::RETURN_HOME)
        {
          response->success = false;
          response->message = "Returning home; wait until the robot reaches home.";
        }
        else if (requested == TeachMode::HOME_HOLD)
        {
          requested_teach_mode_.store(TeachMode::ADMITTANCE, std::memory_order_release);
          response->success = true;
          response->message = "Admittance teach mode requested.";
        }
        else
        {
          requested_teach_mode_.store(TeachMode::RETURN_HOME, std::memory_order_release);
          response->success = true;
          response->message = "Smooth return home requested.";
        }
      });
  }

  command_joint_names_ = admittance_->parameters_.command_joints;
  if (command_joint_names_.empty())
  {
    command_joint_names_ = admittance_->parameters_.joints;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }
  else if (command_joint_names_.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'command_joints' parameter has to have the same size as 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }

  // print and validate interface types
  for (const auto & tmp : admittance_->parameters_.state_interfaces)
  {
    RCLCPP_INFO(get_node()->get_logger(), "%s", ("state int types are: " + tmp + "\n").c_str());
  }
  for (const auto & tmp : admittance_->parameters_.command_interfaces)
  {
    RCLCPP_INFO(get_node()->get_logger(), "%s", ("command int types are: " + tmp + "\n").c_str());
  }

  // validate exported interfaces
  for (const auto & tmp : admittance_->parameters_.chainable_command_interfaces)
  {
    if (tmp == hardware_interface::HW_IF_POSITION || tmp == hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "%s", ("chainable int types are: " + tmp + "\n").c_str());
    }
    else
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "chainable interface type %s is not supported. Supported types are %s and %s", tmp.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  auto contains_interface_type =
    [](const std::vector<std::string> & interface_type_list, const std::string & interface_type)
  {
    return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
           interface_type_list.end();
  };

  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : admittance_->parameters_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Command interface type '%s' not allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  has_position_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_EFFORT);

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : admittance_->parameters_.state_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "State interface type '%s' not allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  has_position_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  if (!has_position_state_interface_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Position state interface is required.");
    return CallbackReturn::FAILURE;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_command_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_command_interfaces << " ";
      }
      ss_command_interfaces << interface_types[index];
    }
    return ss_command_interfaces.str();
  };
  RCLCPP_INFO(
    get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(admittance_->parameters_.command_interfaces).c_str(),
    get_interface_list(admittance_->parameters_.state_interfaces).c_str());

  // setup subscribers and publishers
  auto joint_command_callback =
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg)
  { input_joint_command_.writeFromNonRT(msg); };
  input_joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "~/joint_references", rclcpp::SystemDefaultsQoS(), joint_command_callback);
  s_publisher_ = get_node()->create_publisher<control_msgs::msg::AdmittanceControllerState>(
    "~/status", rclcpp::SystemDefaultsQoS());
  state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg>>(s_publisher_);

  joint_command_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/joint_commands", rclcpp::SensorDataQoS());
  joint_command_state_realtime_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      joint_command_state_publisher_);
  joint_command_state_realtime_publisher_->lock();
  auto & command_msg = joint_command_state_realtime_publisher_->msg_;
  command_msg.name = admittance_->parameters_.joints;
  command_msg.position.resize(num_joints_);
  command_msg.velocity.resize(num_joints_);
  joint_command_state_realtime_publisher_->unlock();

  // Initialize state message
  state_publisher_->lock();
  state_publisher_->msg_ = admittance_->get_controller_state();
  state_publisher_->unlock();

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    admittance_->parameters_.ft_sensor.name);

  // configure admittance rule
  if (admittance_->configure(get_node(), num_joints_) == controller_interface::return_type::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // on_activate is called when the lifecycle node activates.
  if (!admittance_)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_command_state_elapsed_sec_ = 0.0;

  // order all joints in the storage
  for (const auto & interface : admittance_->parameters_.state_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, admittance_->parameters_.joints, interface,
          joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", num_joints_,
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : admittance_->parameters_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", num_joints_,
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  // update parameters if any have changed
  admittance_->apply_parameters_update();

  // initialize interface of the FTS semantic component
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // initialize states
  read_state_from_hardware(joint_state_, ft_values_);
  for (auto val : joint_state_.positions)
  {
    if (std::isnan(val))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint positions from the hardware.\n");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Start every activation with no stored admittance motion.
  admittance_->reset(num_joints_);
  if (admittance_->parameters_.measured_position_command.enabled)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Measured-position admittance mode: %.1f Hz calculations, %.3f s position-step period.",
      admittance_->parameters_.measured_position_command.update_rate, admittance_->parameters_.measured_position_command.sample_time);
  }

  // Use current joint_state as a default reference
  last_reference_ = joint_state_;
  last_commanded_ = joint_state_;
  reference_ = joint_state_;
  reference_admittance_ = joint_state_;

  if (teach_mode_enabled_)
  {
    start_return_home_from_measured_state();
    double max_delta = 0.0;
    for (size_t i = 0; i < home_joint_positions_.size(); ++i)
    {
      max_delta = std::max(max_delta, std::abs(home_joint_positions_[i] - return_start_positions_[i]));
    }
    if (max_delta < 1e-3)
    {
      active_teach_mode_ = TeachMode::HOME_HOLD;
      requested_teach_mode_.store(TeachMode::HOME_HOLD, std::memory_order_release);
      set_home_hold_command();
      RCLCPP_INFO(get_node()->get_logger(), "Already at configured Panda home position; holding.");
    }
    else
    {
      active_teach_mode_ = TeachMode::RETURN_HOME;
      requested_teach_mode_.store(TeachMode::RETURN_HOME, std::memory_order_release);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Moving to configured Panda home at no more than %.3f rad/s per joint.",
        return_max_joint_velocity_);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update_reference_from_subscribers()
{
  // update input reference from ros subscriber message
  if (!admittance_)
  {
    return controller_interface::return_type::ERROR;
  }

  joint_command_msg_ = *input_joint_command_.readFromRT();

  // if message exists, load values into references
  if (joint_command_msg_.get())
  {
    for (const auto & interface : admittance_->parameters_.chainable_command_interfaces)
    {
      if (interface == hardware_interface::HW_IF_POSITION)
      {
        for (size_t i = 0; i < joint_command_msg_->positions.size(); ++i)
        {
          position_reference_[i].get() = joint_command_msg_->positions[i];
        }
      }
      else if (interface == hardware_interface::HW_IF_VELOCITY)
      {
        for (size_t i = 0; i < joint_command_msg_->velocities.size(); ++i)
        {
          velocity_reference_[i].get() = joint_command_msg_->velocities[i];
        }
      }
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Realtime constraints are required in this function
  if (!admittance_)
  {
    return controller_interface::return_type::ERROR;
  }

  // update input reference from chainable interfaces
  read_state_reference_interfaces(reference_);

  // get all controller inputs
  read_state_from_hardware(joint_state_, ft_values_);

  if (teach_mode_enabled_)
  {
    const auto requested = requested_teach_mode_.load(std::memory_order_acquire);
    if (requested != active_teach_mode_)
    {
      if (requested == TeachMode::ADMITTANCE)
      {
        // Discard any prior velocity/offset state before starting a new hand-guiding segment.
        admittance_->reset(num_joints_);
        active_teach_mode_ = TeachMode::ADMITTANCE;
        last_reference_ = joint_state_;
        reference_ = joint_state_;
        reference_admittance_ = joint_state_;
      }
      else if (requested == TeachMode::RETURN_HOME)
      {
        start_return_home_from_measured_state();
        active_teach_mode_ = TeachMode::RETURN_HOME;
      }
      else
      {
        active_teach_mode_ = TeachMode::HOME_HOLD;
      }
    }

    if (active_teach_mode_ == TeachMode::RETURN_HOME)
    {
      return_elapsed_ = std::min(return_duration_, return_elapsed_ + std::max(0.0, period.seconds()));
      const double u = return_duration_ > 0.0 ? return_elapsed_ / return_duration_ : 1.0;
      const double u2 = u * u;
      const double u3 = u2 * u;
      const double u4 = u3 * u;
      const double u5 = u4 * u;
      const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
      const double blend_rate = return_duration_ > 0.0 ?
        (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / return_duration_ : 0.0;
      for (size_t i = 0; i < num_joints_; ++i)
      {
        const double delta = home_joint_positions_[i] - return_start_positions_[i];
        reference_admittance_.positions[i] = return_start_positions_[i] + blend * delta;
        reference_admittance_.velocities[i] = blend_rate * delta;
        if (!reference_admittance_.accelerations.empty())
        {
          reference_admittance_.accelerations[i] = 0.0;
        }
      }
      write_state_to_hardware(reference_admittance_);
      publish_joint_command_state(time, period);
      if (return_elapsed_ >= return_duration_)
      {
        active_teach_mode_ = TeachMode::HOME_HOLD;
        requested_teach_mode_.store(TeachMode::HOME_HOLD, std::memory_order_release);
        set_home_hold_command();
        write_state_to_hardware(reference_admittance_);
      }
      return controller_interface::return_type::OK;
    }

    if (active_teach_mode_ == TeachMode::HOME_HOLD)
    {
      set_home_hold_command();
      write_state_to_hardware(reference_admittance_);
      publish_joint_command_state(time, period);
      return controller_interface::return_type::OK;
    }
  }

  // apply admittance control to reference to determine desired state
  const auto result =
    admittance_->update(joint_state_, ft_values_, reference_, period, reference_admittance_);

  // write calculated values to joint interfaces
  write_state_to_hardware(reference_admittance_);
  publish_joint_command_state(time, period);

  // Status must never delay torque generation if DDS is still publishing.
  if (state_publisher_->trylock())
  {
    state_publisher_->msg_ = admittance_->get_controller_state();
    state_publisher_->unlockAndPublish();
  }

  return result;
}

void AdmittanceController::start_return_home_from_measured_state()
{
  double max_delta = 0.0;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    return_start_positions_[i] = joint_state_.positions[i];
    max_delta = std::max(max_delta, std::abs(home_joint_positions_[i] - return_start_positions_[i]));
  }
  return_elapsed_ = 0.0;
  // The quintic blend's peak derivative is 1.875. This duration therefore
  // limits every joint's peak speed to return_max_joint_velocity_.
  return_duration_ = std::max(2.0, 1.875 * max_delta / return_max_joint_velocity_);
}

void AdmittanceController::set_home_hold_command()
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    reference_admittance_.positions[i] = home_joint_positions_[i];
    if (!reference_admittance_.velocities.empty())
    {
      reference_admittance_.velocities[i] = 0.0;
    }
    if (!reference_admittance_.accelerations.empty())
    {
      reference_admittance_.accelerations[i] = 0.0;
    }
  }
}

void AdmittanceController::publish_joint_command_state(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!joint_command_state_realtime_publisher_ || !std::isfinite(period.seconds()) ||
      period.seconds() <= 0.0)
  {
    return;
  }

  joint_command_state_elapsed_sec_ += period.seconds();
  if (joint_command_state_elapsed_sec_ + 1e-12 < joint_command_state_publish_period_sec_)
  {
    return;
  }
  joint_command_state_elapsed_sec_ = std::fmod(
    joint_command_state_elapsed_sec_, joint_command_state_publish_period_sec_);

  if (!joint_command_state_realtime_publisher_->trylock())
  {
    return;
  }

  auto & msg = joint_command_state_realtime_publisher_->msg_;
  msg.header.stamp = time;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    msg.position[i] = last_commanded_.positions[i];
    msg.velocity[i] = i < last_commanded_.velocities.size() ? last_commanded_.velocities[i] : 0.0;
  }
  joint_command_state_realtime_publisher_->unlockAndPublish();
}

controller_interface::CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!admittance_)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // release force torque sensor interface
  force_torque_sensor_->release_interfaces();

  // reset to prevent stale references
  for (size_t i = 0; i < num_joints_; i++)
  {
    for (const auto & interface : admittance_->parameters_.chainable_command_interfaces)
    {
      if (interface == hardware_interface::HW_IF_POSITION)
        position_reference_[i].get() = std::numeric_limits<double>::quiet_NaN();
      else if (interface == hardware_interface::HW_IF_VELOCITY)
        velocity_reference_[i].get() = std::numeric_limits<double>::quiet_NaN();
    }
  }

  for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();
  admittance_->reset(num_joints_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!admittance_)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  admittance_->reset(num_joints_);
  return controller_interface::CallbackReturn::SUCCESS;
}

void AdmittanceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & state_current,
  geometry_msgs::msg::Wrench & ft_values)
{
  // if any interface has nan values, assume state_current is the last command state
  bool nan_position = false;
  bool nan_velocity = false;
  bool nan_acceleration = false;

  size_t pos_ind = 0;
  size_t vel_ind = pos_ind + has_velocity_state_interface_;
  size_t acc_ind = vel_ind + has_acceleration_state_interface_;
  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind)
  {
    if (has_position_state_interface_)
    {
      state_current.positions[joint_ind] =
        state_interfaces_[pos_ind * num_joints_ + joint_ind].get_value();
      nan_position |= std::isnan(state_current.positions[joint_ind]);
    }
    if (has_velocity_state_interface_)
    {
      state_current.velocities[joint_ind] =
        state_interfaces_[vel_ind * num_joints_ + joint_ind].get_value();
      nan_velocity |= std::isnan(state_current.velocities[joint_ind]);
    }
    if (has_acceleration_state_interface_)
    {
      state_current.accelerations[joint_ind] =
        state_interfaces_[acc_ind * num_joints_ + joint_ind].get_value();
      nan_acceleration |= std::isnan(state_current.accelerations[joint_ind]);
    }
  }

  if (nan_position)
  {
    state_current.positions = last_commanded_.positions;
  }
  if (nan_velocity)
  {
    state_current.velocities = last_commanded_.velocities;
  }
  if (nan_acceleration)
  {
    state_current.accelerations = last_commanded_.accelerations;
  }

  // if any ft_values are nan, assume values are zero
  force_torque_sensor_->get_values_as_message(ft_values);
  if (
    std::isnan(ft_values.force.x) || std::isnan(ft_values.force.y) ||
    std::isnan(ft_values.force.z) || std::isnan(ft_values.torque.x) ||
    std::isnan(ft_values.torque.y) || std::isnan(ft_values.torque.z))
  {
    ft_values = geometry_msgs::msg::Wrench();
  }
}

void AdmittanceController::write_state_to_hardware(
  const trajectory_msgs::msg::JointTrajectoryPoint & state_commanded)
{
  // if any interface has nan values, assume state_commanded is the last command state
  size_t pos_ind = 0;
  size_t vel_ind =
    (has_position_command_interface_) ? pos_ind + has_velocity_command_interface_ : pos_ind;
  size_t acc_ind = vel_ind + has_acceleration_command_interface_;
  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind)
  {
    if (has_position_command_interface_)
    {
      command_interfaces_[pos_ind * num_joints_ + joint_ind].set_value(
        state_commanded.positions[joint_ind]);
    }
    if (has_velocity_command_interface_)
    {
      command_interfaces_[vel_ind * num_joints_ + joint_ind].set_value(
        state_commanded.velocities[joint_ind]);
    }
    if (has_acceleration_command_interface_)
    {
      command_interfaces_[acc_ind * num_joints_ + joint_ind].set_value(
        state_commanded.accelerations[joint_ind]);
    }
  }
  last_commanded_ = state_commanded;
}

void AdmittanceController::read_state_reference_interfaces(
  trajectory_msgs::msg::JointTrajectoryPoint & state_reference)
{
  // TODO(destogl): check why is this here?

  // if any interface has nan values, assume state_reference is the last set reference
  for (size_t i = 0; i < num_joints_; ++i)
  {
    for (const auto & interface : admittance_->parameters_.chainable_command_interfaces)
    {
      // update position
      if (interface == hardware_interface::HW_IF_POSITION)
      {
        if (std::isnan(position_reference_[i]))
        {
          position_reference_[i].get() = last_reference_.positions[i];
        }
        state_reference.positions[i] = position_reference_[i];
      }

      // update velocity
      if (interface == hardware_interface::HW_IF_VELOCITY)
      {
        if (std::isnan(velocity_reference_[i]))
        {
          velocity_reference_[i].get() = last_reference_.velocities[i];
        }
        state_reference.velocities[i] = velocity_reference_[i];
      }
    }
  }

  last_reference_.positions = state_reference.positions;
  last_reference_.velocities = state_reference.velocities;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController, controller_interface::ChainableControllerInterface)
