// Copyright (c) 2021 Franka Emika GmbH
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

#include <franka_hardware/robot.hpp>

#include <cassert>
#include <mutex>

#include <franka/control_tools.h>
#include <rclcpp/logging.hpp>

namespace franka_hardware {

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) {
  tau_command_.fill(0.);
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
  if (!franka::hasRealtimeKernel()) {
    rt_config = franka::RealtimeConfig::kIgnore;
    RCLCPP_WARN(
        logger,
        "You are not using a real-time kernel. Using a real-time kernel is strongly recommended!");
  }
  robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  franka_hardware_model_ = std::make_unique<Model>(model_.get());
}

void Robot::write(const std::array<double, 7>& efforts) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  tau_command_ = efforts;
}

franka::RobotState Robot::read() {
  std::lock_guard<std::mutex> lock(read_mutex_);
  return {current_state_};
}

franka_hardware::Model* Robot::getModel() {
  return franka_hardware_model_.get();
}

void Robot::stopRobot() {
  finish_ = true;
  if (control_thread_ && control_thread_->joinable()) {
    control_thread_->join();
  }
  finish_ = false;
  stopped_ = true;
  faulted_ = false;
}

void Robot::initializeTorqueControl() {
  if (control_thread_) {
    stopRobot();
  }
  stopped_ = false;
  faulted_ = false;
  error_message_.clear();
  const auto kTorqueControl = [this]() {
    try {
      control_running_ = true;
      robot_->control(
          [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
              std::lock_guard<std::mutex> lock(read_mutex_);
              current_state_ = state;
            }
            std::lock_guard<std::mutex> lock(write_mutex_);
            franka::Torques out(tau_command_);
            out.motion_finished = finish_;
            return out;
          },
          true, franka::kMaxCutoffFrequency);
    } catch (const franka::Exception& e) {
      error_message_ = e.what();
      faulted_ = true;
    }
    control_running_ = false;

    if (faulted_) {
      // START FALLBACK READING LOOP
      // This ensures that MoveIt/RViz continue to receive robot state updates
      // (e.g. while hand-guiding after a stop) even though control has ended.
      try {
        robot_->read([this](const franka::RobotState& state) {
          {
            std::lock_guard<std::mutex> lock(read_mutex_);
            current_state_ = state;
          }
          // Automatically clear faulted_ state and STOP the fallback loop if the 
          // robot is no longer in a stop/error state. This allows the hardware 
          // interface to restart a fresh control loop.
          if (state.robot_mode == franka::RobotMode::kIdle) {
            faulted_ = false;
            return false; 
          }
          return !finish_;
        });
      } catch (...) {
        // Fallback read loop failed (likely connection lost), thread will exit.
      }
    }
    stopped_ = true;
  };
  control_thread_ = std::make_unique<std::thread>(kTorqueControl);
}

void Robot::initializeContinuousReading() {
  if (control_thread_) {
    stopRobot();
  }
  stopped_ = false;
  faulted_ = false;
  error_message_.clear();
  const auto kReading = [this]() {
    try {
      robot_->read([this](const franka::RobotState& state) {
        {
          std::lock_guard<std::mutex> lock(read_mutex_);
          current_state_ = state;
        }
        return !finish_;
      });
    } catch (const franka::Exception& e) {
      error_message_ = e.what();
      faulted_ = true;
    }
    stopped_ = true;
  };
  control_thread_ = std::make_unique<std::thread>(kReading);
}

Robot::~Robot() {
  stopRobot();
}

bool Robot::isStopped() const {
  return stopped_;
}

bool Robot::hasError() const {
  return faulted_;
}

bool Robot::isControlRunning() const {
  return control_running_;
}

std::string Robot::getErrorMessage() const {
  return error_message_;
}
}  // namespace franka_hardware
