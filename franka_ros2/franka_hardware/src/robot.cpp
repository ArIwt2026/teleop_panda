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
#include <algorithm>
#include <chrono>

#include <rclcpp/logging.hpp>

namespace franka_hardware {

namespace {
int64_t steadyTimeNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) : logger_(logger) {
  // Permit the requested generic-kernel operation. libfranka 0.9.2's
  // hasRealtimeKernel() returns an uninitialized bool when /sys/kernel/realtime
  // is absent, so it must not decide whether enforcement is enabled here.
  // kIgnore still attempts FIFO scheduling; it suppresses exceptions from the
  // scheduling/kernel checks, not robot communication or motion constraints.
  RCLCPP_INFO(logger,
              "libfranka realtime enforcement disabled (RealtimeConfig::kIgnore); "
              "realtime scheduling will still be attempted.");
  robot_ = std::make_unique<franka::Robot>(robot_ip, franka::RealtimeConfig::kIgnore);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  franka_hardware_model_ = std::make_unique<Model>(model_.get());
}

void Robot::write(const std::array<double, 7>& efforts) {
  command_buffer_.publish(TorqueCommand{efforts, steadyTimeNs()});
}

franka::RobotState Robot::read() {
  return state_buffer_.read();
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
    int64_t previous_callback_ns = 0;
    int64_t max_callback_ns = 0;
    int64_t max_callback_gap_ns = 0;
    int64_t max_command_age_ns = 0;
    int64_t terminal_callback_gap_ns = 0;
    uint64_t missed_robot_cycles = 0;
    double min_success_rate = 1.0;
    try {
      control_running_ = true;
      robot_->control(
          [this, &previous_callback_ns, &max_callback_ns, &max_callback_gap_ns,
           &max_command_age_ns, &missed_robot_cycles, &min_success_rate](
              const franka::RobotState& state, const franka::Duration& period) {
            const auto entered_ns = steadyTimeNs();
            if (previous_callback_ns != 0) {
              max_callback_gap_ns = std::max(max_callback_gap_ns, entered_ns - previous_callback_ns);
            }
            previous_callback_ns = entered_ns;
            if (period.toMSec() > 0) {
              min_success_rate = std::min(min_success_rate, state.control_command_success_rate);
              missed_robot_cycles += period.toMSec() - 1;
            }
            state_buffer_.publish(state);
            const auto& command = command_buffer_.read();
            if (command.written_at_ns > 0 && period.toMSec() > 0) {
              max_command_age_ns = std::max(max_command_age_ns, entered_ns - command.written_at_ns);
            }
            franka::Torques out(command.efforts);
            out.motion_finished = finish_;
            max_callback_ns = std::max(max_callback_ns, steadyTimeNs() - entered_ns);
            return out;
          },
          true, franka::kMaxCutoffFrequency);
    } catch (const franka::Exception& e) {
      if (previous_callback_ns != 0) {
        terminal_callback_gap_ns = steadyTimeNs() - previous_callback_ns;
      }
      error_message_ = e.what();
      faulted_ = true;
    }
    control_running_ = false;

    // Log after the realtime control loop exits, never from its callback.
    RCLCPP_INFO(logger_,
                "FCI timing: max callback %.1f us, max callback gap %.1f us, "
                "terminal gap %.1f us, max ROS torque age %.1f us, "
                "skipped robot cycles %llu, min success rate %.3f",
                max_callback_ns / 1000.0, max_callback_gap_ns / 1000.0,
                terminal_callback_gap_ns / 1000.0, max_command_age_ns / 1000.0,
                static_cast<unsigned long long>(missed_robot_cycles), min_success_rate);

    if (faulted_) {
      // START FALLBACK READING LOOP
      // This ensures that MoveIt/RViz continue to receive robot state updates
      // (e.g. while hand-guiding after a stop) even though control has ended.
      try {
        robot_->read([this](const franka::RobotState& state) {
          state_buffer_.publish(state);
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
        state_buffer_.publish(state);
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
