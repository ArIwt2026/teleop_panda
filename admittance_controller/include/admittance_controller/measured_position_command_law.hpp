// Copyright (c) 2026
// SPDX-License-Identifier: Apache-2.0
// Port of the local lbr_ros2_control AdmittanceController / InvJacCtrlImpl.
#ifndef ADMITTANCE_CONTROLLER__MEASURED_POSITION_COMMAND_LAW_HPP_
#define ADMITTANCE_CONTROLLER__MEASURED_POSITION_COMMAND_LAW_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace admittance_controller
{
struct MeasuredPositionCommandConfig
{
  double update_rate = 100.0;
  double sample_time = 0.01;
  double joint_position_tau = 0.04;
  double damping = 0.2;
  double max_linear_velocity = 2.0;
  double max_angular_velocity = 2.0;
  Eigen::Matrix<double, 7, 1> joint_gains =
    (Eigen::Matrix<double, 7, 1>() << 1, 1, 1, 1, 2, 2, 2).finished();
  Eigen::Matrix<double, 6, 1> cartesian_gains =
    (Eigen::Matrix<double, 6, 1>() << 5, 5, 3, 6, 6, 6).finished();
};

// The outer controller is called at the hardware rate. Only the admittance / SVD
// calculation runs at update_rate; the joint command filter runs every call.
class MeasuredPositionCommandLaw
{
public:
  using JointVector = Eigen::Matrix<double, 7, 1>;
  using CartesianVector = Eigen::Matrix<double, 6, 1>;
  using Jacobian = Eigen::Matrix<double, 6, 7>;

  void reset()
  {
    initialized_ = false;
    elapsed_ = 0.0;
    command.setZero();
    velocity.setZero();
    acceleration.setZero();
  }

  // The first call also needs a valid Jacobian for input validation/telemetry.
  bool needs_jacobian(double dt, const MeasuredPositionCommandConfig & config) const
  {
    return !initialized_ || elapsed_ + dt + 1e-12 >= 1.0 / config.update_rate;
  }

  bool update(
    const JointVector & measured, const Eigen::Isometry3d & pose, const Jacobian & jacobian,
    const CartesianVector & force, const CartesianVector & mass, const CartesianVector & damping,
    const CartesianVector & stiffness, double dt, const MeasuredPositionCommandConfig & config)
  {
    if (
      !std::isfinite(dt) || dt <= 0.0 || !measured.allFinite() || !pose.matrix().allFinite() ||
      !jacobian.allFinite() || !force.allFinite())
    {
      return false;
    }
    if (!initialized_)
    {
      initial_pose = pose;
      previous_pose_ = pose;
      target_ = measured;
      command = measured;
      initialized_ = true;
    }
    elapsed_ += dt;
    if (elapsed_ + 1e-12 >= 1.0 / config.update_rate)
    {
      const double controller_period = elapsed_;
      CartesianVector displacement, measured_motion;
      displacement.head<3>() = pose.translation() - initial_pose.translation();
      measured_motion.head<3>() =
        (pose.translation() - previous_pose_.translation()) / controller_period;
      const Eigen::Quaterniond rotation(pose.rotation());
      const Eigen::AngleAxisd delta(
        rotation.inverse() * Eigen::Quaterniond(initial_pose.rotation()));
      const Eigen::AngleAxisd increment(
        rotation.inverse() * Eigen::Quaterniond(previous_pose_.rotation()));
      displacement.tail<3>() = delta.axis() * delta.angle();
      // Match the KUKA source literally: its angular damping input is an angle
      // increment, without division by the controller period.
      measured_motion.tail<3>() = increment.axis() * increment.angle();
      const CartesianVector ddx =
        (force - damping.cwiseProduct(measured_motion) - stiffness.cwiseProduct(displacement))
          .cwiseQuotient(mass);

      // KUKA overwrites this increment each cycle; it does not integrate stored velocity.
      CartesianVector twist = ddx * controller_period;
      for (Eigen::Index i = 0; i < 6; ++i)
      {
        const double limit = i < 3 ? config.max_linear_velocity : config.max_angular_velocity;
        twist[i] = std::clamp(twist[i], -limit, limit) * config.cartesian_gains[i];
      }
      const Eigen::JacobiSVD<Jacobian> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix<double, 7, 6> inverse_values = Eigen::Matrix<double, 7, 6>::Zero();
      for (Eigen::Index i = 0; i < 6; ++i)
      {
        const double s = svd.singularValues()[i];
        inverse_values(i, i) = s / (s * s + config.damping * config.damping);
      }
      velocity = config.joint_gains.cwiseProduct(
        svd.matrixV() * inverse_values * svd.matrixU().adjoint() * twist);
      acceleration = velocity / controller_period;
      target_ = measured + velocity * config.sample_time;
      if (!target_.allFinite())
      {
        return false;
      }
      previous_pose_ = pose;
      elapsed_ = 0.0;
    }

    // Same exponential joint-position filter as KUKA's FRI command interface.
    const double alpha = config.joint_position_tau > 0.0
      ? 1.0 - std::exp(-dt / config.joint_position_tau) : 1.0;
    command += alpha * (target_ - command);
    return command.allFinite();
  }

  JointVector command = JointVector::Zero();
  JointVector velocity = JointVector::Zero();
  JointVector acceleration = JointVector::Zero();
  Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();

private:
  bool initialized_ = false;
  double elapsed_ = 0.0;
  JointVector target_ = JointVector::Zero();
  Eigen::Isometry3d previous_pose_ = Eigen::Isometry3d::Identity();
};
}  // namespace admittance_controller
#endif  // ADMITTANCE_CONTROLLER__MEASURED_POSITION_COMMAND_LAW_HPP_
