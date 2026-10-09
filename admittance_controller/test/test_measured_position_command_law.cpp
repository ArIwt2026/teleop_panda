// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "admittance_controller/measured_position_command_law.hpp"

namespace admittance_controller
{
class MeasuredPositionCommandLawTest : public ::testing::Test
{
protected:
  using Joint = MeasuredPositionCommandLaw::JointVector;
  using Cartesian = MeasuredPositionCommandLaw::CartesianVector;
  using Jacobian = MeasuredPositionCommandLaw::Jacobian;
  MeasuredPositionCommandLaw law;
  MeasuredPositionCommandConfig config;
  Joint measured = Joint::Constant(0.3);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Jacobian jacobian = Jacobian::Identity();
  Cartesian force = Cartesian::Zero();
  Cartesian mass = Cartesian::Constant(0.01);
  Cartesian damping = Cartesian::Zero();
  Cartesian stiffness = Cartesian::Zero();

  void SetUp() override
  {
    // Formula tests isolate the force/target calculation. Dedicated tests below
    // exercise the operational reference limits.
    config.max_joint_velocity.setConstant(1e4);
    config.max_joint_acceleration.setConstant(1e8);
  }

  bool step(double dt = 0.01)
  {
    return law.update(measured, pose, jacobian, force, mass, damping, stiffness, dt, config);
  }
};

TEST_F(MeasuredPositionCommandLawTest, RetainsForceToTargetFormulaAndGainOrder)
{
  config.joint_position_tau = 0.0;
  force << 1, -2, 3, -4, 5, -6;
  ASSERT_TRUE(step());
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    const double clipped = std::clamp(force[i], -2.0, 2.0);
    const double expected_velocity =
      config.joint_gains[i] * clipped * config.cartesian_gains[i] / 1.04;
    EXPECT_NEAR(law.velocity[i], expected_velocity, 1e-12);
    EXPECT_NEAR(law.command[i], measured[i] + expected_velocity * 0.01, 1e-12);
  }
  EXPECT_DOUBLE_EQ(law.command[6], measured[6]);
}

TEST_F(MeasuredPositionCommandLawTest, BlockedRobotDoesNotAccumulatePositionOrVelocity)
{
  config.joint_position_tau = 0.0;
  force[0] = 1.0;
  ASSERT_TRUE(step());
  const Joint first_command = law.command;
  for (int i = 0; i < 10000; ++i)
  {
    ASSERT_TRUE(step());
  }
  EXPECT_TRUE(law.command.isApprox(first_command, 1e-12));
  EXPECT_TRUE(law.velocity.isZero());
  force.setZero();
  ASSERT_TRUE(step());
  EXPECT_TRUE(law.command.isApprox(measured));
  ASSERT_TRUE(step());
  EXPECT_TRUE(law.velocity.isZero());
}

TEST_F(MeasuredPositionCommandLawTest, PositionStepFollowsMeasuredJoints)
{
  config.joint_position_tau = 0.0;
  force[1] = 1.0;
  ASSERT_TRUE(step());
  const Joint offset = law.command - measured;
  measured[1] += 0.8;
  ASSERT_TRUE(step());
  EXPECT_TRUE((law.command - measured).isApprox(offset, 1e-12));
}

TEST_F(MeasuredPositionCommandLawTest, RunsAt100HzInside1kHzLoop)
{
  config.joint_position_tau = 0.0;
  force[0] = 1.0;
  for (int i = 0; i < 9; ++i)
  {
    ASSERT_TRUE(step(0.001));
    EXPECT_TRUE(law.command.isApprox(measured));
  }
  ASSERT_TRUE(step(0.001));
  const double expected = measured[0] + 5.0 / 1.04 * 0.01;
  EXPECT_NEAR(law.command[0], expected, 1e-12);
  force.setZero();
  for (int i = 0; i < 9; ++i)
  {
    ASSERT_TRUE(step(0.001));
    EXPECT_NEAR(law.command[0], expected, 1e-12);
  }
  ASSERT_TRUE(step(0.001));
  EXPECT_TRUE(law.command.isApprox(measured));
}

TEST_F(MeasuredPositionCommandLawTest, DampingUsesMeasuredMotion)
{
  config.joint_position_tau = 0.0;
  damping[0] = 1.0;
  ASSERT_TRUE(step());
  pose.translation()[0] = 0.001;  // measured 0.1 m/s
  ASSERT_TRUE(step());
  EXPECT_NEAR(law.velocity[0], -0.1 * 5.0 / 1.04, 1e-12);
  ASSERT_TRUE(step());  // no further motion; no stored damping velocity
  EXPECT_TRUE(law.command.isApprox(measured));
  // Telemetry describes the return of the reference, not the raw target increment.
  EXPECT_NEAR(law.velocity[0], 0.1 * 5.0 / 1.04, 1e-12);
}

TEST_F(MeasuredPositionCommandLawTest, AngularDampingOpposesVelocityInBaseFrame)
{
  config.joint_position_tau = 0.0;
  damping[5] = 1.0;
  pose.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
  ASSERT_TRUE(step());
  pose.linear() = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()).toRotationMatrix() * pose.linear();
  ASSERT_TRUE(step());
  // Positive base-Z rotation at 1 rad/s generates negative base-Z damping.
  EXPECT_NEAR(law.velocity[5], -1.0 * 6.0 * 2.0 / 1.04, 1e-12);
  EXPECT_NEAR(law.velocity[3], 0.0, 1e-12);
  EXPECT_NEAR(law.velocity[4], 0.0, 1e-12);
}

TEST_F(MeasuredPositionCommandLawTest, SingularJacobianProducesFiniteBoundedMotion)
{
  config.joint_position_tau = 0.0;
  jacobian.setZero();
  jacobian(0, 0) = 1e-9;
  force.setConstant(100.0);
  ASSERT_TRUE(step());
  EXPECT_TRUE(law.command.allFinite());
  EXPECT_LT(law.velocity.norm(), 1e-5);
  jacobian.setZero();
  ASSERT_TRUE(step());
  EXPECT_TRUE(law.command.isApprox(measured));
  ASSERT_TRUE(step());
  EXPECT_TRUE(law.velocity.isZero());
}

TEST_F(MeasuredPositionCommandLawTest, CapsFinalReferenceDuringForceReversalsAndTimingJitter)
{
  config.max_joint_velocity.setConstant(0.4);
  config.max_joint_acceleration.setConstant(2.0);
  for (int i = 0; i < 6000; ++i)
  {
    force.setConstant(i % 200 < 100 ? 100.0 : -100.0);
    const double dt = i % 2 == 0 ? 0.0007 : 0.0013;
    const Joint previous_command = i == 0 ? measured : law.command;
    const Joint previous_velocity = law.velocity;
    ASSERT_TRUE(step(dt));
    EXPECT_LE(law.velocity.cwiseAbs().maxCoeff(), 0.4 + 1e-10);
    EXPECT_LE(law.acceleration.cwiseAbs().maxCoeff(), 2.0 + 1e-10);
    EXPECT_TRUE(((law.command - previous_command) / dt).isApprox(law.velocity, 1e-8));
    EXPECT_TRUE(((law.velocity - previous_velocity) / dt).isApprox(law.acceleration, 1e-10));
  }
}

TEST_F(MeasuredPositionCommandLawTest, LimitedReferenceSettlesWithoutAccumulatingOffset)
{
  config.max_joint_velocity.setConstant(0.2);
  config.max_joint_acceleration.setConstant(0.5);
  config.cartesian_gains.setOnes();
  config.joint_gains.setOnes();
  force[0] = 1.0;
  for (int i = 0; i < 5000; ++i) ASSERT_TRUE(step(0.001));
  EXPECT_NEAR(law.command[0], measured[0] + 0.01 / 1.04, 1e-8);
  EXPECT_NEAR(law.velocity.norm(), 0.0, 1e-6);
  force.setZero();
  for (int i = 0; i < 5000; ++i) ASSERT_TRUE(step(0.001));
  EXPECT_TRUE(law.command.isApprox(measured, 1e-8));
  EXPECT_NEAR(law.velocity.norm(), 0.0, 1e-6);
}

TEST_F(MeasuredPositionCommandLawTest, AngularStiffnessOpposesBaseFrameDisplacement)
{
  config.joint_position_tau = 0.0;
  stiffness[5] = 1.0;
  pose.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
  ASSERT_TRUE(step());
  pose.linear() = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()).toRotationMatrix() * pose.linear();
  ASSERT_TRUE(step());
  EXPECT_NEAR(law.velocity[5], -0.01 * 6.0 * 2.0 / 1.04, 1e-12);
}

TEST_F(MeasuredPositionCommandLawTest, RejectsInvalidReferenceLimits)
{
  config.max_joint_velocity[0] = 0.0;
  EXPECT_FALSE(step());
  config.max_joint_velocity[0] = 1.0;
  config.max_joint_acceleration[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(step());
}

TEST_F(MeasuredPositionCommandLawTest, JointCommandFilterMatchesKukaTimeConstant)
{
  force[0] = 1.0;
  ASSERT_TRUE(step());
  const double alpha = 1.0 - std::exp(-0.01 / 0.04);
  EXPECT_NEAR(law.command[0], measured[0] + alpha * 5.0 / 1.04 * 0.01, 1e-12);
  force.setZero();
  ASSERT_TRUE(step());
  EXPECT_NEAR(law.command[0], measured[0] + (1.0 - alpha) * alpha * 5.0 / 1.04 * 0.01, 1e-12);
}

TEST_F(MeasuredPositionCommandLawTest, ResetStartsAtCurrentPoseAndClearsMotion)
{
  force.setOnes();
  ASSERT_TRUE(step());
  law.reset();
  force.setZero();
  stiffness.setOnes();
  pose.translation().setConstant(0.8);
  measured.setConstant(-0.4);
  ASSERT_TRUE(step());
  EXPECT_TRUE(law.command.isApprox(measured));
  EXPECT_TRUE(law.velocity.isZero());
}

TEST_F(MeasuredPositionCommandLawTest, RejectsInvalidTimingAndWrench)
{
  EXPECT_FALSE(step(0.0));
  EXPECT_FALSE(step(-0.01));
  EXPECT_FALSE(step(std::numeric_limits<double>::quiet_NaN()));
  force[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(step());
}
}  // namespace admittance_controller
