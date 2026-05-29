#include <cmath>

#include <gtest/gtest.h>

#include "robot_admittance_control/tool_frame_admittance_math.h"

namespace
{

constexpr double kTolerance = 1e-9;

void expectVectorNear(const Eigen::Vector3d & actual, const Eigen::Vector3d & expected)
{
  EXPECT_NEAR(actual.x(), expected.x(), kTolerance);
  EXPECT_NEAR(actual.y(), expected.y(), kTolerance);
  EXPECT_NEAR(actual.z(), expected.z(), kTolerance);
}

}  // namespace

TEST(ToolFrameAdmittanceMath, LocalTranslationIsRotatedIntoCommandBaseFrame)
{
  const Eigen::Vector3d reference_position(1.0, 2.0, 3.0);
  const Eigen::Quaterniond reference_orientation(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

  Eigen::Matrix<double, 6, 1> local_displacement;
  local_displacement << 0.10, 0.0, 0.0, 0.0, 0.0, 0.0;

  const auto command = robot_admittance_control::composeToolFramePoseCommand(
    reference_position,
    reference_orientation,
    local_displacement);

  expectVectorNear(command.position, Eigen::Vector3d(1.0, 2.10, 3.0));
}

TEST(ToolFrameAdmittanceMath, LocalRotationIsRightMultipliedOntoReferenceOrientation)
{
  const Eigen::Vector3d reference_position(0.0, 0.0, 0.0);
  const Eigen::Quaterniond reference_orientation(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

  Eigen::Matrix<double, 6, 1> local_displacement;
  local_displacement << 0.0, 0.0, 0.0, M_PI / 2.0, 0.0, 0.0;

  const auto command = robot_admittance_control::composeToolFramePoseCommand(
    reference_position,
    reference_orientation,
    local_displacement);

  const Eigen::Vector3d rotated_y = command.orientation * Eigen::Vector3d::UnitY();
  expectVectorNear(rotated_y, Eigen::Vector3d(0.0, 0.0, 1.0));

  const Eigen::Vector3d rotated_z = command.orientation * Eigen::Vector3d::UnitZ();
  expectVectorNear(rotated_z, Eigen::Vector3d(1.0, 0.0, 0.0));
}

TEST(ToolFrameAdmittanceMath, ToolFrameStateIsReexpressedWhenToolFrameRotates)
{
  const Eigen::Quaterniond previous_orientation(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond current_orientation = Eigen::Quaterniond::Identity();

  Eigen::Matrix<double, 6, 1> previous_local_state;
  previous_local_state << 0.10, 0.0, 0.0, 0.0, 0.20, 0.0;

  const auto current_local_state = robot_admittance_control::reexpressToolFrameVector(
    previous_orientation,
    current_orientation,
    previous_local_state);

  EXPECT_NEAR(current_local_state(0), 0.0, kTolerance);
  EXPECT_NEAR(current_local_state(1), 0.10, kTolerance);
  EXPECT_NEAR(current_local_state(2), 0.0, kTolerance);
  EXPECT_NEAR(current_local_state(3), -0.20, kTolerance);
  EXPECT_NEAR(current_local_state(4), 0.0, kTolerance);
  EXPECT_NEAR(current_local_state(5), 0.0, kTolerance);
}
