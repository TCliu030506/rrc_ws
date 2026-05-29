#ifndef ROBOT_ADMITTANCE_CONTROL_TOOL_FRAME_ADMITTANCE_MATH_H
#define ROBOT_ADMITTANCE_CONTROL_TOOL_FRAME_ADMITTANCE_MATH_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_admittance_control
{

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct PoseCommand
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

inline Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle <= 1e-12) {
    return Eigen::Quaterniond::Identity();
  }

  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

inline PoseCommand composeToolFramePoseCommand(
  const Eigen::Vector3d & reference_position,
  const Eigen::Quaterniond & reference_orientation,
  const Vector6d & local_displacement)
{
  Eigen::Quaterniond reference = reference_orientation.normalized();
  const Eigen::Vector3d local_translation = local_displacement.segment<3>(0);
  const Eigen::Vector3d local_rotation = local_displacement.segment<3>(3);

  PoseCommand command;
  command.position = reference_position + reference * local_translation;
  command.orientation = reference * rotationVectorToQuaternion(local_rotation);
  command.orientation.normalize();
  return command;
}

inline Vector6d rotateToolFrameTwistToBase(
  const Eigen::Quaterniond & reference_orientation,
  const Vector6d & local_twist)
{
  const Eigen::Quaterniond reference = reference_orientation.normalized();
  Vector6d base_twist;
  base_twist.segment<3>(0) = reference * local_twist.segment<3>(0);
  base_twist.segment<3>(3) = reference * local_twist.segment<3>(3);
  return base_twist;
}

inline Vector6d reexpressToolFrameVector(
  const Eigen::Quaterniond & previous_orientation,
  const Eigen::Quaterniond & current_orientation,
  const Vector6d & previous_local_vector)
{
  const Eigen::Quaterniond previous = previous_orientation.normalized();
  const Eigen::Quaterniond current = current_orientation.normalized();
  const Eigen::Quaterniond previous_to_current_local = current.conjugate() * previous;

  Vector6d current_local_vector;
  current_local_vector.segment<3>(0) =
    previous_to_current_local * previous_local_vector.segment<3>(0);
  current_local_vector.segment<3>(3) =
    previous_to_current_local * previous_local_vector.segment<3>(3);
  return current_local_vector;
}

}  // namespace robot_admittance_control

#endif  // ROBOT_ADMITTANCE_CONTROL_TOOL_FRAME_ADMITTANCE_MATH_H
