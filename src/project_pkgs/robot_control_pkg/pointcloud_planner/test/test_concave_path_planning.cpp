#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "pointcloud_planner/concave_path_planning.h"

namespace
{

double layer_roughness(
  const std::vector<pointcloudslam_cpp::ConcaveWorkpiecePathPoint> & points)
{
  double roughness_sum = 0.0;
  size_t roughness_count = 0;
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    if (points[i - 1].is_transition || points[i].is_transition ||
      points[i + 1].is_transition)
    {
      continue;
    }
    if (points[i - 1].layer_id != points[i].layer_id ||
      points[i + 1].layer_id != points[i].layer_id)
    {
      continue;
    }
    roughness_sum += (
      points[i + 1].surface_point -
      2.0 * points[i].surface_point +
      points[i - 1].surface_point).norm();
    ++roughness_count;
  }
  return roughness_count > 0 ? roughness_sum / static_cast<double>(roughness_count) : 0.0;
}

}  // namespace

int main()
{
  pointcloudslam_cpp::ConcavePathPointCloud::Ptr cloud(new pointcloudslam_cpp::ConcavePathPointCloud);
  for (int ix = -35; ix <= 35; ++ix) {
    for (int iy = -35; iy <= 35; ++iy) {
      const double x = static_cast<double>(ix) * 0.002;
      const double y = static_cast<double>(iy) * 0.002;
      const double rho = std::sqrt(x * x + y * y);
      if (rho > 0.070 || rho < 0.004) {
        continue;
      }

      pcl::PointXYZ point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      const double deterministic_noise =
        0.0015 * std::sin(45.0 * x) * std::cos(55.0 * y);
      point.z = static_cast<float>(-0.20 * rho * rho + deterministic_noise);
      cloud->push_back(point);
    }
  }

  pointcloudslam_cpp::ConcavePathParams params;
  params.probe_radial_length = 0.020;
  params.radial_step = 0.012;
  params.boundary_margin = 0.002;
  params.r_end = 0.010;
  params.scan_point_spacing = 0.006;
  params.r_fit = 0.006;
  params.phi_fit_deg = 8.0;
  params.min_fit_points = 4;
  params.normal_fit_radius = 0.008;
  params.min_normal_fit_points = 4;
  params.min_total_path_points = 20;
  params.min_valid_points_per_layer = 4;
  params.enable_transition_points = true;
  params.enable_path_smoothing = false;

  pointcloudslam_cpp::ConcavePathResult result;
  const bool ok = pointcloudslam_cpp::generate_concave_path(
    cloud,
    Eigen::Matrix4d::Identity(),
    params,
    Eigen::Vector3d::Zero(),
    Eigen::Matrix3d::Identity(),
    &result);

  assert(ok);
  assert(result.base_poses.size() >= static_cast<size_t>(params.min_total_path_points));
  assert(result.workpiece_points.size() == result.base_poses.size());
  assert(result.layer_count >= 2);
  assert(result.skipped_theta_fit_points > 0 || result.valid_path_points > 0);

  for (const auto & point : result.workpiece_points) {
    assert(std::isfinite(point.surface_point(0)));
    assert(std::isfinite(point.normal(2)));
    assert(point.normal(2) > 0.0);
    assert(std::abs(point.rotation_workpiece.determinant() - 1.0) < 1e-6);
  }

  params.align_x_axis_to_scan_direction = false;
  pointcloudslam_cpp::ConcavePathResult continuous_result;
  const bool continuous_ok = pointcloudslam_cpp::generate_concave_path(
    cloud,
    Eigen::Matrix4d::Identity(),
    params,
    Eigen::Vector3d::Zero(),
    Eigen::Matrix3d::Identity(),
    &continuous_result);
  assert(continuous_ok);

  double max_neighbor_angle = 0.0;
  for (size_t i = 1; i < continuous_result.workpiece_points.size(); ++i) {
    const Eigen::Quaterniond previous(
      continuous_result.workpiece_points[i - 1].rotation_workpiece);
    const Eigen::Quaterniond current(
      continuous_result.workpiece_points[i].rotation_workpiece);
    const double dot = std::abs(previous.dot(current));
    const double angle = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));
    max_neighbor_angle = std::max(max_neighbor_angle, angle);
  }
  if (max_neighbor_angle > 0.5) {
    std::cerr << "Expected continuous orientation, max_neighbor_angle_deg="
              << max_neighbor_angle * 180.0 / M_PI << "\n";
    return 1;
  }

  pointcloudslam_cpp::ConcavePathParams smoothed_params = params;
  smoothed_params.enable_path_smoothing = true;
  smoothed_params.position_smoothing_window = 9;
  smoothed_params.position_smoothing_order = 2;
  smoothed_params.position_smoothing_passes = 2;
  smoothed_params.position_smoothing_max_deviation = 0.002;
  smoothed_params.normal_smoothing_window = 9;
  smoothed_params.normal_smoothing_passes = 2;
  smoothed_params.enable_quintic_transition = true;

  pointcloudslam_cpp::ConcavePathResult smoothed_result;
  const bool smoothed_ok = pointcloudslam_cpp::generate_concave_path(
    cloud,
    Eigen::Matrix4d::Identity(),
    smoothed_params,
    Eigen::Vector3d::Zero(),
    Eigen::Matrix3d::Identity(),
    &smoothed_result);
  assert(smoothed_ok);
  assert(smoothed_result.workpiece_points.size() == result.workpiece_points.size());
  assert(layer_roughness(smoothed_result.workpiece_points) <
    layer_roughness(result.workpiece_points));

  bool has_transition_point = false;
  for (size_t i = 0; i < smoothed_result.workpiece_points.size(); ++i) {
    const auto & smoothed_point = smoothed_result.workpiece_points[i];
    const auto & raw_point = result.workpiece_points[i];
    if (!smoothed_point.is_transition) {
      assert(
        (smoothed_point.surface_point - raw_point.surface_point).norm() <=
        smoothed_params.position_smoothing_max_deviation + 1e-9);
    } else {
      has_transition_point = true;
    }
    assert(std::abs(smoothed_point.rotation_workpiece.determinant() - 1.0) < 1e-6);
  }
  assert(has_transition_point);

  return 0;
}
