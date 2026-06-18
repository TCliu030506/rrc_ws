#include <cassert>
#include <cmath>
#include <iostream>

#include "pointcloud_planner/concave_path_planning.h"

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
      point.z = static_cast<float>(-0.20 * rho * rho);
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

  return 0;
}
