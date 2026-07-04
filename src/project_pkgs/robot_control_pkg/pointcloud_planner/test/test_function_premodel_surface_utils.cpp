#include <cmath>
#include <iostream>

#include "pointcloud_planner/premodel_surface_utils.h"

namespace
{

bool near(double actual, double expected, double tolerance)
{
  return std::abs(actual - expected) <= tolerance;
}

bool expect_z(double radius_mm, double expected_z_mm)
{
  double actual_z_mm = 0.0;
  if (!pointcloudslam_cpp::function_premodel_surface_z_mm(radius_mm, &actual_z_mm)) {
    std::cerr << "radius_mm=" << radius_mm << " did not evaluate\n";
    return false;
  }
  if (!near(actual_z_mm, expected_z_mm, 1e-6)) {
    std::cerr << "radius_mm=" << radius_mm
              << " expected_z_mm=" << expected_z_mm
              << " actual_z_mm=" << actual_z_mm << "\n";
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  if (!expect_z(0.0, 0.0) ||
    !expect_z(205.98, 26.0) ||
    !expect_z(222.90, 17.0))
  {
    return 1;
  }

  double ignored_z_mm = 0.0;
  if (pointcloudslam_cpp::function_premodel_surface_z_mm(230.0, &ignored_z_mm)) {
    std::cerr << "radius_mm=230 unexpectedly evaluated\n";
    return 1;
  }

  const auto cloud = pointcloudslam_cpp::build_function_premodel_surface_cloud(0.05);
  if (!cloud || cloud->points.empty()) {
    std::cerr << "function premodel cloud is empty\n";
    return 1;
  }

  double max_radius_m = 0.0;
  double min_z_m = 1.0;
  double max_z_m = -1.0;
  for (const auto & point : cloud->points) {
    const double radius_m = std::hypot(point.x, point.y);
    max_radius_m = std::max(max_radius_m, radius_m);
    min_z_m = std::min(min_z_m, static_cast<double>(point.z));
    max_z_m = std::max(max_z_m, static_cast<double>(point.z));
  }

  if (max_radius_m > 0.22290 + 1e-6 ||
    min_z_m < -1e-6 ||
    max_z_m > 0.02600 + 1e-6)
  {
    std::cerr << "unexpected cloud bounds: max_radius_m=" << max_radius_m
              << " min_z_m=" << min_z_m
              << " max_z_m=" << max_z_m << "\n";
    return 1;
  }

  return 0;
}
