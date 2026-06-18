#include "pointcloud_planner/camera_distance_filter.h"

#include <cmath>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {

bool expect_true(bool value, const char *message)
{
  if (!value) {
    std::cerr << message << std::endl;
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->push_back(pcl::PointXYZ(0.0F, 0.0F, 0.29F));
  cloud->push_back(pcl::PointXYZ(0.0F, 0.0F, 0.30F));
  cloud->push_back(pcl::PointXYZ(0.0F, 0.0F, 0.50F));
  cloud->push_back(pcl::PointXYZ(0.0F, 0.0F, 2.00F));
  cloud->push_back(pcl::PointXYZ(0.0F, 0.0F, 2.01F));
  cloud->push_back(pcl::PointXYZ(std::nanf(""), 0.0F, 0.50F));
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;

  const auto filtered =
    pointcloudslam_cpp::filter_camera_distance_range(cloud, 0.30, 2.00);

  bool ok = true;
  ok &= expect_true(filtered != nullptr, "filtered cloud should not be null");
  ok &= expect_true(filtered->size() == 3, "filtered cloud should keep exactly 3 points");
  ok &= expect_true(filtered->height == 1, "filtered cloud should be unorganized");
  ok &= expect_true(filtered->points[0].z == 0.30F, "threshold point should be kept");
  ok &= expect_true(filtered->points[1].z == 0.50F, "far point should be kept");
  ok &= expect_true(filtered->points[2].z == 2.00F, "max threshold point should be kept");
  return ok ? 0 : 1;
}
