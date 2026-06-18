#ifndef POINTCLOUD_PLANNER_CAMERA_DISTANCE_FILTER_H_
#define POINTCLOUD_PLANNER_CAMERA_DISTANCE_FILTER_H_

#include <cmath>
#include <cstdint>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloudslam_cpp {

inline pcl::PointCloud<pcl::PointXYZ>::Ptr filter_camera_distance_range(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_in,
  double min_distance_m,
  double max_distance_m)
{
  auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (!cloud_in) {
    return cloud_out;
  }

  const double min_distance_sq = min_distance_m * min_distance_m;
  const double max_distance_sq = max_distance_m * max_distance_m;
  cloud_out->reserve(cloud_in->size());

  for (const auto & point : cloud_in->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    const double distance_sq =
      static_cast<double>(point.x) * static_cast<double>(point.x) +
      static_cast<double>(point.y) * static_cast<double>(point.y) +
      static_cast<double>(point.z) * static_cast<double>(point.z);
    if (distance_sq >= min_distance_sq && distance_sq <= max_distance_sq) {
      cloud_out->push_back(point);
    }
  }

  cloud_out->width = static_cast<std::uint32_t>(cloud_out->size());
  cloud_out->height = 1;
  cloud_out->is_dense = true;
  return cloud_out;
}

}  // namespace pointcloudslam_cpp

#endif  // POINTCLOUD_PLANNER_CAMERA_DISTANCE_FILTER_H_
