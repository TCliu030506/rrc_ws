#pragma once

#include <cstddef>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloudslam_cpp {

  using PointCloudXYZ = pcl::PointCloud < pcl::PointXYZ >;

  struct PlaneFitResult
  {
    Eigen::Vector4d coefficients = Eigen::Vector4d::Zero();
    int inlier_count = 0;
  };

  bool fit_plane_ransac(
    const PointCloudXYZ::ConstPtr & cloud_in,
    double distance_threshold,
    int min_inliers,
    PlaneFitResult * result);

  PointCloudXYZ::Ptr extract_corner_candidates_xy(
    const PointCloudXYZ::ConstPtr & cloud_in,
    double patch_ratio);

  void orient_plane_toward_workpiece(PointCloudXYZ::ConstPtr cloud_in, Eigen::Vector4d * plane);

  PointCloudXYZ::Ptr filter_by_platform_height(
    const PointCloudXYZ::ConstPtr & cloud_in,
    const Eigen::Vector4d & plane,
    double height_min,
    double height_max);

  PointCloudXYZ::Ptr filter_by_normal_dot(
    const PointCloudXYZ::ConstPtr & cloud_in,
    const Eigen::Vector3d & platform_normal,
    int normal_k,
    double normal_dot_min);

  PointCloudXYZ::Ptr keep_largest_cluster(
    const PointCloudXYZ::ConstPtr & cloud_in,
    double cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size,
    size_t * cluster_count_out);

}  // namespace pointcloudslam_cpp
