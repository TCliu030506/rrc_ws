#pragma once

#include <cstddef>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloudslam_cpp {

  using WorkpiecePointCloudXYZ = pcl::PointCloud < pcl::PointXYZ >;

  struct WorkpieceFrameParams
  {
    double center_percentile_low = 5.0;
    double center_percentile_high = 95.0;
    double normal_fit_radius = 0.03;
    int normal_fit_k = 100;
    int min_normal_fit_points = 50;
    double axis_projection_min_norm = 0.1;
    int min_roi_points = 500;
  };

  struct WorkpieceFrameResult
  {
    Eigen::Matrix4d T_base_from_workpiece = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_workpiece_from_base = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
    Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();
    Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    size_t roi_point_count = 0;
    size_t normal_fit_point_count = 0;
    double det_R = 1.0;
    double dot_xy = 0.0;
    double dot_xz = 0.0;
    double dot_yz = 0.0;
  };

  bool fit_workpiece_frame(
    const WorkpiecePointCloudXYZ::ConstPtr & cloud_in,
    const WorkpieceFrameParams & params,
    const Eigen::Vector3d & preferred_z_axis,
    WorkpieceFrameResult * result);

  WorkpiecePointCloudXYZ::Ptr transform_cloud_to_workpiece(
    const WorkpiecePointCloudXYZ::ConstPtr & cloud_in,
    const Eigen::Matrix4d & T_workpiece_from_base);

}  // namespace pointcloudslam_cpp
