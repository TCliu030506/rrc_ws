#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloudslam_cpp {

  using ConcavePathPointCloud = pcl::PointCloud < pcl::PointXYZ >;

  struct ConcavePathParams
  {
    double probe_radial_length = 0.030;
    double radial_step = 0.020;
    double boundary_margin = 0.003;
    double r_end = 0.008;
    double scan_point_spacing = 0.003;
    double delta_phi_min_deg = 0.5;
    double delta_phi_max_deg = 5.0;
    double r_fit = 0.012;
    double phi_fit_deg = 5.0;
    int min_fit_points = 10;
    double normal_fit_radius = 0.015;
    int min_normal_fit_points = 10;
    double normal_smoothing_alpha = 0.2;
    double feed_step = 0.003;
    bool enable_transition_points = true;
    bool tool_z_negative_normal = true;
    int min_total_path_points = 20;
    int min_valid_points_per_layer = 8;
    double surface_clearance = 0.005;
  };

  struct ConcaveWorkpiecePathPoint
  {
    Eigen::Vector3d surface_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d rotation_workpiece = Eigen::Matrix3d::Identity();
    double r = 0.0;
    double theta = 0.0;
    double phi = 0.0;
    int layer_id = 0;
    int point_id = 0;
  };

  struct ConcavePathResult
  {
    std::vector < Eigen::Matrix < double, 6, 1 >> base_poses;
    std::vector < ConcaveWorkpiecePathPoint > workpiece_points;
    size_t layer_count = 0;
    size_t total_candidate_points = 0;
    size_t valid_path_points = 0;
    size_t skipped_theta_fit_points = 0;
    size_t skipped_normal_fit_points = 0;
    size_t skipped_orientation_points = 0;
    double r_min = 0.0;
    double r_max = 0.0;
    double r_95 = 0.0;
  };

  bool generate_concave_path(
    const ConcavePathPointCloud::ConstPtr & cloud_workpiece,
    const Eigen::Matrix4d & T_base_from_workpiece,
    const ConcavePathParams & params,
    const Eigen::Vector3d & probe_offset,
    const Eigen::Matrix3d & tool_mount_rotation,
    ConcavePathResult * result);

}  // namespace pointcloudslam_cpp
