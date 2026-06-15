#include "pointcloud_planner/concave_roi_refinement.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace pointcloudslam_cpp
{
namespace
{

double point_plane_distance(const pcl::PointXYZ & point, const Eigen::Vector4d & plane)
{
  return plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3);
}

}  // namespace

bool fit_plane_ransac(
  const PointCloudXYZ::ConstPtr & cloud_in,
  double distance_threshold,
  int min_inliers,
  PlaneFitResult * result)
{
  if (!cloud_in || cloud_in->points.size() < 3 || result == nullptr) {
    return false;
  }

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setInputCloud(cloud_in);
  seg.segment(*inliers, *coefficients);

  if (coefficients->values.size() < 4 || static_cast<int>(inliers->indices.size()) < min_inliers) {
    return false;
  }

  Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  const double normal_norm = normal.norm();
  if (normal_norm < 1e-8) {
    return false;
  }

  result->coefficients << normal(0) / normal_norm,
    normal(1) / normal_norm,
    normal(2) / normal_norm,
    coefficients->values[3] / normal_norm;
  result->inlier_count = static_cast<int>(inliers->indices.size());
  return true;
}

PointCloudXYZ::Ptr extract_corner_candidates_xy(
  const PointCloudXYZ::ConstPtr & cloud_in,
  double patch_ratio)
{
  PointCloudXYZ::Ptr candidates(new PointCloudXYZ);
  if (!cloud_in || cloud_in->empty()) {
    return candidates;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & point : cloud_in->points) {
    min_x = std::min(min_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_x = std::max(max_x, static_cast<double>(point.x));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }

  patch_ratio = std::clamp(patch_ratio, 0.02, 0.45);
  const double patch_x = (max_x - min_x) * patch_ratio;
  const double patch_y = (max_y - min_y) * patch_ratio;
  if (patch_x <= 0.0 || patch_y <= 0.0) {
    return candidates;
  }

  for (const auto & point : cloud_in->points) {
    const bool in_x_corner = point.x <= min_x + patch_x || point.x >= max_x - patch_x;
    const bool in_y_corner = point.y <= min_y + patch_y || point.y >= max_y - patch_y;
    if (in_x_corner && in_y_corner) {
      candidates->push_back(point);
    }
  }
  return candidates;
}

void orient_plane_toward_workpiece(PointCloudXYZ::ConstPtr cloud_in, Eigen::Vector4d * plane)
{
  if (!cloud_in || !plane) {
    return;
  }

  int positive_count = 0;
  int negative_count = 0;
  for (const auto & point : cloud_in->points) {
    const double distance = point_plane_distance(point, *plane);
    if (distance > 1e-5) {
      ++positive_count;
    } else if (distance < -1e-5) {
      ++negative_count;
    }
  }

  if (negative_count > positive_count) {
    *plane *= -1.0;
  }
}

PointCloudXYZ::Ptr filter_by_platform_height(
  const PointCloudXYZ::ConstPtr & cloud_in,
  const Eigen::Vector4d & plane,
  double height_min,
  double height_max)
{
  PointCloudXYZ::Ptr filtered(new PointCloudXYZ);
  if (!cloud_in) {
    return filtered;
  }

  for (const auto & point : cloud_in->points) {
    const double height = point_plane_distance(point, plane);
    if (height > height_min && height < height_max) {
      filtered->push_back(point);
    }
  }
  return filtered;
}

PointCloudXYZ::Ptr filter_by_normal_dot(
  const PointCloudXYZ::ConstPtr & cloud_in,
  const Eigen::Vector3d & platform_normal,
  int normal_k,
  double normal_dot_min)
{
  PointCloudXYZ::Ptr filtered(new PointCloudXYZ);
  if (!cloud_in || cloud_in->points.size() < 3) {
    return filtered;
  }

  normal_k = std::clamp(normal_k, 3, static_cast<int>(cloud_in->points.size()));
  normal_dot_min = std::clamp(normal_dot_min, 0.0, 1.0);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normal_estimation.setInputCloud(cloud_in);
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setKSearch(normal_k);
  normal_estimation.compute(*normals);

  for (size_t i = 0; i < cloud_in->points.size() && i < normals->points.size(); ++i) {
    const auto & normal = normals->points[i];
    Eigen::Vector3d normal_vec(normal.normal_x, normal.normal_y, normal.normal_z);
    if (!std::isfinite(normal_vec(0)) || !std::isfinite(normal_vec(1)) ||
      !std::isfinite(normal_vec(2)))
    {
      continue;
    }
    const double norm = normal_vec.norm();
    if (norm < 1e-8) {
      continue;
    }
    normal_vec /= norm;
    if (std::abs(normal_vec.dot(platform_normal)) > normal_dot_min) {
      filtered->push_back(cloud_in->points[i]);
    }
  }
  return filtered;
}

PointCloudXYZ::Ptr keep_largest_cluster(
  const PointCloudXYZ::ConstPtr & cloud_in,
  double cluster_tolerance,
  int min_cluster_size,
  int max_cluster_size,
  size_t * cluster_count_out)
{
  PointCloudXYZ::Ptr largest_cluster(new PointCloudXYZ);
  if (cluster_count_out != nullptr) {
    *cluster_count_out = 0;
  }
  if (!cloud_in || cloud_in->empty()) {
    return largest_cluster;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
  extraction.setClusterTolerance(cluster_tolerance);
  extraction.setMinClusterSize(std::max(1, min_cluster_size));
  extraction.setMaxClusterSize(std::max(max_cluster_size, min_cluster_size));
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cloud_in);
  extraction.extract(cluster_indices);

  if (cluster_count_out != nullptr) {
    *cluster_count_out = cluster_indices.size();
  }
  if (cluster_indices.empty()) {
    return largest_cluster;
  }

  const auto largest_it = std::max_element(
    cluster_indices.begin(),
    cluster_indices.end(),
    [](const pcl::PointIndices & lhs, const pcl::PointIndices & rhs) {
      return lhs.indices.size() < rhs.indices.size();
    });
  for (const int index : largest_it->indices) {
    largest_cluster->push_back(cloud_in->points[index]);
  }
  return largest_cluster;
}

}  // namespace pointcloudslam_cpp
