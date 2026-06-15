#include "pointcloud_planner/concave_workpiece_frame.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pointcloudslam_cpp
{
namespace
{

bool is_finite_point(const pcl::PointXYZ & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

WorkpiecePointCloudXYZ::Ptr remove_nan_points(const WorkpiecePointCloudXYZ::ConstPtr & cloud_in)
{
  WorkpiecePointCloudXYZ::Ptr cleaned(new WorkpiecePointCloudXYZ);
  if (!cloud_in) {
    return cleaned;
  }

  cleaned->reserve(cloud_in->size());
  for (const auto & point : cloud_in->points) {
    if (is_finite_point(point)) {
      cleaned->push_back(point);
    }
  }
  return cleaned;
}

bool compute_centroid(const WorkpiecePointCloudXYZ::ConstPtr & cloud, Eigen::Vector3d * centroid)
{
  if (!cloud || cloud->empty() || centroid == nullptr) {
    return false;
  }

  Eigen::Vector4d centroid4 = Eigen::Vector4d::Zero();
  pcl::compute3DCentroid(*cloud, centroid4);
  *centroid = centroid4.head<3>();
  return true;
}

bool compute_pca_axes(
  const WorkpiecePointCloudXYZ::ConstPtr & cloud,
  const Eigen::Vector3d & centroid,
  Eigen::Matrix3d * axes)
{
  if (!cloud || cloud->size() < 3 || axes == nullptr) {
    return false;
  }

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : cloud->points) {
    const Eigen::Vector3d delta(point.x - centroid(0), point.y - centroid(1),
      point.z - centroid(2));
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<double>(cloud->size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return false;
  }

  axes->col(0) = solver.eigenvectors().col(2).normalized();
  axes->col(1) = solver.eigenvectors().col(1).normalized();
  axes->col(2) = solver.eigenvectors().col(0).normalized();
  return true;
}

double percentile(std::vector<double> values, double percent)
{
  if (values.empty()) {
    return 0.0;
  }

  std::sort(values.begin(), values.end());
  percent = std::clamp(percent, 0.0, 100.0);
  const double index = (percent / 100.0) * static_cast<double>(values.size() - 1);
  const size_t low = static_cast<size_t>(std::floor(index));
  const size_t high = static_cast<size_t>(std::ceil(index));
  if (low == high) {
    return values[low];
  }
  const double t = index - static_cast<double>(low);
  return values[low] * (1.0 - t) + values[high] * t;
}

bool choose_origin_on_surface(
  const WorkpiecePointCloudXYZ::ConstPtr & cloud,
  const Eigen::Vector3d & centroid,
  const Eigen::Matrix3d & global_axes,
  double percentile_low,
  double percentile_high,
  Eigen::Vector3d * origin)
{
  if (!cloud || cloud->empty() || origin == nullptr) {
    return false;
  }

  if (percentile_low > percentile_high) {
    std::swap(percentile_low, percentile_high);
  }
  std::vector<double> u_values;
  std::vector<double> v_values;
  u_values.reserve(cloud->size());
  v_values.reserve(cloud->size());

  for (const auto & point : cloud->points) {
    Eigen::Vector3d delta(point.x - centroid(0), point.y - centroid(1), point.z - centroid(2));
    u_values.push_back(delta.dot(global_axes.col(0)));
    v_values.push_back(delta.dot(global_axes.col(1)));
  }

  const double u0 = 0.5 *
    (percentile(u_values, percentile_low) + percentile(u_values, percentile_high));
  const double v0 = 0.5 *
    (percentile(v_values, percentile_low) + percentile(v_values, percentile_high));
  const Eigen::Vector3d projected_center = centroid + u0 * global_axes.col(0) + v0 *
    global_axes.col(1);

  double best_distance = std::numeric_limits<double>::max();
  Eigen::Vector3d best_point = projected_center;
  for (const auto & point : cloud->points) {
    Eigen::Vector3d candidate(point.x, point.y, point.z);
    const double distance = (candidate - projected_center).squaredNorm();
    if (distance < best_distance) {
      best_distance = distance;
      best_point = candidate;
    }
  }

  *origin = best_point;
  return true;
}

bool collect_normal_fit_points(
  const WorkpiecePointCloudXYZ::ConstPtr & cloud,
  const Eigen::Vector3d & origin,
  const WorkpieceFrameParams & params,
  WorkpiecePointCloudXYZ::Ptr * normal_points)
{
  if (!cloud || cloud->empty() || normal_points == nullptr) {
    return false;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);
  pcl::PointXYZ search_point;
  search_point.x = static_cast<float>(origin(0));
  search_point.y = static_cast<float>(origin(1));
  search_point.z = static_cast<float>(origin(2));

  std::vector<int> indices;
  std::vector<float> distances;
  if (params.normal_fit_radius > 0.0) {
    tree.radiusSearch(search_point, params.normal_fit_radius, indices, distances);
  }

  if (static_cast<int>(indices.size()) < params.min_normal_fit_points) {
    indices.clear();
    distances.clear();
    tree.nearestKSearch(search_point, std::max(3, params.normal_fit_k), indices, distances);
  }

  if (static_cast<int>(indices.size()) < params.min_normal_fit_points) {
    return false;
  }

  WorkpiecePointCloudXYZ::Ptr selected(new WorkpiecePointCloudXYZ);
  selected->reserve(indices.size());
  for (const int index : indices) {
    selected->push_back(cloud->points[index]);
  }
  *normal_points = selected;
  return true;
}

bool fit_local_normal(
  const WorkpiecePointCloudXYZ::ConstPtr & normal_points,
  const Eigen::Vector3d & preferred_z_axis,
  Eigen::Vector3d * z_axis)
{
  Eigen::Vector3d local_centroid;
  Eigen::Matrix3d local_axes;
  if (!compute_centroid(normal_points, &local_centroid) ||
    !compute_pca_axes(normal_points, local_centroid, &local_axes) ||
    z_axis == nullptr)
  {
    return false;
  }

  Eigen::Vector3d normal = local_axes.col(2);
  Eigen::Vector3d preferred = preferred_z_axis;
  if (preferred.norm() < 1e-8) {
    preferred = Eigen::Vector3d::UnitZ();
  }
  preferred.normalize();
  if (normal.dot(preferred) < 0.0) {
    normal = -normal;
  }
  *z_axis = normal.normalized();
  return true;
}

bool build_tangent_axes(
  const Eigen::Vector3d & z_axis,
  double axis_projection_min_norm,
  Eigen::Vector3d * x_axis,
  Eigen::Vector3d * y_axis)
{
  if (x_axis == nullptr || y_axis == nullptr || z_axis.norm() < 1e-8) {
    return false;
  }

  const Eigen::Vector3d base_x = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d base_y = Eigen::Vector3d::UnitY();
  Eigen::Vector3d x_proj = base_x - base_x.dot(z_axis) * z_axis;
  Eigen::Vector3d reference = base_x;

  if (x_proj.norm() < axis_projection_min_norm) {
    x_proj = base_y - base_y.dot(z_axis) * z_axis;
    reference = base_y;
  }

  if (x_proj.norm() < 1e-8) {
    return false;
  }

  Eigen::Vector3d x = x_proj.normalized();
  if (x.dot(reference) < 0.0) {
    x = -x;
  }
  Eigen::Vector3d y = z_axis.cross(x);
  if (y.norm() < 1e-8) {
    return false;
  }
  y.normalize();
  x = y.cross(z_axis).normalized();

  *x_axis = x;
  *y_axis = y;
  return true;
}

bool check_frame_quality(WorkpieceFrameResult * result)
{
  if (result == nullptr) {
    return false;
  }

  result->det_R = result->rotation.determinant();
  result->dot_xy = result->x_axis.dot(result->y_axis);
  result->dot_xz = result->x_axis.dot(result->z_axis);
  result->dot_yz = result->y_axis.dot(result->z_axis);

  return std::abs(result->x_axis.norm() - 1.0) < 1e-3 &&
         std::abs(result->y_axis.norm() - 1.0) < 1e-3 &&
         std::abs(result->z_axis.norm() - 1.0) < 1e-3 &&
         std::abs(result->dot_xy) < 1e-3 &&
         std::abs(result->dot_xz) < 1e-3 &&
         std::abs(result->dot_yz) < 1e-3 &&
         std::abs(result->det_R - 1.0) < 1e-3;
}

}  // namespace

bool fit_workpiece_frame(
  const WorkpiecePointCloudXYZ::ConstPtr & cloud_in,
  const WorkpieceFrameParams & params,
  const Eigen::Vector3d & preferred_z_axis,
  WorkpieceFrameResult * result)
{
  if (result == nullptr) {
    return false;
  }

  WorkpiecePointCloudXYZ::Ptr cloud = remove_nan_points(cloud_in);
  if (static_cast<int>(cloud->size()) < params.min_roi_points) {
    return false;
  }

  Eigen::Vector3d centroid;
  Eigen::Matrix3d global_axes;
  if (!compute_centroid(cloud, &centroid) || !compute_pca_axes(cloud, centroid, &global_axes)) {
    return false;
  }

  Eigen::Vector3d origin;
  if (!choose_origin_on_surface(
      cloud,
      centroid,
      global_axes,
      params.center_percentile_low,
      params.center_percentile_high,
      &origin))
  {
    return false;
  }

  WorkpiecePointCloudXYZ::Ptr normal_points;
  if (!collect_normal_fit_points(cloud, origin, params, &normal_points)) {
    return false;
  }

  Eigen::Vector3d z_axis;
  if (!fit_local_normal(normal_points, preferred_z_axis, &z_axis)) {
    return false;
  }

  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  if (!build_tangent_axes(z_axis, params.axis_projection_min_norm, &x_axis, &y_axis)) {
    return false;
  }

  result->origin = origin;
  result->x_axis = x_axis;
  result->y_axis = y_axis;
  result->z_axis = z_axis;
  result->rotation.col(0) = x_axis;
  result->rotation.col(1) = y_axis;
  result->rotation.col(2) = z_axis;
  result->roi_point_count = cloud->size();
  result->normal_fit_point_count = normal_points->size();

  if (!check_frame_quality(result)) {
    return false;
  }

  result->T_base_from_workpiece = Eigen::Matrix4d::Identity();
  result->T_base_from_workpiece.block<3, 3>(0, 0) = result->rotation;
  result->T_base_from_workpiece.block<3, 1>(0, 3) = origin;

  result->T_workpiece_from_base = Eigen::Matrix4d::Identity();
  result->T_workpiece_from_base.block<3, 3>(0, 0) = result->rotation.transpose();
  result->T_workpiece_from_base.block<3, 1>(0, 3) = -result->rotation.transpose() * origin;

  return true;
}

WorkpiecePointCloudXYZ::Ptr transform_cloud_to_workpiece(
  const WorkpiecePointCloudXYZ::ConstPtr & cloud_in,
  const Eigen::Matrix4d & T_workpiece_from_base)
{
  WorkpiecePointCloudXYZ::Ptr transformed(new WorkpiecePointCloudXYZ);
  if (!cloud_in) {
    return transformed;
  }

  transformed->reserve(cloud_in->size());
  for (const auto & point : cloud_in->points) {
    if (!is_finite_point(point)) {
      continue;
    }
    const Eigen::Vector4d point_base(point.x, point.y, point.z, 1.0);
    const Eigen::Vector4d point_workpiece = T_workpiece_from_base * point_base;
    pcl::PointXYZ out;
    out.x = static_cast<float>(point_workpiece(0));
    out.y = static_cast<float>(point_workpiece(1));
    out.z = static_cast<float>(point_workpiece(2));
    transformed->push_back(out);
  }
  return transformed;
}

}  // namespace pointcloudslam_cpp
