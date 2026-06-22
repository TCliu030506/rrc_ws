#include "pointcloud_planner/concave_path_planning.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

namespace pointcloudslam_cpp
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

struct SphericalPoint
{
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  double r = 0.0;
  double theta = 0.0;
  double phi = 0.0;
};

bool is_finite_point(const pcl::PointXYZ & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double normalize_phi(double phi)
{
  while (phi < 0.0) {
    phi += kTwoPi;
  }
  while (phi >= kTwoPi) {
    phi -= kTwoPi;
  }
  return phi;
}

double wrap_to_pi(double angle)
{
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

double deg_to_rad(double degrees)
{
  return degrees * kPi / 180.0;
}

double percentile(std::vector<double> values, double percent)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  percent = std::clamp(percent, 0.0, 100.0);
  const double index = percent / 100.0 * static_cast<double>(values.size() - 1);
  const size_t low = static_cast<size_t>(std::floor(index));
  const size_t high = static_cast<size_t>(std::ceil(index));
  if (low == high) {
    return values[low];
  }
  const double t = index - static_cast<double>(low);
  return values[low] * (1.0 - t) + values[high] * t;
}

int normalized_odd_window(int requested_window, size_t point_count)
{
  if (point_count < 3) {
    return 0;
  }
  int window = std::max(3, requested_window);
  if (window % 2 == 0) {
    ++window;
  }
  const int max_window = point_count % 2 == 0 ?
    static_cast<int>(point_count) - 1 : static_cast<int>(point_count);
  return std::min(window, max_window);
}

std::vector<double> savitzky_golay_coefficients(int window, int order)
{
  const int half_window = window / 2;
  order = std::clamp(order, 0, window - 1);
  Eigen::MatrixXd design(window, order + 1);
  for (int row = 0; row < window; ++row) {
    const double x = static_cast<double>(row - half_window);
    double value = 1.0;
    for (int column = 0; column <= order; ++column) {
      design(row, column) = value;
      value *= x;
    }
  }

  Eigen::VectorXd intercept = Eigen::VectorXd::Zero(order + 1);
  intercept(0) = 1.0;
  const Eigen::VectorXd coefficients =
    design * (design.transpose() * design).ldlt().solve(intercept);
  return std::vector<double>(coefficients.data(), coefficients.data() + window);
}

void smooth_periodic_theta(
  const std::vector<size_t> & indices,
  const ConcavePathParams & params,
  std::vector<ConcaveWorkpiecePathPoint> * points)
{
  const int window = normalized_odd_window(
    params.position_smoothing_window, indices.size());
  if (window == 0 || params.position_smoothing_passes <= 0) {
    return;
  }

  const std::vector<double> coefficients = savitzky_golay_coefficients(
    window, params.position_smoothing_order);
  const int half_window = window / 2;
  std::vector<double> values;
  values.reserve(indices.size());
  for (const size_t index : indices) {
    values.push_back((*points)[index].theta);
  }

  for (int pass = 0; pass < params.position_smoothing_passes; ++pass) {
    std::vector<double> smoothed(values.size(), 0.0);
    for (size_t i = 0; i < values.size(); ++i) {
      for (int offset = -half_window; offset <= half_window; ++offset) {
        const int source_index = (
          static_cast<int>(i) + offset + static_cast<int>(values.size())) %
          static_cast<int>(values.size());
        const size_t source = static_cast<size_t>(source_index);
        smoothed[i] += coefficients[offset + half_window] * values[source];
      }
    }
    values.swap(smoothed);
  }

  const double max_deviation = std::max(
    0.0, params.position_smoothing_max_deviation);
  for (size_t i = 0; i < indices.size(); ++i) {
    ConcaveWorkpiecePathPoint & point = (*points)[indices[i]];
    const Eigen::Vector3d original = point.surface_point;
    Eigen::Vector3d candidate(
      point.r * std::cos(values[i]) * std::cos(point.phi),
      point.r * std::cos(values[i]) * std::sin(point.phi),
      point.r * std::sin(values[i]));
    Eigen::Vector3d correction = candidate - original;
    if (max_deviation > 0.0 && correction.norm() > max_deviation) {
      correction *= max_deviation / correction.norm();
      candidate = original + correction;
    }
    point.surface_point = candidate;
    point.r = candidate.norm();
    point.theta = std::atan2(
      candidate(2), std::hypot(candidate(0), candidate(1)));
  }
}

void smooth_periodic_normals(
  const std::vector<size_t> & indices,
  const ConcavePathParams & params,
  std::vector<ConcaveWorkpiecePathPoint> * points)
{
  const int window = normalized_odd_window(
    params.normal_smoothing_window, indices.size());
  if (window == 0 || params.normal_smoothing_passes <= 0) {
    return;
  }

  const int half_window = window / 2;
  std::vector<Eigen::Vector3d> normals;
  normals.reserve(indices.size());
  for (const size_t index : indices) {
    normals.push_back((*points)[index].normal.normalized());
  }

  for (int pass = 0; pass < params.normal_smoothing_passes; ++pass) {
    std::vector<Eigen::Vector3d> smoothed(normals.size(), Eigen::Vector3d::Zero());
    for (size_t i = 0; i < normals.size(); ++i) {
      for (int offset = -half_window; offset <= half_window; ++offset) {
        const int wrapped = (
          static_cast<int>(i) + offset + static_cast<int>(normals.size())) %
          static_cast<int>(normals.size());
        Eigen::Vector3d normal = normals[static_cast<size_t>(wrapped)];
        if (normal.dot(normals[i]) < 0.0) {
          normal = -normal;
        }
        const double weight = static_cast<double>(half_window + 1 - std::abs(offset));
        smoothed[i] += weight * normal;
      }
      if (smoothed[i].norm() < 1e-8) {
        smoothed[i] = normals[i];
      } else {
        smoothed[i].normalize();
      }
    }
    normals.swap(smoothed);
  }

  for (size_t i = 0; i < indices.size(); ++i) {
    (*points)[indices[i]].normal = normals[i];
  }
}

void smooth_workpiece_path(
  const ConcavePathParams & params,
  std::vector<ConcaveWorkpiecePathPoint> * points)
{
  if (!params.enable_path_smoothing || points == nullptr || points->empty()) {
    return;
  }

  int max_layer_id = -1;
  for (const auto & point : *points) {
    max_layer_id = std::max(max_layer_id, point.layer_id);
  }
  std::vector<std::vector<size_t>> layer_indices(
    static_cast<size_t>(max_layer_id + 1));
  for (size_t i = 0; i < points->size(); ++i) {
    const auto & point = (*points)[i];
    if (!point.is_transition && point.layer_id >= 0) {
      layer_indices[static_cast<size_t>(point.layer_id)].push_back(i);
    }
  }

  for (const auto & indices : layer_indices) {
    smooth_periodic_theta(indices, params, points);
    smooth_periodic_normals(indices, params, points);
  }
}

Eigen::Vector3d project_to_tangent(
  const Eigen::Vector3d & vector_in,
  const Eigen::Vector3d & normal)
{
  return vector_in - vector_in.dot(normal) * normal;
}

Eigen::Vector3d rotation_to_rotvec_continuous(
  const Eigen::Matrix3d & rotation,
  bool has_previous_quaternion,
  const Eigen::Quaterniond & previous_quaternion,
  Eigen::Quaterniond * current_quaternion_out)
{
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  if (has_previous_quaternion && quaternion.dot(previous_quaternion) < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  if (current_quaternion_out != nullptr) {
    *current_quaternion_out = quaternion;
  }

  Eigen::AngleAxisd angle_axis(quaternion);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1e-8) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

bool build_spherical_points(
  const ConcavePathPointCloud::ConstPtr & cloud,
  std::vector<SphericalPoint> * spherical_points,
  std::vector<double> * radii)
{
  if (!cloud || spherical_points == nullptr || radii == nullptr) {
    return false;
  }

  spherical_points->clear();
  radii->clear();
  spherical_points->reserve(cloud->size());
  radii->reserve(cloud->size());

  for (const auto & point : cloud->points) {
    if (!is_finite_point(point)) {
      continue;
    }
    SphericalPoint spherical;
    spherical.p = Eigen::Vector3d(point.x, point.y, point.z);
    const double xy = std::hypot(spherical.p(0), spherical.p(1));
    spherical.r = spherical.p.norm();
    if (spherical.r < 1e-9) {
      continue;
    }
    spherical.theta = std::atan2(spherical.p(2), xy);
    spherical.phi = normalize_phi(std::atan2(spherical.p(1), spherical.p(0)));
    spherical_points->push_back(spherical);
    radii->push_back(spherical.r);
  }
  return !spherical_points->empty();
}

bool estimate_theta(
  const std::vector<SphericalPoint> & points,
  double target_r,
  double target_phi,
  const ConcavePathParams & params,
  double * theta_out)
{
  const double phi_fit = deg_to_rad(params.phi_fit_deg);
  const double sigma_r = std::max(1e-6, params.r_fit * 0.5);
  const double sigma_phi = std::max(1e-6, phi_fit * 0.5);
  double weighted_sum = 0.0;
  double weight_sum = 0.0;
  double plain_sum = 0.0;
  int count = 0;

  for (const auto & point : points) {
    const double dr = point.r - target_r;
    if (std::abs(dr) > params.r_fit) {
      continue;
    }
    const double dphi = wrap_to_pi(point.phi - target_phi);
    if (std::abs(dphi) > phi_fit) {
      continue;
    }

    const double weight = std::exp(
      -(dr * dr) / (2.0 * sigma_r * sigma_r) -
      (dphi * dphi) / (2.0 * sigma_phi * sigma_phi));
    weighted_sum += weight * point.theta;
    weight_sum += weight;
    plain_sum += point.theta;
    ++count;
  }

  if (count < params.min_fit_points || theta_out == nullptr) {
    return false;
  }
  if (weight_sum > 1e-9) {
    *theta_out = weighted_sum / weight_sum;
  } else {
    *theta_out = plain_sum / static_cast<double>(count);
  }
  return true;
}

bool fit_normal(
  const ConcavePathPointCloud::ConstPtr & cloud,
  pcl::KdTreeFLANN<pcl::PointXYZ> & tree,
  const Eigen::Vector3d & surface_point,
  const ConcavePathParams & params,
  Eigen::Vector3d * normal_out)
{
  pcl::PointXYZ search_point;
  search_point.x = static_cast<float>(surface_point(0));
  search_point.y = static_cast<float>(surface_point(1));
  search_point.z = static_cast<float>(surface_point(2));

  std::vector<int> indices;
  std::vector<float> distances;
  tree.radiusSearch(search_point, params.normal_fit_radius, indices, distances);
  if (static_cast<int>(indices.size()) < params.min_normal_fit_points || normal_out == nullptr) {
    return false;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const int index : indices) {
    const auto & point = cloud->points[index];
    centroid += Eigen::Vector3d(point.x, point.y, point.z);
  }
  centroid /= static_cast<double>(indices.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const int index : indices) {
    const auto & point = cloud->points[index];
    const Eigen::Vector3d delta(point.x - centroid(0), point.y - centroid(1),
      point.z - centroid(2));
    covariance += delta * delta.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return false;
  }

  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  if (normal.norm() < 1e-8) {
    return false;
  }
  normal.normalize();
  if (normal(2) < 0.0) {
    normal = -normal;
  }
  *normal_out = normal;
  return true;
}

bool build_orientation(
  const Eigen::Vector3d & normal,
  double theta,
  double phi,
  bool reverse_scan,
  const ConcavePathParams & params,
  Eigen::Matrix3d * rotation_out)
{
  Eigen::Vector3d e_r(
    std::cos(theta) * std::cos(phi),
    std::cos(theta) * std::sin(phi),
    std::sin(theta));
  Eigen::Vector3d e_phi(-std::sin(phi), std::cos(phi), 0.0);

  Eigen::Vector3d x_axis = project_to_tangent(e_phi, normal);
  if (x_axis.norm() < 1e-8) {
    return false;
  }
  x_axis.normalize();
  if (reverse_scan && params.align_x_axis_to_scan_direction) {
    x_axis = -x_axis;
  }

  Eigen::Vector3d feed_axis = project_to_tangent(e_r, normal);
  if (feed_axis.norm() < 1e-8) {
    return false;
  }
  feed_axis.normalize();

  Eigen::Vector3d z_axis = params.tool_z_negative_normal ? -normal : normal;
  if (z_axis.norm() < 1e-8) {
    return false;
  }
  z_axis.normalize();

  Eigen::Vector3d y_axis = z_axis.cross(x_axis);
  if (y_axis.norm() < 1e-8) {
    y_axis = z_axis.cross(feed_axis);
  }
  if (y_axis.norm() < 1e-8) {
    return false;
  }
  y_axis.normalize();
  x_axis = y_axis.cross(z_axis);
  if (x_axis.norm() < 1e-8) {
    return false;
  }
  x_axis.normalize();

  rotation_out->col(0) = x_axis;
  rotation_out->col(1) = y_axis;
  rotation_out->col(2) = z_axis;
  return std::abs(rotation_out->determinant() - 1.0) < 1e-3;
}

bool build_orientation_from_tangent(
  const Eigen::Vector3d & normal,
  Eigen::Vector3d tangent,
  bool reverse_scan,
  const ConcavePathParams & params,
  Eigen::Matrix3d * rotation_out)
{
  if (!params.align_x_axis_to_scan_direction && reverse_scan) {
    tangent = -tangent;
  }
  Eigen::Vector3d x_axis = project_to_tangent(tangent, normal);
  if (x_axis.norm() < 1e-8) {
    return false;
  }
  x_axis.normalize();

  Eigen::Vector3d z_axis = params.tool_z_negative_normal ? -normal : normal;
  if (z_axis.norm() < 1e-8) {
    return false;
  }
  z_axis.normalize();

  Eigen::Vector3d y_axis = z_axis.cross(x_axis);
  if (y_axis.norm() < 1e-8) {
    return false;
  }
  y_axis.normalize();
  x_axis = y_axis.cross(z_axis).normalized();

  rotation_out->col(0) = x_axis;
  rotation_out->col(1) = y_axis;
  rotation_out->col(2) = z_axis;
  return std::abs(rotation_out->determinant() - 1.0) < 1e-3;
}

bool append_path_point(
  const std::vector<SphericalPoint> & spherical_points,
  const ConcavePathPointCloud::ConstPtr & cloud,
  pcl::KdTreeFLANN<pcl::PointXYZ> & tree,
  const ConcavePathParams & params,
  double target_r,
  double target_phi,
  bool is_transition,
  int layer_id,
  int point_id,
  bool * has_previous_normal,
  Eigen::Vector3d * previous_normal,
  ConcavePathResult * result)
{
  result->total_candidate_points++;

  double theta = 0.0;
  if (!estimate_theta(spherical_points, target_r, target_phi, params, &theta)) {
    result->skipped_theta_fit_points++;
    return false;
  }

  Eigen::Vector3d surface_workpiece(
    target_r * std::cos(theta) * std::cos(target_phi),
    target_r * std::cos(theta) * std::sin(target_phi),
    target_r * std::sin(theta));

  Eigen::Vector3d normal_workpiece;
  if (!fit_normal(cloud, tree, surface_workpiece, params, &normal_workpiece)) {
    result->skipped_normal_fit_points++;
    return false;
  }

  if (*has_previous_normal && normal_workpiece.dot(*previous_normal) < 0.0) {
    normal_workpiece = -normal_workpiece;
  }
  if (*has_previous_normal) {
    const double alpha = std::clamp(params.normal_smoothing_alpha, 0.0, 1.0);
    normal_workpiece = alpha * normal_workpiece + (1.0 - alpha) * (*previous_normal);
    if (normal_workpiece.norm() < 1e-8) {
      result->skipped_normal_fit_points++;
      return false;
    }
    normal_workpiece.normalize();
  }

  ConcaveWorkpiecePathPoint debug_point;
  debug_point.surface_point = surface_workpiece;
  debug_point.normal = normal_workpiece;
  debug_point.r = target_r;
  debug_point.theta = theta;
  debug_point.phi = target_phi;
  debug_point.layer_id = layer_id;
  debug_point.point_id = point_id;
  debug_point.is_transition = is_transition;
  result->workpiece_points.push_back(debug_point);

  *previous_normal = normal_workpiece;
  *has_previous_normal = true;
  result->valid_path_points++;
  return true;
}

bool rebuild_orientations_and_base_poses(
  const Eigen::Matrix4d & T_base_from_workpiece,
  const ConcavePathParams & params,
  const Eigen::Vector3d & probe_offset,
  const Eigen::Matrix3d & tool_mount_rotation,
  ConcavePathResult * result)
{
  if (result == nullptr) {
    return false;
  }

  result->base_poses.clear();
  result->base_poses.reserve(result->workpiece_points.size());
  const Eigen::Matrix3d R_base_workpiece =
    T_base_from_workpiece.block<3, 3>(0, 0);
  const Eigen::Vector3d origin_base =
    T_base_from_workpiece.block<3, 1>(0, 3);
  bool has_previous_quaternion = false;
  Eigen::Quaterniond previous_quaternion = Eigen::Quaterniond::Identity();
  std::vector<size_t> previous_layer_point(
    result->workpiece_points.size(), std::numeric_limits<size_t>::max());
  std::vector<size_t> next_layer_point(
    result->workpiece_points.size(), std::numeric_limits<size_t>::max());

  int max_layer_id = -1;
  for (const auto & point : result->workpiece_points) {
    max_layer_id = std::max(max_layer_id, point.layer_id);
  }
  std::vector<std::vector<size_t>> layer_indices(
    static_cast<size_t>(max_layer_id + 1));
  for (size_t i = 0; i < result->workpiece_points.size(); ++i) {
    const auto & point = result->workpiece_points[i];
    if (!point.is_transition && point.layer_id >= 0) {
      layer_indices[static_cast<size_t>(point.layer_id)].push_back(i);
    }
  }
  for (const auto & indices : layer_indices) {
    if (indices.size() < 3) {
      continue;
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      previous_layer_point[indices[i]] =
        indices[(i + indices.size() - 1) % indices.size()];
      next_layer_point[indices[i]] = indices[(i + 1) % indices.size()];
    }
  }

  for (size_t i = 0; i < result->workpiece_points.size(); ++i) {
    auto & point = result->workpiece_points[i];
    const bool reverse_scan = (point.layer_id % 2) == 1;
    Eigen::Matrix3d rotation_workpiece;
    bool orientation_ok = false;
    if (!point.is_transition &&
      previous_layer_point[i] != std::numeric_limits<size_t>::max())
    {
      const Eigen::Vector3d tangent =
        result->workpiece_points[next_layer_point[i]].surface_point -
        result->workpiece_points[previous_layer_point[i]].surface_point;
      orientation_ok = build_orientation_from_tangent(
        point.normal, tangent, reverse_scan, params, &rotation_workpiece);
    }
    if (!orientation_ok) {
      orientation_ok = build_orientation(
        point.normal, point.theta, point.phi, reverse_scan, params,
        &rotation_workpiece);
    }
    if (!orientation_ok) {
      result->skipped_orientation_points++;
      return false;
    }
    rotation_workpiece *= tool_mount_rotation;
    point.rotation_workpiece = rotation_workpiece;

    const Eigen::Vector3d surface_base =
      R_base_workpiece * point.surface_point + origin_base;
    const Eigen::Vector3d normal_base = R_base_workpiece * point.normal;
    const Eigen::Matrix3d rotation_base =
      R_base_workpiece * rotation_workpiece;
    const Eigen::Vector3d tcp_base =
      surface_base + params.surface_clearance * normal_base -
      rotation_base * probe_offset;

    Eigen::Quaterniond current_quaternion;
    const Eigen::Vector3d rotvec = rotation_to_rotvec_continuous(
      rotation_base, has_previous_quaternion, previous_quaternion,
      &current_quaternion);
    Eigen::Matrix<double, 6, 1> pose;
    pose << tcp_base(0), tcp_base(1), tcp_base(2),
      rotvec(0), rotvec(1), rotvec(2);
    result->base_poses.push_back(pose);
    previous_quaternion = current_quaternion;
    has_previous_quaternion = true;
  }
  return true;
}

}  // namespace

bool generate_concave_path(
  const ConcavePathPointCloud::ConstPtr & cloud_workpiece,
  const Eigen::Matrix4d & T_base_from_workpiece,
  const ConcavePathParams & params,
  const Eigen::Vector3d & probe_offset,
  const Eigen::Matrix3d & tool_mount_rotation,
  ConcavePathResult * result)
{
  if (!cloud_workpiece || !result || params.probe_radial_length <= 0.0 ||
    params.radial_step <= 0.0 || params.scan_point_spacing <= 0.0)
  {
    return false;
  }

  *result = ConcavePathResult();
  std::vector<SphericalPoint> spherical_points;
  std::vector<double> radii;
  if (!build_spherical_points(cloud_workpiece, &spherical_points, &radii)) {
    return false;
  }

  result->r_min = *std::min_element(radii.begin(), radii.end());
  result->r_max = *std::max_element(radii.begin(), radii.end());
  result->r_95 = percentile(radii, 95.0);
  const double outer_radius = result->r_95 - params.boundary_margin - 0.5 *
    params.probe_radial_length;
  if (outer_radius <= params.r_end) {
    return false;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud_workpiece);

  const double delta_phi_min = deg_to_rad(params.delta_phi_min_deg);
  const double delta_phi_max = deg_to_rad(params.delta_phi_max_deg);
  bool has_previous_normal = false;
  Eigen::Vector3d previous_normal = Eigen::Vector3d::UnitZ();
  std::vector<double> layer_radii;
  for (double radius = outer_radius; radius > params.r_end; radius -= params.radial_step) {
    layer_radii.push_back(radius);
  }
  result->layer_count = layer_radii.size();

  int global_point_id = 0;
  for (size_t layer = 0; layer < layer_radii.size(); ++layer) {
    const double radius = layer_radii[layer];
    const bool reverse_scan = (layer % 2) == 1;
    const double raw_delta_phi = params.scan_point_spacing / std::max(radius, 1e-6);
    const double delta_phi = std::clamp(raw_delta_phi, delta_phi_min, delta_phi_max);
    const int sample_count = std::max(8, static_cast<int>(std::ceil(kTwoPi / delta_phi)));
    size_t valid_in_layer = 0;

    for (int sample = 0; sample < sample_count; ++sample) {
      const double t = static_cast<double>(sample) / static_cast<double>(sample_count);
      double phi = reverse_scan ? kTwoPi * (1.0 - t) : kTwoPi * t;
      phi = normalize_phi(phi);
      if (append_path_point(
          spherical_points, cloud_workpiece, tree, params,
          radius, phi, false,
          static_cast<int>(layer), global_point_id, &has_previous_normal,
          &previous_normal, result))
      {
        ++valid_in_layer;
        ++global_point_id;
      }
    }

    if (valid_in_layer < static_cast<size_t>(params.min_valid_points_per_layer)) {
      continue;
    }

    if (params.enable_transition_points && layer + 1 < layer_radii.size()) {
      const double next_radius = layer_radii[layer + 1];
      const double phi_end = reverse_scan ? 0.0 : 0.0;
      const int transition_count = std::max(
        0,
        static_cast<int>(std::floor(
          std::abs(radius - next_radius) /
          std::max(params.feed_step, 1e-6))) - 1);
      for (int step = 1; step <= transition_count; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(transition_count + 1);
        const double interpolation_ratio = params.enable_quintic_transition ?
          ratio * ratio * ratio * (10.0 - 15.0 * ratio + 6.0 * ratio * ratio) :
          ratio;
        const double transition_radius =
          radius + interpolation_ratio * (next_radius - radius);
        if (append_path_point(
            spherical_points, cloud_workpiece, tree, params,
            transition_radius, phi_end, true,
            static_cast<int>(layer), global_point_id, &has_previous_normal,
            &previous_normal, result))
        {
          ++global_point_id;
        }
      }
    }
  }

  smooth_workpiece_path(params, &result->workpiece_points);
  if (!rebuild_orientations_and_base_poses(
      T_base_from_workpiece, params, probe_offset, tool_mount_rotation, result))
  {
    return false;
  }
  return static_cast<int>(result->base_poses.size()) >= params.min_total_path_points;
}

}  // namespace pointcloudslam_cpp
