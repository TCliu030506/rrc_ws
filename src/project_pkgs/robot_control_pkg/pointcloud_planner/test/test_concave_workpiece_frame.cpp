#include <cassert>
#include <cmath>

#include "pointcloudslam_cpp/concave_workpiece_frame.h"

namespace
{

bool near(double lhs, double rhs, double tolerance)
{
  return std::abs(lhs - rhs) < tolerance;
}

}  // namespace

int main()
{
  pointcloudslam_cpp::WorkpiecePointCloudXYZ::Ptr cloud(new pointcloudslam_cpp::
    WorkpiecePointCloudXYZ);
  for (int ix = -10; ix <= 10; ++ix) {
    for (int iy = -8; iy <= 8; ++iy) {
      pcl::PointXYZ point;
      point.x = 0.5f + static_cast<float>(ix) * 0.005f;
      point.y = -0.2f + static_cast<float>(iy) * 0.005f;
      point.z = 0.3f;
      cloud->push_back(point);
    }
  }

  pointcloudslam_cpp::WorkpieceFrameParams params;
  params.min_roi_points = 20;
  params.normal_fit_radius = 0.03;
  params.min_normal_fit_points = 20;
  params.normal_fit_k = 80;

  pointcloudslam_cpp::WorkpieceFrameResult result;
  const bool ok = pointcloudslam_cpp::fit_workpiece_frame(
    cloud, params,
    Eigen::Vector3d::UnitZ(), &result);
  assert(ok);
  assert(result.normal_fit_point_count >= static_cast<size_t>(params.min_normal_fit_points));
  assert(std::abs(result.rotation.determinant() - 1.0) < 1e-6);
  assert(std::abs(result.x_axis.dot(result.y_axis)) < 1e-6);
  assert(std::abs(result.x_axis.dot(result.z_axis)) < 1e-6);
  assert(std::abs(result.y_axis.dot(result.z_axis)) < 1e-6);
  assert(result.z_axis.dot(Eigen::Vector3d::UnitZ()) > 0.99);

  pointcloudslam_cpp::WorkpiecePointCloudXYZ::Ptr transformed =
    pointcloudslam_cpp::transform_cloud_to_workpiece(cloud, result.T_workpiece_from_base);
  assert(transformed->size() == cloud->size());

  Eigen::Vector4d origin_in_workpiece = result.T_workpiece_from_base *
    Eigen::Vector4d(result.origin(0), result.origin(1), result.origin(2), 1.0);
  assert(near(origin_in_workpiece(0), 0.0, 1e-9));
  assert(near(origin_in_workpiece(1), 0.0, 1e-9));
  assert(near(origin_in_workpiece(2), 0.0, 1e-9));

  return 0;
}
