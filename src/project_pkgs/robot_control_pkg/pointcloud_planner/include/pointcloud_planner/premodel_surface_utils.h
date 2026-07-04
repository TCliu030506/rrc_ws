#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloudslam_cpp {

  using FunctionPremodelPointCloud = pcl::PointCloud < pcl::PointXYZ >;

  inline bool function_premodel_surface_z_mm(double radius_mm, double * z_mm)
  {
    if (z_mm == nullptr || radius_mm < 0.0) {
      return false;
    }

    const auto safe_sqrt = [] (double value) {
      return std::sqrt(std::max(0.0, value));
    };

    if (radius_mm <= 17.74) {
      *z_mm = 127.5 - safe_sqrt(127.5 * 127.5 - radius_mm * radius_mm);
      return true;
    }
    if (radius_mm <= 189.97) {
      *z_mm = -1.25 + std::tan(8.0 * M_PI / 180.0) * radius_mm;
      return true;
    }
    if (radius_mm <= 205.98) {
      *z_mm = -146.50 + safe_sqrt(172.5 * 172.5 - (radius_mm - 205.98) * (radius_mm - 205.98));
      return true;
    }
    if (radius_mm <= 213.90) {
      *z_mm = 26.0;
      return true;
    }
    if (radius_mm <= 222.90) {
      *z_mm = 17.0 + safe_sqrt(9.0 * 9.0 - (radius_mm - 213.90) * (radius_mm - 213.90));
      return true;
    }
    return false;
  }

  inline FunctionPremodelPointCloud::Ptr build_function_premodel_surface_cloud(
    double sample_spacing_m)
  {
    FunctionPremodelPointCloud::Ptr cloud_model(new FunctionPremodelPointCloud);
    if (sample_spacing_m <= 0.0) {
      return cloud_model;
    }

    constexpr double mm_to_m = 0.001;
    constexpr double radius_max_mm = 222.90;
    const double radius_max_m = radius_max_mm * mm_to_m;
    const double radial_step_m = std::max(0.5 * sample_spacing_m, 0.001);
    for (double rho_m = radial_step_m; rho_m <= radius_max_m + 1e-9; rho_m += radial_step_m) {
      const double rho_mm = rho_m / mm_to_m;
      double z_mm = 0.0;
      if (!function_premodel_surface_z_mm(rho_mm, &z_mm)) {
        continue;
      }

      const int phi_count = std::max(
        16,
        static_cast < int > (std::ceil(2.0 * M_PI * rho_m / sample_spacing_m))
      );
      for (int i = 0; i < phi_count; ++i) {
        const double phi = 2.0 * M_PI * static_cast < double > (i) /
          static_cast < double > (phi_count);
        pcl::PointXYZ point;
        point.x = static_cast < float > (rho_m * std::cos(phi));
        point.y = static_cast < float > (rho_m * std::sin(phi));
        point.z = static_cast < float > (z_mm * mm_to_m);
        cloud_model->points.push_back(point);
      }
    }

    cloud_model->width = static_cast < std::uint32_t > (cloud_model->points.size());
    cloud_model->height = 1;
    cloud_model->is_dense = false;
    return cloud_model;
  }

}  // namespace pointcloudslam_cpp
