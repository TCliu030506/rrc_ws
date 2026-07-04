#pragma once

#include <string>

namespace pointcloudslam_cpp {

  inline bool is_concave_surface_mode(const std::string & planning_mode)
  {
    return planning_mode == "concave_surface" ||
           planning_mode == "premodel_concave_surface" ||
           planning_mode == "function_premodel_surface";
  }

  inline bool is_premodel_surface_mode(const std::string & planning_mode)
  {
    return planning_mode == "premodel_concave_surface" ||
           planning_mode == "function_premodel_surface";
  }

  inline std::string roi_input_file_for_mode(const std::string & planning_mode)
  {
    if (is_concave_surface_mode(planning_mode)) {
      return "pcl_roi_refined.pcd";
    }
    return "pcl_roi.pcd";
  }

}  // namespace pointcloudslam_cpp
