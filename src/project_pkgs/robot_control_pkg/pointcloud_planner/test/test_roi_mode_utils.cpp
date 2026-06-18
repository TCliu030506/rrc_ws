#include <iostream>
#include <string>

#include "pointcloud_planner/roi_mode_utils.h"

int main()
{
  const auto expect_roi_file = [](const std::string & mode, const std::string & expected) {
    const std::string actual = pointcloudslam_cpp::roi_input_file_for_mode(mode);
    if (actual != expected) {
      std::cerr << "mode=" << mode << " expected=" << expected << " actual=" << actual << "\n";
      return false;
    }
    return true;
  };

  if (!expect_roi_file("plane", "pcl_roi.pcd") ||
      !expect_roi_file("cylinder_preplan", "pcl_roi.pcd") ||
      !expect_roi_file("concave_surface", "pcl_roi_refined.pcd") ||
      !expect_roi_file("premodel_concave_surface", "pcl_roi_refined.pcd") ||
      !expect_roi_file("unknown", "pcl_roi.pcd")) {
    return 1;
  }

  return 0;
}
