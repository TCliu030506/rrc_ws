#include <cassert>
#include <string>

#include "pointcloudslam_cpp/roi_mode_utils.h"

int main()
{
  assert(pointcloudslam_cpp::roi_input_file_for_mode("plane") == "pcl_roi.pcd");
  assert(pointcloudslam_cpp::roi_input_file_for_mode("cylinder_preplan") == "pcl_roi.pcd");
  assert(pointcloudslam_cpp::roi_input_file_for_mode("concave_surface") == "pcl_roi_refined.pcd");
  assert(pointcloudslam_cpp::roi_input_file_for_mode("unknown") == "pcl_roi.pcd");

  return 0;
}
