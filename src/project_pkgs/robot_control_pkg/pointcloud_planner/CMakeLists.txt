cmake_minimum_required(VERSION 3.8)
project(pointcloud_planner)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 使用 C++17（filesystem）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters segmentation search kdtree features)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(X11 REQUIRED)
# 查找 ur5_msg 包（关键步骤）
find_package(ur5_msg REQUIRED)
# find_package(puncture_model REQUIRED)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(pcl_conversions REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED
# # 找到 librealsense2 包
# find_package(realsense2 REQUIRED)



# executable
add_executable(pcd_write src/pcd_write.cpp)
add_executable(pcl_cluster src/pcl_cluster.cpp)
add_executable(pcl_select src/pcl_select.cpp)
add_executable(curve_polyfit src/curve_polyfit.cpp)
add_executable(pcl_cloudslam
  src/pcl_cloudslam.cpp
  src/concave_roi_refinement.cpp
  src/concave_workpiece_frame.cpp
  src/concave_path_planning.cpp
)
target_compile_definitions(pcl_cloudslam PRIVATE
  POINTCLOUD_PLANNER_SOURCE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)
# dependencies
ament_target_dependencies(pcd_write rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(pcl_cluster pcl_conversions)
ament_target_dependencies(pcl_select pcl_conversions)
ament_target_dependencies(curve_polyfit pcl_conversions)
ament_target_dependencies(pcl_cloudslam rclcpp sensor_msgs pcl_conversions ur5_msg nlohmann_json ament_index_cpp)
# include and link PCL
target_include_directories(pcd_write PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(pcl_cluster PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(pcl_select PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(curve_polyfit PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(pcl_cloudslam PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_write ${PCL_LIBRARIES})
target_link_libraries(pcl_cluster ${PCL_LIBRARIES})
target_link_libraries(pcl_select ${PCL_LIBRARIES})
target_link_libraries(curve_polyfit ${PCL_LIBRARIES})
target_link_libraries(pcl_cloudslam ${PCL_LIBRARIES} X11::X11)
# target_link_libraries(read_robot_state ${x_core_sdk_LIBRARIES})
# # 在 target_link_libraries 中添加 realsense2
# target_link_libraries(pcl_cloudslam
#     ${PCL_LIBRARIES}
#     realsense2::realsense2
# )

install(TARGETS pcd_write DESTINATION lib/${PROJECT_NAME})
install(TARGETS pcl_cluster DESTINATION lib/${PROJECT_NAME})
install(TARGETS pcl_select DESTINATION lib/${PROJECT_NAME})
install(TARGETS curve_polyfit DESTINATION lib/${PROJECT_NAME})
install(TARGETS pcl_cloudslam DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/move_test.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 设置包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 或使用target_include_directories
target_include_directories(pcl_cloudslam PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_executable(test_roi_mode_utils test/test_roi_mode_utils.cpp)
  target_include_directories(test_roi_mode_utils PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  )
  add_test(NAME test_roi_mode_utils COMMAND test_roi_mode_utils)
  add_executable(test_concave_workpiece_frame
    test/test_concave_workpiece_frame.cpp
    src/concave_workpiece_frame.cpp
  )
  target_include_directories(test_concave_workpiece_frame PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_concave_workpiece_frame ${PCL_LIBRARIES})
  add_test(NAME test_concave_workpiece_frame COMMAND test_concave_workpiece_frame)
  add_executable(test_concave_path_planning
    test/test_concave_path_planning.cpp
    src/concave_path_planning.cpp
  )
  target_include_directories(test_concave_path_planning PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_concave_path_planning ${PCL_LIBRARIES})
  add_test(NAME test_concave_path_planning COMMAND test_concave_path_planning)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
