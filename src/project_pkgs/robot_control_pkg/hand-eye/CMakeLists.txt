cmake_minimum_required(VERSION 3.8)
project(hand-eye)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(coordinate REQUIRED)
find_package(ur5_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PCL REQUIRED COMPONENTS common io)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include/hand-eye ${catkin_INCLUDE_DIRS})

install(DIRECTORY 
  include/hand-eye
  DESTINATION include/${PROJECT_NAME}/)

add_executable(handeye_calibration src/hand_eye.cpp src/hand_eye_srv.cpp)
target_link_libraries(handeye_calibration ${OpenCV_LIBS})
ament_target_dependencies(handeye_calibration rclcpp geometry_msgs coordinate ur5_msg)
install(TARGETS handeye_calibration DESTINATION lib/${PROJECT_NAME})

add_executable(test_hand src/test_hand.cpp)
ament_target_dependencies(test_hand rclcpp sensor_msgs pcl_conversions)
target_link_libraries(test_hand ${PCL_LIBRARIES})
install(TARGETS test_hand DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS
  script/aruco_marker_pose_relay.py
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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
