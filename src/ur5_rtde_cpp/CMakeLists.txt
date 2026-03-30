cmake_minimum_required(VERSION 3.8)
project(ur5_rtde_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(ur_rtde REQUIRED) # 如有C++库请取消注释

include_directories(
  include
  # ${ur_rtde_INCLUDE_DIRS}
)

add_executable(servol_pose_controller_node src/servol_pose_controller_node.cpp)
ament_target_dependencies(servol_pose_controller_node rclcpp geometry_msgs std_msgs)
# target_link_libraries(servol_pose_controller_node ${ur_rtde_LIBRARIES})

add_executable(velocity_publisher_node src/velocity_publisher_node.cpp)
ament_target_dependencies(velocity_publisher_node rclcpp geometry_msgs std_msgs)
# target_link_libraries(velocity_publisher_node ${ur_rtde_LIBRARIES})

add_executable(test_rtde_performance src/test_rtde_performance.cpp)
ament_target_dependencies(test_rtde_performance rclcpp)
# target_link_libraries(test_rtde_performance ${ur_rtde_LIBRARIES})

install(TARGETS
  servol_pose_controller_node
  velocity_publisher_node
  test_rtde_performance
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
