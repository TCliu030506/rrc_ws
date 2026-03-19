cmake_minimum_required(VERSION 3.8)
project(ur5_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# set(msg_files
#   msg/RobotState.msg
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  DEPENDENCIES std_msgs 
)

# include_directories(include)

add_executable(ur5_msg_demo src/ur5_msg_demo.cpp)
# target_link_libraries(ur5_msg_demo ${catkin_LIBRARIES})
ament_target_dependencies(ur5_msg_demo
  rclcpp std_msgs sensor_msgs)

add_executable(msgsub_demo src/msgsub_demo.cpp)
ament_target_dependencies(msgsub_demo
  rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl_target_interfaces(ur5_msg_demo
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ur5_msg_demo "${cpp_typesupport_target}")
install(TARGETS ur5_msg_demo DESTINATION lib/${PROJECT_NAME})
# target_include_directories(ur5_msg_demo PUBLIC /opt/ros/humble/include)

# rosidl_get_typesupport_target(msgsub_demo
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(msgsub_demo "${cpp_typesupport_target}")
install(TARGETS msgsub_demo DESTINATION lib/${PROJECT_NAME})

include_directories(include)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

add_executable(ur5_msg_pub src/ur5_msg.cpp src/tf_fwd.cpp)
ament_target_dependencies(ur5_msg_pub rclcpp sensor_msgs)
target_link_libraries(ur5_msg_pub "${cpp_typesupport_target}")
install(TARGETS ur5_msg_pub DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)


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
