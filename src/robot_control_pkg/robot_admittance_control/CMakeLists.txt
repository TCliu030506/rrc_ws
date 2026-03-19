cmake_minimum_required(VERSION 3.8)
project(robot_admittance_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(force_sensor_msg REQUIRED)
find_package(ur5_control_cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(admittance_controller_node
  src/admittance_controller_node.cpp
  src/AdmittanceController.cpp
)

set(INSTALL_TARGETS admittance_controller_node)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/readForce.cpp")
  add_executable(read_force_node
    src/readForce.cpp
  )
  list(APPEND INSTALL_TARGETS read_force_node)
endif()

add_executable(control_wrench_publisher_node
  src/control_wrench_publisher_node.cpp
)

add_executable(twist_to_urscript_speedl_node
  src/twist_to_urscript_speedl_node.cpp
)

ament_target_dependencies(admittance_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  Eigen3
)


if(TARGET read_force_node)
  ament_target_dependencies(read_force_node
    rclcpp
    geometry_msgs
    std_srvs
  )
endif()

ament_target_dependencies(control_wrench_publisher_node
  rclcpp
  geometry_msgs
)

ament_target_dependencies(twist_to_urscript_speedl_node
  rclcpp
  geometry_msgs
  ur5_control_cpp
)

# 新增三轴力转发节点
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/three_axis_force_relay.cpp")
  add_executable(three_axis_force_relay
    src/three_axis_force_relay.cpp
  )
  ament_target_dependencies(three_axis_force_relay
    rclcpp
    geometry_msgs
    force_sensor_msg
  )
  target_compile_features(three_axis_force_relay PUBLIC cxx_std_17)
  list(APPEND INSTALL_TARGETS three_axis_force_relay)
endif()

target_compile_features(admittance_controller_node PUBLIC cxx_std_17)

target_compile_features(twist_to_urscript_speedl_node PUBLIC cxx_std_17)

install(TARGETS
  ${INSTALL_TARGETS}
  control_wrench_publisher_node
  twist_to_urscript_speedl_node
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Scripts/readForceSensor.py")
  install(PROGRAMS
    Scripts/readForceSensor.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(PROGRAMS
  scripts/flange_to_sensor_static_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME flange_to_sensor_static_tf
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ur_description
  DESTINATION share/${PROJECT_NAME}
)

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
