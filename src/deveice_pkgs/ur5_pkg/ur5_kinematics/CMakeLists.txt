cmake_minimum_required(VERSION 3.8)
project(ur5_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur5_msg REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# set(msg_files
#    msg/JointTarget.msg
#  )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointTarget.msg"
  "msg/RobotState.msg"
  DEPENDENCIES std_msgs ur5_msg
)

ament_export_dependencies(rosidl_default_runtime)


add_executable(demo src/demo.cpp include/ur5_kinematics.cpp)
ament_target_dependencies(demo Eigen3)
add_executable(demo_2 src/demo_2.cpp include/ur5_kinematics.cpp)
ament_target_dependencies(demo_2 Eigen3)
add_executable(demo_3 src/demo_3.cpp include/ur5_kinematics.cpp)
ament_target_dependencies(demo_3 Eigen3)
add_executable(demo_4 src/demo_4.cpp include/ur5_kinematics.cpp)
ament_target_dependencies(demo_4 Eigen3)
add_executable(ur5_linearmotion src/ur5_linearmotion.cpp include/ur5_kinematics.cpp)
ament_target_dependencies(ur5_linearmotion
   rclcpp std_msgs ur5_msg)

install(TARGETS
demo
demo_2
demo_3
demo_4
ur5_linearmotion
DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
   ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ur5_linearmotion "${cpp_typesupport_target}")


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
