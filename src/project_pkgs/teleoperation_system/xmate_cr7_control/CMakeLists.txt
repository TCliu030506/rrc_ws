cmake_minimum_required(VERSION 3.8)
project(xmate_cr7_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(x_core_sdk REQUIRED)
find_package(xmate_cr7_msg REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(${x_core_sdk_INCLUDE_DIRS})

add_executable(cr7_server src/cr7_server.cpp)
add_executable(cr7_client_demo src/cr7_client_demo.cpp)

ament_target_dependencies(cr7_server x_core_sdk xmate_cr7_msg rclcpp)
ament_target_dependencies(cr7_client_demo xmate_cr7_msg rclcpp)

target_link_libraries(cr7_server ${x_core_sdk_LIBRARIES}  ) 

install(TARGETS cr7_server DESTINATION lib/${PROJECT_NAME})
install(TARGETS cr7_client_demo DESTINATION lib/${PROJECT_NAME})


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
