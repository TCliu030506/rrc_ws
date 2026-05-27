cmake_minimum_required(VERSION 3.8)
project(mtactuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mtactuator_msg REQUIRED)
find_package(serial REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(mtactuator_server_node src/mtactuator_server.cpp src/mtactuator.cpp)
# target_link_libraries(mtactuator_server_node serial_lib)
ament_target_dependencies(mtactuator_server_node 
  rclcpp std_msgs mtactuator_msg serial)
install(TARGETS mtactuator_server_node DESTINATION lib/${PROJECT_NAME})

add_executable(mtactuator_server_node_8 src/mtactuator_server.cpp src/mtactuator_8.cpp)
# target_link_libraries(mtactuator_server_node_8 serial_lib)
ament_target_dependencies(mtactuator_server_node_8 
  rclcpp std_msgs mtactuator_msg serial)
install(TARGETS mtactuator_server_node_8 DESTINATION lib/${PROJECT_NAME})

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
