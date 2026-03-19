cmake_minimum_required(VERSION 3.8)
project(aimooe_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AimCoord.msg"
  DEPENDENCIES std_msgs geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/aimooe_sdk/AimPosition.so
  DESTINATION local/lib/python3.10/dist-packages/aimooe_sdk
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/aimooe_sdk/libopencv_core.so.407
  DESTINATION lib
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
