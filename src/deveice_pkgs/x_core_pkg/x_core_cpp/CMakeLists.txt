cmake_minimum_required(VERSION 3.8)
project(x_core_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(x_core_sdk REQUIRED)

include_directories(include/x_core_cpp)
include_directories(${x_core_sdk_INCLUDE_DIRS})

add_executable(read_robot_state src/read_robot_state.cpp)
add_executable(get_keypad_state src/get_keypad_state.cpp)
add_executable(movel_example src/movel_example.cpp)

ament_target_dependencies(read_robot_state x_core_sdk)
ament_target_dependencies(get_keypad_state x_core_sdk)
ament_target_dependencies(movel_example x_core_sdk)

target_link_libraries(read_robot_state ${x_core_sdk_LIBRARIES})
target_link_libraries(get_keypad_state ${x_core_sdk_LIBRARIES})
target_link_libraries(movel_example ${x_core_sdk_LIBRARIES})

install(TARGETS read_robot_state DESTINATION lib/${PROJECT_NAME})
install(TARGETS get_keypad_state DESTINATION lib/${PROJECT_NAME})
install(TARGETS movel_example DESTINATION lib/${PROJECT_NAME})


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
