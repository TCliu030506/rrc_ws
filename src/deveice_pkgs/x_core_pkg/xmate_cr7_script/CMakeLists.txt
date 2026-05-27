cmake_minimum_required(VERSION 3.8)
project(xmate_cr7_script)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(x_core_sdk REQUIRED)
find_package(xmate_cr7_msg REQUIRED)
find_package(force_sensor_msg REQUIRED)
find_package(coordinate REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${x_core_sdk_INCLUDE_DIRS})

add_executable(cr7_server src/cr7_server.cpp)
add_executable(cr7_force_sensor src/cr7_force_sensor.cpp)
add_executable(cr7_state_seneor src/cr7_state_seneor.cpp)
add_executable(cr7_script src/cr7_script.cpp)
add_executable(cr7_script_client_demo src/cr7_script_client_demo.cpp)

ament_target_dependencies(cr7_server x_core_sdk xmate_cr7_msg rclcpp coordinate)
ament_target_dependencies(cr7_force_sensor force_sensor_msg rclcpp)
ament_target_dependencies(cr7_state_seneor xmate_cr7_msg rclcpp)
ament_target_dependencies(cr7_script coordinate rclcpp std_msgs)
ament_target_dependencies(cr7_script_client_demo rclcpp)


target_link_libraries(cr7_server ${x_core_sdk_LIBRARIES}  ) 

# 添加 cr7_script 共享库（如果你希望作为模块被其他包调用）
add_library(cr7_script_lib SHARED src/cr7_script.cpp)

target_include_directories(cr7_script_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cr7_script_lib
    coordinate
    rclcpp
    std_msgs
)

# 安装库和头文件
install(TARGETS cr7_script_lib
    EXPORT export_cr7_script_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_cr7_script_lib)
ament_export_dependencies(coordinate rclcpp std_msgs)

# 链接 cr7_script_lib 共享库
target_link_libraries(cr7_script_client_demo
    cr7_script_lib
)
target_link_libraries(cr7_force_sensor
    cr7_script_lib
)
target_link_libraries(cr7_state_seneor
    cr7_script_lib
)

install(TARGETS cr7_server DESTINATION lib/${PROJECT_NAME})
install(TARGETS cr7_force_sensor DESTINATION lib/${PROJECT_NAME})
install(TARGETS cr7_state_seneor DESTINATION lib/${PROJECT_NAME})
install(TARGETS cr7_script DESTINATION lib/${PROJECT_NAME})
install(TARGETS cr7_script_client_demo DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
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
