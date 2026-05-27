cmake_minimum_required(VERSION 3.8)
project(ui_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sigema7_msg REQUIRED)
find_package(ui_control_msg REQUIRED)



# 添加可执行文件
add_executable(ui_node src/ui_node.cpp)
ament_target_dependencies(ui_node 
  rclcpp
  rclcpp_components 
  ament_cmake 
  Qt5Core 
  Qt5Gui 
  Qt5Widgets
  image_transport
  cv_bridge
  OpenCV
  sensor_msgs
  sigema7_msg
  ui_control_msg
)
# 链接Qt5库
target_link_libraries(ui_node
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 安装可执行文件
install(TARGETS
  ui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
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
