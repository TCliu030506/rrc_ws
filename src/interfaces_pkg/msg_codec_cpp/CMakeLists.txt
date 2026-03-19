cmake_minimum_required(VERSION 3.8)
project(msg_codec_cpp)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 设置自定义消息和服务的路径
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Script.srv"
  DEPENDENCIES std_msgs
)

# 包含头文件
include_directories(
  include
)

# 生成动态库
add_library(${PROJECT_NAME}_lib SHARED
  src/msg_coder.cpp
)

# 设置库的属性和依赖
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接库的依赖
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp)

# 添加可执行文件及其ROS2依赖项
# 添加节点
add_executable(msg_publisher src/msg_publisher.cpp)
target_link_libraries(msg_publisher PUBLIC ${PROJECT_NAME}_lib)
ament_target_dependencies(msg_publisher PUBLIC rclcpp std_msgs)

# 添加节点
add_executable(msg_subscriber src/msg_subscriber.cpp)
target_link_libraries(msg_subscriber ${PROJECT_NAME}_lib)
ament_target_dependencies(msg_subscriber rclcpp std_msgs)

# 导出库的信息，以便其他功能包使用
ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)

# 安装自定义头文件
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

# 安装库文件
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装可执行文件
install(TARGETS
  msg_publisher
  msg_subscriber
  DESTINATION lib/${PROJECT_NAME}
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