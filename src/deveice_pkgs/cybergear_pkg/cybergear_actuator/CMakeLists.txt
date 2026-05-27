cmake_minimum_required(VERSION 3.8)
project(cybergear_actuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(coordinate REQUIRED)
find_package(cybergear_msg REQUIRED)
find_package(serial REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

# 编译CPP文件:第一个CPP文件为编译的CPP文件,后接依赖的CPP文件
add_executable(cybergear_server_node 
  src/cybergear_server.cpp src/cybergear.cpp)  # 主程序，功能实现文件
# ROS1(已弃用): target_link_libraries(insactuator_server_node serial_lib)
ament_target_dependencies(cybergear_server_node 
  rclcpp std_msgs coordinate cybergear_msg serial)
# 在默认目录生成节点可执行文件
install(TARGETS cybergear_server_node 
  DESTINATION lib/${PROJECT_NAME})

# 确保找到launch文件目录下的launch.py启动文件
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/)

# 在安装目录建立launch文件的符号链接,实际上使用的是源代码的launch文件
file(GLOB LAUNCH_FILES "${CMAKE_CURRENT_SOURCE_DIR}/launch/*.launch.py")
foreach(LAUNCH_FILE ${LAUNCH_FILES})
  get_filename_component(LAUNCH_FILENAME ${LAUNCH_FILE} NAME)
  set(SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/launch//${LAUNCH_FILENAME}")
  set(TARGET_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/${LAUNCH_FILENAME}")
  #install(FILES ${file} DESTINATION share/${PROJECT_NAME} RENAME ${file_name})
  #add_custom_command(TARGET ${PROJECT_NAME}POST_BUILD COMMANDit)
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch")
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${SOURCE_PATH} ${TARGET_PATH})
endforeach()

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
