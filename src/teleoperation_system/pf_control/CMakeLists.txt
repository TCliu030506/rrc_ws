cmake_minimum_required(VERSION 3.8)
project(pf_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pf_kinematics REQUIRED)  # 确保能找到 pf_kinematics
find_package(ft_servo_motor_pkg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aimooe_sdk REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# 包含目录
include_directories(
  include
  include/pf_control

)

ament_export_include_directories(include)
include_directories(${pf_kinematics_INCLUDE_DIRS})

# 添加可执行文件
add_executable(pf_control_test src/pf_control_test.cpp src/pf_control.cpp)
add_executable(pf_move_tests_8points src/pf_move_tests_8points.cpp src/pf_control.cpp)
add_executable(auto_path src/auto_path.cpp src/pf_control.cpp)
add_executable(pf_move_tests_from_file src/pf_move_tests_from_file.cpp src/pf_control.cpp)
add_executable(pf_base_measurement_aim src/pf_base_measurement_aim.cpp src/pf_control.cpp)
add_executable(exp1_positioning src/exp1_positioning.cpp src/pf_control.cpp)
add_executable(exp2_repositioning src/exp2_repositioning.cpp src/pf_control.cpp)
add_executable(exp3_following src/exp3_following.cpp src/pf_control.cpp)



# 设置依赖项
ament_target_dependencies(pf_control_test
  rclcpp
  ft_servo_motor_pkg
  pf_kinematics
)

# 设置依赖项
ament_target_dependencies(pf_move_tests_8points
  rclcpp
  ft_servo_motor_pkg
  pf_kinematics
)

ament_target_dependencies(auto_path
  rclcpp
  ft_servo_motor_pkg
  pf_kinematics
)

# 设置依赖项
ament_target_dependencies(pf_move_tests_from_file
  rclcpp
  ft_servo_motor_pkg
  pf_kinematics
  aimooe_sdk
)
ament_target_dependencies(pf_base_measurement_aim
  rclcpp
  aimooe_sdk
  ft_servo_motor_pkg
  pf_kinematics
)
ament_target_dependencies(exp1_positioning
  rclcpp
  aimooe_sdk
  ft_servo_motor_pkg
  pf_kinematics
)
ament_target_dependencies(exp2_repositioning
  rclcpp
  aimooe_sdk
  ft_servo_motor_pkg
  pf_kinematics
)
ament_target_dependencies(exp3_following
  rclcpp
  aimooe_sdk
  ft_servo_motor_pkg
  pf_kinematics
)

# 链接库
target_link_libraries(pf_control_test ${ft_servo_motor_pkg_LIBS})
target_link_libraries(pf_move_tests_8points ${ft_servo_motor_pkg_LIBS})
target_link_libraries(auto_path ${ft_servo_motor_pkg_LIBS})
target_link_libraries(pf_move_tests_from_file ${ft_servo_motor_pkg_LIBS} ${aimooe_sdk_LIBS})
target_link_libraries(pf_base_measurement_aim ${ft_servo_motor_pkg_LIBS} ${aimooe_sdk_LIBS})
target_link_libraries(exp1_positioning ${ft_servo_motor_pkg_LIBS} ${aimooe_sdk_LIBS})
target_link_libraries(exp2_repositioning ${ft_servo_motor_pkg_LIBS} ${aimooe_sdk_LIBS})
target_link_libraries(exp3_following ${ft_servo_motor_pkg_LIBS} ${aimooe_sdk_LIBS})

# 安装可执行文件
install(TARGETS pf_control_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS pf_move_tests_8points DESTINATION lib/${PROJECT_NAME})
install(TARGETS auto_path DESTINATION lib/${PROJECT_NAME})
install(TARGETS pf_move_tests_from_file DESTINATION lib/${PROJECT_NAME})
install(TARGETS pf_base_measurement_aim DESTINATION lib/${PROJECT_NAME})
install(TARGETS exp1_positioning DESTINATION lib/${PROJECT_NAME})
install(TARGETS exp2_repositioning DESTINATION lib/${PROJECT_NAME})
install(TARGETS exp3_following DESTINATION lib/${PROJECT_NAME})

# 安装资源文件
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# 将include目录下的所有头文件安装到include目录下
install(
  DIRECTORY include/ 
  DESTINATION include
)

# 生成共享库文件
add_library(pf_control SHARED src/pf_control.cpp)

# 设置共享库的包含路径
target_include_directories(pf_control
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pf_kinematics_INCLUDE_DIRS}
)
# 设置共享库的依赖项
ament_target_dependencies(pf_control
  rclcpp
  ft_servo_motor_pkg
  pf_kinematics
  aimooe_sdk
)
# 链接共享库到其他库
target_link_libraries(pf_control
  Eigen3::Eigen
  ft_servo_motor_pkg::ft_servo_motor_pkg
)

# 将编译好的库文件安装到相应目录
# 一般位于install/${PROJECT_NAME}/lib下
# 并且指定了寻找该共享库的头文件的位置
# export指令将该模块设定为导出模块
install(
  TARGETS pf_control
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
  INCLUDES DESTINATION include
)

# 在install/${PROJECT_NAME}/share/${PROJECT_NAME}下将展开相关配置
# 使得其他项目可以通过功能包名称定位该模块
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint 检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()