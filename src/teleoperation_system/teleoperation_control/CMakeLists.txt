cmake_minimum_required(VERSION 3.8)
project(teleoperation_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(x_core_sdk REQUIRED)
find_package(xmate_cr7_msg REQUIRED)
find_package(sigema7_msg REQUIRED)
find_package(sigema7_getpos REQUIRED)
find_package(pf_control REQUIRED)
find_package(pf_kinematics REQUIRED)
find_package(ft_servo_motor_pkg REQUIRED)
find_package(ui_control_msg REQUIRED)
find_package(aimooe_sdk REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(${x_core_sdk_INCLUDE_DIRS})
include_directories(${pf_control_INCLUDE_DIRS})
include_directories(${sigema7_getpos_INCLUDE_DIRS})


# add_executable(teleoperation_control_node src/teleoperation_control_node.cpp)
# add_executable(teleoperation_control_node_fp src/teleoperation_control_node_fp.cpp)
# add_executable(teleoperation_control_node_rt src/teleoperation_control_node_rt.cpp)
add_executable(teleoperation_control_node_rtloop src/teleoperation_control_node_rtloop.cpp)
# add_executable(pf_teleoperation_node_vel src/pf_teleoperation_node_vel.cpp)
add_executable(pf_teleoperation_node_pos src/pf_teleoperation_node_pos.cpp)
# add_executable(math_test src/math_test.cpp)
add_executable(system_control_node src/system_control_node.cpp)
# add_executable(delay_test src/delay_test.cpp)
# add_executable(trans_coordinate src/trans_coordinate.cpp)
# add_executable(joint_position_control src/joint_position_control.cpp)
add_executable(system_control_node_ui src/system_control_node_ui.cpp)
add_executable(exp_task_invest src/exp_task_invest.cpp)
add_executable(exp_task_invest_world src/exp_task_invest_world.cpp)



add_executable(sigema7_getpos src/sigema7_getpos.cpp)
add_executable(sigema7_getpos_old src/sigema7_getpos_old.cpp)
add_executable(sigema7_control src/sigema7_control.cpp)


# ament_target_dependencies(teleoperation_control_node x_core_sdk sigema7_msg rclcpp)
# ament_target_dependencies(teleoperation_control_node_fp x_core_sdk sigema7_msg rclcpp)
# ament_target_dependencies(teleoperation_control_node_rt x_core_sdk sigema7_msg rclcpp)
ament_target_dependencies(teleoperation_control_node_rtloop x_core_sdk sigema7_msg rclcpp xmate_cr7_msg Eigen3)
# ament_target_dependencies(pf_teleoperation_node_vel pf_control pf_kinematics ft_servo_motor_pkg sigema7_msg rclcpp)
ament_target_dependencies(pf_teleoperation_node_pos pf_control pf_kinematics ft_servo_motor_pkg sigema7_msg rclcpp)
ament_target_dependencies(system_control_node pf_control pf_kinematics ft_servo_motor_pkg sigema7_msg rclcpp x_core_sdk xmate_cr7_msg)
# ament_target_dependencies(delay_test x_core_sdk sigema7_getpos rclcpp)
# ament_target_dependencies(trans_coordinate x_core_sdk )
# ament_target_dependencies(joint_position_control x_core_sdk )
ament_target_dependencies(system_control_node_ui pf_control pf_kinematics ft_servo_motor_pkg sigema7_msg rclcpp x_core_sdk xmate_cr7_msg ui_control_msg)
ament_target_dependencies(exp_task_invest pf_control pf_kinematics ft_servo_motor_pkg sigema7_msg rclcpp x_core_sdk xmate_cr7_msg ui_control_msg)
ament_target_dependencies(exp_task_invest_world pf_control pf_kinematics ft_servo_motor_pkg sigema7_msg rclcpp x_core_sdk xmate_cr7_msg ui_control_msg)



ament_target_dependencies(sigema7_getpos 
  rclcpp std_msgs sigema7_msg xmate_cr7_msg ui_control_msg)
ament_target_dependencies(sigema7_control 
  rclcpp std_msgs xmate_cr7_msg)
ament_target_dependencies(sigema7_getpos_old 
  rclcpp std_msgs sigema7_msg xmate_cr7_msg)

# target_link_libraries(teleoperation_control_node ${x_core_sdk_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES}) 
# target_link_libraries(teleoperation_control_node_fp ${x_core_sdk_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES}) 
# target_link_libraries(teleoperation_control_node_rt ${x_core_sdk_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES}) 
target_link_libraries(teleoperation_control_node_rtloop ${x_core_sdk_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES} ${xmate_cr7_msg_LIBRARIES}) 
# target_link_libraries(pf_teleoperation_node_vel ${pf_control_LIBRARIES} ${pf_kinematics_LIBRARIES} ${ft_servo_motor_pkg_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES}) 
target_link_libraries(pf_teleoperation_node_pos ${pf_control_LIBRARIES} ${pf_kinematics_LIBRARIES} ${ft_servo_motor_pkg_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES}) 
target_link_libraries(system_control_node ${pf_control_LIBRARIES} ${pf_kinematics_LIBRARIES} ${ft_servo_motor_pkg_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES} ${x_core_sdk_LIBRARIES} ${xmate_cr7_msg_LIBRARIES}) 
# target_link_libraries(delay_test ${x_core_sdk_LIBRARIES} ${sigema7_getpos_LIBRARIES} libusb-1.0.so   ${rclcpp_LIBRARIES}) 
# target_link_libraries(trans_coordinate ${x_core_sdk_LIBRARIES}) 
# target_link_libraries(joint_position_control ${x_core_sdk_LIBRARIES}) 
target_link_libraries(system_control_node_ui ${pf_control_LIBRARIES} ${pf_kinematics_LIBRARIES} ${ft_servo_motor_pkg_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES} ${x_core_sdk_LIBRARIES} ${xmate_cr7_msg_LIBRARIES}) 
target_link_libraries(exp_task_invest ${pf_control_LIBRARIES} ${pf_kinematics_LIBRARIES} ${ft_servo_motor_pkg_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES} ${x_core_sdk_LIBRARIES} ${xmate_cr7_msg_LIBRARIES}) 
target_link_libraries(exp_task_invest_world ${pf_control_LIBRARIES} ${pf_kinematics_LIBRARIES} ${ft_servo_motor_pkg_LIBRARIES} ${sigema7_msg_LIBRARIES} ${rclcpp_LIBRARIES} ${x_core_sdk_LIBRARIES} ${xmate_cr7_msg_LIBRARIES}) 



# 获取 sigema7_getpos 的 install 路径
target_link_libraries(sigema7_getpos
  ${CMAKE_INSTALL_PREFIX}/../sigema7_getpos/lib/libdhd.a
  ${CMAKE_INSTALL_PREFIX}/../sigema7_getpos/lib/libdrd.a
  libusb-1.0.so
)
target_link_libraries(sigema7_control
  ${CMAKE_INSTALL_PREFIX}/../sigema7_getpos/lib/libdhd.a
  ${CMAKE_INSTALL_PREFIX}/../sigema7_getpos/lib/libdrd.a
  libusb-1.0.so
)
target_link_libraries(sigema7_getpos_old
  ${CMAKE_INSTALL_PREFIX}/../sigema7_getpos/lib/libdhd.a
  ${CMAKE_INSTALL_PREFIX}/../sigema7_getpos/lib/libdrd.a
  libusb-1.0.so
)


# install(TARGETS teleoperation_control_node DESTINATION lib/${PROJECT_NAME})
# install(TARGETS teleoperation_control_node_fp DESTINATION lib/${PROJECT_NAME})
# install(TARGETS teleoperation_control_node_rt DESTINATION lib/${PROJECT_NAME})
install(TARGETS teleoperation_control_node_rtloop DESTINATION lib/${PROJECT_NAME})
# install(TARGETS pf_teleoperation_node_vel DESTINATION lib/${PROJECT_NAME})
install(TARGETS pf_teleoperation_node_pos DESTINATION lib/${PROJECT_NAME})
# install(TARGETS math_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS system_control_node DESTINATION lib/${PROJECT_NAME})
# install(TARGETS delay_test DESTINATION lib/${PROJECT_NAME})
# install(TARGETS trans_coordinate DESTINATION lib/${PROJECT_NAME})
# install(TARGETS joint_position_control DESTINATION lib/${PROJECT_NAME})
install(TARGETS system_control_node_ui DESTINATION lib/${PROJECT_NAME})
install(TARGETS exp_task_invest DESTINATION lib/${PROJECT_NAME})
install(TARGETS exp_task_invest_world DESTINATION lib/${PROJECT_NAME})



install(TARGETS sigema7_getpos DESTINATION lib/${PROJECT_NAME})
install(TARGETS sigema7_control DESTINATION lib/${PROJECT_NAME})
install(TARGETS sigema7_getpos_old DESTINATION lib/${PROJECT_NAME})


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
