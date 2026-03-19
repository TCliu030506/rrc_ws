cmake_minimum_required(VERSION 3.8)
project(ft_servo_motor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(write_pos src/write_pos.cpp src/SCS.cpp src/SCSCL.cpp src/SCSerial.cpp src/SMS_STS.cpp src/SMSBL.cpp src/SMSCL.cpp) 
add_executable(broadcast src/broadcast.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(calibration_ofs src/calibration_ofs.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(feedback src/feedback.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(ping src/ping.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(program_eprom src/program_eprom.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(reg_write_pos src/reg_write_pos.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(sync_read src/sync_read.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(sync_write_pos src/sync_write_pos.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(test_step src/test_step.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(write_pos src/write_pos.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 
add_executable(write_spe src/write_spe.cpp include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SMSCL.cpp) 

include_directories(
  include
  include/ft_servo_motor_pkg
)

install(TARGETS broadcast DESTINATION lib/${PROJECT_NAME})
install(TARGETS calibration_ofs DESTINATION lib/${PROJECT_NAME})
install(TARGETS feedback DESTINATION lib/${PROJECT_NAME})
install(TARGETS ping DESTINATION lib/${PROJECT_NAME})
install(TARGETS program_eprom DESTINATION lib/${PROJECT_NAME})
install(TARGETS reg_write_pos DESTINATION lib/${PROJECT_NAME})
install(TARGETS sync_read DESTINATION lib/${PROJECT_NAME})
install(TARGETS sync_write_pos DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_step DESTINATION lib/${PROJECT_NAME})
install(TARGETS write_pos DESTINATION lib/${PROJECT_NAME})
install(TARGETS write_spe DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY include/ 
  DESTINATION include
)



# 生成共享库文件
# add_library(ft_servo_motor_pkg SHARED include/ft_servo_motor_pkg/SCS.cpp include/ft_servo_motor_pkg/SCSCL.cpp include/ft_servo_motor_pkg/SCSerial.cpp include/ft_servo_motor_pkg/SCServo.cpp include/ft_servo_motor_pkg/SCS.h include/ft_servo_motor_pkg/SMS_STS.cpp include/ft_servo_motor_pkg/SMSBL.cpp include/ft_servo_motor_pkg/SCSCL.cpp)

add_library(ft_servo_motor_pkg SHARED src/SCS.cpp src/SCSCL.cpp src/SCSerial.cpp src/SMS_STS.cpp src/SMSBL.cpp src/SMSCL.cpp)


target_include_directories(ft_servo_motor_pkg
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 将编译好的库文件安装到相应目录
# 一般位于install/${PROJECT_NAME}/lib下
# 并且指定了寻找该共享库的头文件的位置
# export指令将该模块设定为导出模块

install(
  TARGETS ft_servo_motor_pkg
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
  INCLUDES DESTINATION include
)

# 在install/${PROJECT_NAME}/share/${PROJECT_NAME}下将展开相关配置
# 使得其他项目可以通过功能包名称定位该模块
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)




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
