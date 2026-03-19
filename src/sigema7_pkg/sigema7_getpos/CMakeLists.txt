cmake_minimum_required(VERSION 3.8)
project(sigema7_getpos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sigema7_msg REQUIRED)
find_library(DHDLIB NAMES dhd PATHS "${PROJECT_SOURCE_DIR}/lib" NO_DEFAULT_PATH)
find_library(DRDLIB NAMES drd PATHS "${PROJECT_SOURCE_DIR}/lib" NO_DEFAULT_PATH)

add_executable(lab_getdata src/lab_getdata.cpp)
add_executable(sigema7_pub_test src/sigema7_pub_test.cpp)
add_executable(delay_test src/delay_test.cpp)

ament_target_dependencies(lab_getdata 
  rclcpp std_msgs)

ament_target_dependencies(sigema7_pub_test 
  rclcpp std_msgs sigema7_msg)

ament_target_dependencies(delay_test 
  rclcpp std_msgs )


set(CAN_LIB lib)  
message(STATUS "Linking libraries...")  
target_link_libraries(lab_getdata  
    ${PROJECT_SOURCE_DIR}/${CAN_LIB}/libdhd.a 
    ${PROJECT_SOURCE_DIR}/${CAN_LIB}/libdrd.a
    libusb-1.0.so    
)  

target_link_libraries(sigema7_pub_test  
    ${PROJECT_SOURCE_DIR}/${CAN_LIB}/libdhd.a 
    ${PROJECT_SOURCE_DIR}/${CAN_LIB}/libdrd.a
    libusb-1.0.so    
)  

target_link_libraries(delay_test
    ${PROJECT_SOURCE_DIR}/${CAN_LIB}/libdhd.a 
    ${PROJECT_SOURCE_DIR}/${CAN_LIB}/libdrd.a
    libusb-1.0.so    
)


message(STATUS "Installing libraries...")  
install(DIRECTORY ${CAN_LIB}/  DESTINATION lib/) 

install(TARGETS lab_getdata DESTINATION lib/${PROJECT_NAME})
install(TARGETS sigema7_pub_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS delay_test DESTINATION lib/${PROJECT_NAME})

add_library(libdhd STATIC IMPORTED)
set_target_properties(libdhd PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.a"
)

add_library(libdrd STATIC IMPORTED)
set_target_properties(libdrd PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.a"
)
# 安装共享库

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.a
        DESTINATION lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.a
        DESTINATION lib)

# 安装头文件
install(DIRECTORY include/
        DESTINATION include)

# 导出头文件路径
ament_export_include_directories(include)
ament_export_libraries(libdhd libdrd)

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
