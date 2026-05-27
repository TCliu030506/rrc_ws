cmake_minimum_required(VERSION 3.8)
project(x_core_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(
  include/x_core_sdk
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XcoreState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_library(xCoreSDK_static STATIC IMPORTED)
set_target_properties(xCoreSDK_static PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/x_core_cpp_sdk/libxCoreSDK.a"
)

# 创建共享库（.so 文件）
add_library(xCoreSDK_shared SHARED IMPORTED)
set_target_properties(xCoreSDK_shared PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/x_core_cpp_sdk/libxCoreSDK.so.0.4.1"
)

add_library(xMateModel STATIC IMPORTED)
set_target_properties(xMateModel PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/x_core_cpp_sdk/libxMateModel.a"
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/x_core_cpp_sdk/libxCoreSDK.a
        DESTINATION lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/x_core_cpp_sdk/libxCoreSDK.so.0.4.1
        DESTINATION lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/x_core_cpp_sdk/libxMateModel.a
        DESTINATION lib)


# 安装头文件
install(DIRECTORY include/x_core_sdk
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(xCoreSDK_shared xCoreSDK_static xMateModel)



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
