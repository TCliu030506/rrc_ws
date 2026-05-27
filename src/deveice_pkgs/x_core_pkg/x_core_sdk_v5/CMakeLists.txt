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
find_package(Eigen3 REQUIRED)

include_directories(
  include/x_core_sdk_v5
)

# 创建静态库
add_library(xCoreSDKv5_static STATIC IMPORTED)
set_target_properties(xCoreSDKv5_static PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libxCoreSDK.a"
)

# 创建共享库（.so 文件）
add_library(xCoreSDKv5_shared SHARED IMPORTED)
set_target_properties(xCoreSDKv5_shared PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libxCoreSDK.so.0.5.0"
)

add_library(xMateModelv5 STATIC IMPORTED)
set_target_properties(xMateModelv5 PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libxMateModel.a"
)
# 安装库
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libxCoreSDK.a
        DESTINATION lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libxCoreSDK.so.0.5.0
        DESTINATION lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libxMateModel.a
        DESTINATION lib)

# 安装头文件
# 将头文件以兼容旧包含路径的方式安装到 include/x_core_sdk/
# 这样下游可以继续使用 #include "x_core_sdk/<header>.h"
install(DIRECTORY include/x_core_sdk_v5/
  DESTINATION include/x_core_sdk
)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(xCoreSDKv5_static xCoreSDKv5_shared xMateModelv5)


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
