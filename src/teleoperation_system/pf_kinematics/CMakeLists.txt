cmake_minimum_required(VERSION 3.8)
project(pf_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  include/pf_kinematics
)

# 生成可执行文件
add_executable(pf_kinematics_test src/pf_kinematics_test.cpp src/pf_kinematics.cpp)
install(TARGETS pf_kinematics_test DESTINATION lib/${PROJECT_NAME})

# 将include目录下的所有头文件安装到include目录下
install(
  DIRECTORY include/ 
  DESTINATION include
)

# 生成共享库文件
add_library(pf_kinematics SHARED src/pf_kinematics.cpp)

target_include_directories(pf_kinematics
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 将编译好的库文件安装到相应目录
# 一般位于install/${PROJECT_NAME}/lib下
# 并且指定了寻找该共享库的头文件的位置
# export指令将该模块设定为导出模块
install(
  TARGETS pf_kinematics
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
