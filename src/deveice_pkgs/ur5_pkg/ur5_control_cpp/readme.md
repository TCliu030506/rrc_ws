生成控制ur5运动的c++库文件
使用说明
1.包含头文件
#include "ur5_control_cpp/urscript.h"
2.代码使用类ur_script::script_pub定义一个变量，如：
ur_script::script_pub urscript;
3.调用类的成员函数(其中pose的位姿表示为轴角，单位为m和rad，joint的单位为rad)
urscript_publish_stopj(double acc)
urscript_publish_stopl(double acc)
urscript_publish_speedj(double (&vel)[6], double acc, double tim)
urscript_publish_speedl(double (&vel)[6], double acc, double tim)
urscript_publish_movej_pose(double (&pose)[6], double acc, double vel, double tim, double rad)
urscript_publish_movej_joint(double (&joint)[6], double acc, double vel, double tim, double rad)
urscript_publish_movel_pose(double (&pose)[6], double acc, double vel, double tim, double rad)
urscript_publish_movel_joint(double (&joint)[6], double acc, double vel, double tim, double rad)
urscript_publish_movep_pose(double (&pose)[6], double acc, double vel, double rad)
urscript_publish_movep_joint(double (&joint)[6], double acc, double vel, double rad)


CmakeLists.txt中添加：
find_package(ur5_control_cpp REQUIRED)
include_directories(${ur5_control_cpp_INCLUDE_DIRS})
ament_target_dependencies(可执行文件名称 ...其余依赖... ur5_control_cpp)
target_link_libraries(ur5_aim_calibration ${ur5_control_cpp_LIBS})

package.xml中添加：
<depend>ur5_control_cpp</depend>