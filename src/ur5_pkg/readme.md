# 使用说明

## 该功能包依赖UR驱动功能包,安装方法:
- sudo apt-get install ros-humble-ur

### 说明文档
- https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble

## 此功能包适用UR5机械臂，包括如下功能包：
- ur5_control       控制机械臂运动的python实现,内运动学求解模块   --Python
  - URControl       机械臂指令发送模块，想要与机械臂通讯并向其发送消息可导入该模块
  - ur5_kinematics  正逆运动学python实现
  - ur5_pos_init    机械臂位置初始化节点
  - move_demo       运动控制demo节点
  - linearmotion_demo 控制机械臂直线运动（暂未完善）    
- ur5_kinematics    机械臂正逆运动学计算的C++实现               --C++
- ur5_msg           将机械臂的位姿信息分类存储后发布             --C++
  - msg.RobotState.msg 为与机械臂通信的消息格式

## 具体使用前，可参照各功能包的demo

## 运行方法：
## 连接ur5并发布话题robotstate
- ros2 launch ur5_msg ur5.launch.py
## 新建终端，启用驱动     ##开启后不能关闭
- ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=false
## 新建终端，编译
- cd ~/zzrobot_ws
- colcon build
## 运行功能包内节点
- ros2 run 功能包 节点