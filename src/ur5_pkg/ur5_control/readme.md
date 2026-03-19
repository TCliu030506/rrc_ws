## 此功能包适用UR5机械臂，包括如下功能包：
  - URControl       机械臂指令发送模块，想要与机械臂通讯并向其发送消息可导入该模块
  - ur5_kinematics  正逆运动学python实现
  - ur5_pos_init    机械臂位置初始化节点
  - move_demo       运动控制demo节点
  - linearmotion_demo 控制机械臂直线运动（暂未完善）    

## 具体使用前，可参照各功能包的demo

## 运行方法：
## 新建终端，启用驱动     ##开启后不能关闭
- ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=false
## 新建终端，编译
- cd ~/zzrobot_ws
- colcon build
## 运行功能包内节点
- ros2 run 功能包 节点


## 各节点使用具体介绍
#  ur5_pos_init    
- 功能：机械臂位置初始化节点
- 运行后，循环检测输入：
  - 0    机械臂回到停止位置（便于长期放置与搬运）
  - 1    机械臂运动至工作位置 （设为工作初始位置）
