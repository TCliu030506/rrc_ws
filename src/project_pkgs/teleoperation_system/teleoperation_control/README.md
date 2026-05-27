# 功能包介绍
teleoperation_control功能包实现了利用Sigema7对xMate7机械臂的遥控操作，采用C++模块编写。
# 依赖
- sigema7_getpos
- sigema7_msg
- x_core_sdk
- rclcpp

# 节点功能介绍
## teleoperation_control_node
通过机械臂的非实时运动控制指令，实现对xMate7机械臂的控制。
## teleoperation_control_node_rt
通过机械臂的实时运动控制指令，实现对xMate7机械臂的控制。
## teleoperation_control_node_fp
通过机械臂的点位跟随控制指令，实现对xMate7机械臂的控制。
## system_control_node_ui
通过机械臂的实时运动控制指令，实现对xMate7机械臂的控制。集成了对pf的遥操作。利用UI界面实现对模式和映射比例进行条再

# 使用方法
1. 设备连接USB端口后打开终端，运行：lsusb
   获取bus usb 号
2. 运行：sudo chmod 777 /dev/bus/usb/003/010   （后面两数字对应bus/usb号）
3. 运行：ros2 launch ui_test ui_test.launch.py
