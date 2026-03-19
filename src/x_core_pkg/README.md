## 关于本项目

- 珞石xMate CR7机械臂。

## 开始

- 编译通过后，可以运行相关程序。

### X_CORE_CPP功能包

 - 已经存在一些珞石xMate CR7机械臂相关示例程序。

### X_CORE_SDK功能包

 - 将珞石xMate CR7机械臂相关的C++以及python的SDK文件配置到ROS2环境中，使得其他功能包可以直接调用。

### X_CORE_SDK_V5功能包(更新版的SDK功能包)

 - 将珞石xMate CR7机械臂相关的C++以及python的SDK文件配置到ROS2环境中，使得其他功能包可以直接调用。

### XMATECR7功能包

 - 珞石xMate CR7机械臂rviz2的可视化模型。

### XMATE_CR7_CONTROL功能包

 - 珞石xMate CR7机械臂经封装后的控制功能包。(基于xmate_cr7_msg)
 - 其主要功能是：
   - 创建一个服务器，直接与机械臂进行通信。
   - 可以按照“cr7_client”的格式，创建客户端与服务器进行通信，从而控制机械臂的相关功能。

### XMATE_CR7_SCRIPT功能包
 - 珞石xMate CR7机械臂的脚本功能包。(基于coordinate)
 - 其主要功能是：
   - cr7_server:创建一个服务器，直接与机械臂进行通信。
   - cr7_script:创建一个cr7_script服务器脚本函数，可调用与服务器进行通信。
   - cr7_state_sensor:创建一个周期发布机械臂状态消息的节点。
   - cr7_force_sensor:创建一个周期发布机械臂力传感器消息的节点。
   - cr7_script_client_demo:创建一个客户端实例，说明了如何调用cr7_script服务器脚本函数。
 - 统一启动文件：launch/cr7_script.launch.py
 - 运行示例：
   - ros2 launch xmate_cr7_script cr7_driver.launch.py force_poll_hz:=50 state_poll_hz:=50 

