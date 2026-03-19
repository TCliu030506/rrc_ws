# 功能包介绍

## coordinate pkg
### 定义了坐标相关的常用消息
ros2 interface show coordinate/msg/Coord

# aimooe pkg
## 通过USB端口获取aimooe光定位仪坐标信息
ros2 run aimooe pub_tracker

# inspire_msg pkg
## 定义了inspire actuator相关的常用消息
ros2 interface show inspire_msg/msg/RobotState

# inspire_actuator pkg
## 发布了inspire actuator相关传感器数据，提供了指令控制的接口服务
ros2 run inspire_actuator insactuator_server_node

# serial pkg
## 第三方串口功能包

# 常用指令
## 创建C++功能包
ros2 pkg create --build-type ament_cmake <package_name>

## 创建Python功能包
ros2 pkg create --build-type ament_python <package_name>

## 编译工作空间
colcon build

## 运行功能包前先添加工作空间环境变量
source ~/zzrobot_ws/install/setup.sh

## 查看功能包的消息
ros2 interface show coordinate/msg/Coord

## 可视化查看发布数据
ros2 run rqt_plot rqt_plot

# 其他注意事项

## 锁定设备端口号
modify /etc/udev/rules.d/

### 查看设备端口号
lsusb
sudo nano /etc/udev/rules.d/99-usb-serial.rules