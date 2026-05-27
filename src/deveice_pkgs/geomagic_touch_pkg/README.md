# Geomagic_Touch_ROS2
ROS2 Port of the ROS Driver for the Geomagic Touch Haptic Interface 

## General

This repository contains a port of the Geomagic Touch ROS driver provided by Bharatm11 at: https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers.git

## installation of Geomagic drivers 

To use the Geomagic Touch interface you first need to install both the Touch Device drivers and Openhaptics drivers provided at: (Ubuntu 20.04)
https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US

## Usage
To use the ROS2 driver, connect the interface to the computer and check that there is connection
Then simply launch via:
```
ros2 launch omni_common omni_state.launch.py
```



## 此功能包为Geomagic Touch的ROS2接口功能包
## 需说明的问题：
1. 本功能包不包含驱动，需要用户自己安装驱动。
2. 本功能包中提供了使用Geomagic Touch函数的ROS2接口，用户可以调用Geomagic Touch的函数。详细建omni_common中的例程（请勿修改）！

## 节点说明
1. omni_state节点：用于发布Geomagic Touch的状态信息。
2. omni节点：用于控制Geomagic Touch运动（类似于弹簧的效果）