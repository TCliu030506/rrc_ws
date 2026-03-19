##

使用说明

###

## 运行节点
```sh
ros2 run cybergear_actuator cybergear_server_node
```
topic: 'cybergear_msg', type: coordinate::msg::ArrayInt16 service: 'cybergear_srv', type: cybergear_msg::srv::CybergearScript

```sh
ros2 launch cybergear_actuator cybergear.launch.py
```
topic: 'cybergear_msg', type: coordinate::msg::ArrayInt16 service: 'cybergear_srv', type: cybergear_msg::srv::CybergearScript

ROS_SRV: ros2 service call /cybergear_srv cybergear_msg/srv/CybergearScript "{command: ' ', id: , data: [ ]}"

### 1.修改端口配置

cybergear.launch.py会加载param/cybergear.launch.py,修改YAML文件即可


### 2.运行节点
```sh
source install/setup.bash
```
```sh
ros2 run cybergear_actuator cybergear_server_node
```
```sh
ls /dev/ttyUSB*
```