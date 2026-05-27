## 关于本项目

手柄控制机械臂、捷源电缸和脉塔电机。
按LB为控制机械臂模式，按RB为控制电机模式。默认为控制机械臂模式，两种模式可以来回切换。
控制机械臂模式：
  B按键控制机械臂末端逐次直线运动至预先设定的点  按B键运动至最后一个设定的目标点后再按两次B键 之后B键功能切换为从当前位置沿通道方向拔出
  XX键控制机械臂左右微调
  YY键控制机械臂前后微调
  Y、A键控制机械臂上下微调
  左摇杆控制机械臂XY角度微调
  LT、RT键控制机械臂Z角度微调
  X键从当前位置沿通道位置插入 第一次按粗调 后面为精调 

控制电机模式：
  左摇杆控制工作通道绝对位置，摇杆方向代表弯曲方向，摇杆位移代表弯曲幅度。
  键盘微调位置，上下键控制弯曲，左右键控制旋转。
  A键为复位键。
  B键为锁死键。
  X键为解锁键。

## 手柄使用说明

### 连接好设备之后检查端口号
- cat /proc/bus/input/devices

$ 回复
I: Bus=0003 Vendor=045e Product=028e Version=0110
N: Name="Microsoft X-Box 360 pad"
P: Phys=usb-0000:03:00.0-2/input0
S: Sysfs=/devices/pci0000:00/0000:00:15.0/0000:03:00.0/usb3/3-2/3-2:1.0/input/input7
U: Uniq=
H: Handlers=event6 js1 
B: PROP=0
B: EV=20000b
B: KEY=7cdb000000000000 0 0 0 0
B: ABS=3003f
B: FF=107030000 0
$

### 读取游戏手柄端口数据

#### 检查端口
- ls /dev/input

#### 输出端口数据
- cat /dev/input/js1 | hexdump

#### 读取端口数据
- sudo apt install joystick
- jstest /dev/input/js1

###

## 开始

连接好设备后，可以按照下面的步骤操作。

### 1.串口设置

查看串口是否在使用
```sh
ls -l /dev/ttyUSB*
  ```
若出现：无法访问 ‘/dev/ttyUSB’: 没有那个文件或目录，运行
```sh
sudo dmesg | grep brltty
```
运行后若出现 
[ 7033.078452] usb 1-13: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1 这样的结果，则意味着是驱动占用问题，运行
```
sudo apt remove brltty
```
然后重新插拔一下设备即可解决问题。

### 2.运行节点

开第一个终端，捷源电机节点打开
```sh
cd ~/zzrobot_ws
source install/local_setup.sh
ls -l /dev/ttyUSB0
ros2 run jyactuator jyactuator_server_node
```
开第二个终端，脉塔电机节点打开
```sh
cd ~/zzrobot_ws
source install/local_setup.sh
ls -l /dev/ttyUSB1
ros2 run mtactuator mtactuator_server_node
```
开第三个终端，机械臂驱动节点打开
```sh
cd ~/zzrobot_ws
source install/local_setup.sh
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=false
```
开第四个终端，手柄输入节点，监测手柄输入
```sh
cd ~/zzrobot_ws
source install/local_setup.sh
ros2 run joystick joystick_input_node 
```
开第五个终端，手柄控制节点，接收指令控制电机运动
```sh
cd ~/zzrobot_ws
source install/local_setup.sh
ros2 run joystick joystick_control_extended_node 
```


### 3.手柄控制


## 联系我们

项目链接: [https://gitee.com/cloudriver0226/zzrobot_ws](https://gitee.com/cloudriver0226/zzrobot_ws)

<p align="right"><a href="#top">返回顶部</a></p>



