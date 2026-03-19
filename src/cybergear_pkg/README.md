## 关于本项目

CyberGear小米电机控制

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

修改串口权限
```sh
sudo chmod 666 /dev/ttyUSB0
```

### 2.运行节点

ros2 run cybergear_actuator cybergear_server_node