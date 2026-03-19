# 功能包介绍
sigema7遥操作模块,采用C++编写

## lab_getdata   // 最初测试用
完成设备的初始化并按照指定频率读取位置信息

## lab_getpos
完成设备的初始化并按照指定频率读取位置、XYZ直线速度、力信息，并发布在话题sigema7_getpos里

# 使用方法
1. 设备连接USB端口后打开终端，运行：lsusb
   获取bus usb 号
2. 运行：sudo chmod 777 /dev/bus/usb/003/004   （后面两数字对应bus/usb号）
3. 运行：ros2 run sigema7_pkg lab_getdata   （先定位到工作空间后source）
4. 运行：ros2 run sigema7_pkg lab_getpos   （先定位到工作空间后source）
