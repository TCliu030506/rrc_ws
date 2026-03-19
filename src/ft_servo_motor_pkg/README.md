# 功能包介绍
飞特舵机STS5420M舵机控制功能包,采用C++编写

## 底层文件逻辑
通信层:SCS
----------------------------
硬件接口层:SCSerail
----------------------------
应用层:SMSBL SMSCL SCSCL分别对应飞特三个系列舵机
应用层:SMS_STS兼容SMS/STS系列舵机

SMSBL sm;//定义SMSBL系列舵机
SMSCL sm;//定义SMSCL系列舵机
SCSCL sc;//定义SCSCL系列舵机
SMS_STS sm_st;//定义SMS/STS系列舵机
SMSCL\SMSBL\SCSCL\SMS_STS各自接口参考相应头文件

INST.h---指令定义头文件
SCS.h/SCS.cpp---通信层程序
SCSerial.h/SCSerial.cpp---硬件接口程序
SMSBL.h/SMSBL.cpp---SMSBL应用层程序
SMSCL.h/SMSCL.cpp---SMSCL应用层程序
SCSCL.h/SCSCL.cpp---SCSCL应用层程序
SMS_STS.h/SMS_STS.cpp---SMS/STS应用层程序
(内存表定义于应用层程序头文件SMSBL.h\SMSCL.h\SCSCL.h\SMS_STS.h中不同系列舵内存表定义存在差异)

                       SMSBL类
SCS类<---SCSerial类<---SMSCL类
                       SCSCL类
                       SMS_STS类

## 各节点具体功能可在文件中查阅

# 使用方法
1. 获取设备串口号，运行：ls /dev/ttyUSB*
2. 开启串口权限，运行：sudo chmod 666 /dev/ttyUSB0（根据第一步的实际结果打开对应的串口） 
3. 运行节点（以test_step为例），运行：ros2 run ft_servo_motor_pkg test_step /dev/ttyUSB0   （先定位到工作空间后source）

