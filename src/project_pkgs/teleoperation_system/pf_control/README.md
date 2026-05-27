# 使用方法
1. 获取设备串口号，运行：ls /dev/ttyUSB*
2. 开启串口权限，运行：sudo chmod 666 /dev/ttyUSB0（根据第一步的实际结果打开对应的串口） 
3. 运行节点（以test_step为例），运行：ros2 run pf_control pf_control_test   （先定位到工作空间后source）

# 0909 新一次装配使用
角度偏差：62（+2） 58（-2） 55（-5）

# 260112 新一次装配使用
角度偏差：62（+2） 62（+2） 58（-2）