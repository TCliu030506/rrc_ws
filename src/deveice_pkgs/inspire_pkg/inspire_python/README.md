# 功能包说明
## 节点说明
1. [Node] record:     save inspire_msg in file: /record

2. [Node] echo:       print msg
    - [Params] {id: <int>}
    - exp: ros2 run inspire_python echo --ros-args -p "id:=1"

# 使用说明

# 提供了inspire_client模块直接控制因时电机
from inspire_python import inspire_client
sudo chmod 666 /dev/ttyUSB0

# 启动因时电机节点
ros2 launch inspire_actuator insactuator.launch.py 
# 监听电机状态 位置 力传感数据
ros2 topic echo /insactuator_msg
# 控制电机
ros2 run inspire_python szmd


产品参数：
力传感器检测范围：-100～100N     力传感器分辨率：1N
最大推拉力：50N    堵转推拉力：80N    最大自锁力：80N
输入的力控参数单位：g

不要让推杆到达极端位置 是准的
输入正数：按住力传感器 超过目标力控值推杆缩回 低于目标力控值推杆伸长
输入负数：拉住力传感器 超过目标力控值推杆伸长 低于目标力控值推杆缩回
所以，想用推杆施加推力 输入正数；想施加拉力 输入负数
