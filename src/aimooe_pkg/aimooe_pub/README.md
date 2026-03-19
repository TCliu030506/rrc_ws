# 功能包说明
## 节点说明
1. [Node] pub_trackers:     pub trackers msg
    - [Publisher] <aimooe_sdk::msg::AimCoord> {"aimooe_tracker"}
    - [Params] (# freq = 0.1)

2. [Node] pub_test:         pub fake trackers msg
    - [Publisher] <aimooe_sdk::msg::AimCoord> {"aimooe_tracker"}
    - [Params] {base_frame: <string>}
    - exp: ros2 run aimooe_pub pub_test --ros-args -p "frame:='fake_tool'"

3. [Node] tool_add:         add tool to "/Aimtools"

4. [Node] tool_calib:       tool calibration

5. [Node] tool_tip:         register tip point

6. [Node] tool_transfer:    register tool coordinate

7. [Node] pub_coords:       pub trackers msg in ref coordinate
    - [Subscriber] <aimooe_sdk::msg::AimCoord> {"aimooe_tracker"}
    - [Publisher] <aimooe_sdk::msg::AimCoord> {"aimooe_coord"}
    - [Params] {base_frame: <string>}
    - exp: ros2 run aimooe_pub pub_coords --ros-args -p "base_frame:='ZZR-base'"

8. [Node] aim_demo:         set DualExpTime of camera


# 使用说明

## 发布Aimtools文件夹下所有工具的坐标信息
信息格式:topic: 'aimooe_tracker', type: aimooe_sdk::msg::AimCoord
- ros2 run aimooe_pub pub_trackers
or
- ros2 launch aimooe_pub aimooe.launch.py

### 以"fake_tool"为工具名发布空坐标消息:
- ros2 launch aimooe_pub aimooe_fake.launch.py

## 注册新的工具
- ros2 run aimooe_pub tool_add

## 校准已有工具
- ros2 run aimooe_pub tool_calib

## 绕点注册
- ros2 run aimooe_pub tool_tip

## 标定转移坐标系
- ros2 run aimooe_pub tool_transfer

# 故障排查

## 如果能找到Aimooe设备但无法建立通讯, 打开访问权限
sudo cp ~/zzrobot_ws/src/aimooe/dev/aimooe-usb-ethernet.rules /etc/udev/rules.d/

## 终端运行出现Segmentation fault
可能是没有连接光定位或者设备没有开机

## 重新加载 udev 规则
sudo udevadm control --reload-rules

# 输出当前Python环境变量包含的路径
import sys
for path in sys.path:
    print(path)

# 获取LD_LIBRARY_PATH的值
import os
ld_library_path = os.environ.get('LD_LIBRARY_PATH')

# 打印LD_LIBRARY_PATH的值
print("LD_LIBRARY_PATH:", ld_library_path)

# 报错：[ros2run]: Segmentation fault
重新插拔或者重启光定位设备，然后重新运行节点

## unknown problem
[pub_trackers-1] 2024-03-19 21:26:38.071 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7413: open_and_lock_file failed -> Function open_port_internal



# 使用步骤说明
## 1、注册工具
- ros2 run aimooe_pub tool_add
## 2、校准工具
- ros2 run aimooe_pub tool_calib

## 3、循环1-2步直至所有工具都校准完成

## 4、注册探针
- ros2 run aimooe_pub tool_tip

## 5、转移坐标系（逐个对需要转移的工具进行操作）
- ros2 run aimooe_pub pub_trackers
- ros2 run aimooe_pub tool_transfer

## 6、发布相对于某个工具（以“HYC”这个工具为例）的坐标信息
### 6.1 先将坐标发布至“aimooe_tracker”话题
- ros2 run aimooe_pub pub_trackers
- ros2 run aimooe_pub pub_trackers_marker(仅适用于LTC)
- ros2 run aimooe_pub pub_trackers_8markers(仅适用于刚柔耦合蛇形臂)
### 6.2 将“aimooe_tracker”话题下的坐标发送至“aimooe_coord”话题
- ros2 run aimooe_pub pub_coords --ros-args -p base_frame:=HYC

## 7、自行编写节点或者通过订阅“aimooe_coord”话题获取坐标信息
- ros2 topic echo /aimooe_coord

# szmd使用步骤说明
终端窗口1运行以下命令  用于控制电机
ros2 launch inspire_actuator insactuator.launch.py 
ros2 run inspire_python szmd
ros2 run inspire_python szmd_fc 
终端窗口2运行以下命令  用于监听电机状态
ros2 run inspire_python echo
ros2 run inspire_python echo_copy
终端窗口3运行以下命令  用于发布13个光定位球的坐标
ros2 run aimooe_pub pub_trackers_8markers
ros2 run aimooe_pub pub_coords --ros-args -p base_frame:=SZMD-base
终端窗口4运行以下命令  用于过滤和记录数据
ros2 launch aimooe_pub szmd.launch.py
ros2 run aimooe_pub szmd_record_demo 

电机数据的拆分监听目前有一个bug 会显示/insactuator_msg有两种消息类型ArrayInt16和AimCoord 
我需要找出所有发布到/insactuator_msg的节点，并统一它们的消息类型（待解决）





