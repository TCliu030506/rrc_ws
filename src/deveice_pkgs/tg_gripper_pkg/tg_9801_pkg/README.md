# tg_9801_pkg

TG-9801 夹爪 ROS2 功能包（`ament_python`）。

## 1. 构建

```bash
cd ~/Lab_WS/zzrobot_ws
colcon build --symlink-install --packages-select tg_9801_pkg
source install/setup.bash
```

### 1.1 查看当前连接设备（命令行）

查看常见串口设备：

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

ls -l /dev/serial/by-id/
```

查看 USB 设备列表：

```bash
lsusb
```

实时查看串口设备插拔变化（推荐联调时使用）：

```bash
watch -n 1 'ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null'
```

### 1.2 固定设备号（udev 规则，推荐）

为避免设备重插后 `ttyACM0/ttyACM1` 变化，建议配置固定别名（如 `/dev/tg_gripper`）。

1) 创建规则文件：

```bash
sudo tee /etc/udev/rules.d/99-zzrobot-serial.rules >/dev/null <<'EOF'
# TG-9801 夹爪 -> /dev/tg_gripper
SUBSYSTEM=="tty", ENV{ID_SERIAL}=="HKV_TG-9801_HKV_TG9801_00016", SYMLINK+="tg_gripper", GROUP="dialout", MODE="0660"

# 3D Systems 设备 -> /dev/omni_haptic
SUBSYSTEM=="tty", ENV{ID_SERIAL}=="3D_Systems_3DSystems_Virtual_Serial_Port_0000000071B9", SYMLINK+="omni_haptic", GROUP="dialout", MODE="0660"
EOF
```

2) 重载规则并触发：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

3) 验证别名：

```bash
ls -l /dev/tg_gripper /dev/omni_haptic
```

4) 在启动命令中使用固定别名：

```bash
ros2 launch tg_9801_pkg tg_9801.launch.py port:=/dev/tg_gripper
```

---

## 2. 功能包主要功能

本功能包重点提供三类核心能力：

- 夹爪服务器节点：`tg_9801_node`
- 状态发布节点：`tg_state_broadcaster`
- 可直接调用的 ROS 客户端模块：`TG9801RosClient`

### 2.1 夹爪服务器节点（`tg_9801_node`）

节点：`tg_9801_node`
启动方式：

```bash
ros2 launch tg_9801_pkg tg_9801.launch.py
```

说明：该 launch 默认会同时启动 `tg_state_broadcaster`，即启动服务器后即可直接订阅状态话题。
如仅需服务器节点，可显式关闭：

```bash
ros2 launch tg_9801_pkg tg_9801.launch.py enable_state_broadcaster:=false
```

#### 提供的 ROS2 接口

- 服务：`/set_grip`（`std_srvs/srv/SetBool`）
  - `true`：夹取
  - `false`：松开
- 话题订阅：`/raw_tx`（`std_msgs/msg/String`）
  - 发送原始字节串到夹爪
- 话题发布：`/rx_hex`（`std_msgs/msg/String`）
  - 发布串口接收数据（HEX）
- 话题订阅：`/command_json`（`std_msgs/msg/String`）
  - 发送 JSON 命令（统一动作入口）
- 话题发布：`/response_json`（`std_msgs/msg/String`）
  - 返回 JSON 执行结果

#### 已封装的主要交互功能

- 控制开合：`grip` / `release`
- 读取状态：`get_status`
- 读取受力：`get_force`
- 设置受力：`set_force`
- 读取设备信息：`get_id` / `get_serial`
- 其他协议动作：`set_id`、`set_protocol`、`set_position_mode`、`position`、`calibrate`、`hardware_zero`、`record_zero`、`read_holding`、`read_input`、`write_single`、`write_multi`、`set_speed`、`set_command`、`get_follow_status`、`get_finger`、`set_baudrate`

#### 使用示例

1) 夹取/松开

```bash
ros2 service call /set_grip std_srvs/srv/SetBool "{data: true}"
ros2 service call /set_grip std_srvs/srv/SetBool "{data: false}"
```

2) 读取夹爪状态（JSON 命令）

```bash
ros2 topic pub /command_json std_msgs/msg/String "{data: '{\"action\":\"get_status\",\"protocol\":\"modbus_rtu\",\"device_addr\":1}'}" -1
ros2 topic echo /response_json
```

3) 读取夹爪受力（JSON 命令）

```bash
ros2 topic pub /command_json std_msgs/msg/String "{data: '{\"action\":\"get_force\",\"protocol\":\"modbus_rtu\",\"device_addr\":1}'}" -1
```

4) 设置夹爪受力（JSON 命令）

```bash
ros2 topic pub /command_json std_msgs/msg/String "{data: '{\"action\":\"set_force\",\"protocol\":\"modbus_rtu\",\"device_addr\":1,\"target_force\":50}'}" -1
```

#### 常用参数（launch）

- `port`（默认 `/dev/ttyACM0`）
- `baudrate`（默认 `1000000`）
- `protocol`（`rs485_custom` / `modbus_rtu`）
- `device_address`
- `modbus_start_addr`（默认 `0x000A`）
- `rs485_release_cmd`（默认 `1`）
- `serial_parity` / `serial_stopbits` / `serial_bytesize`

---

### 2.2 状态发布节点（`tg_state_broadcaster`）

用途：定时调用 `tg_9801_node` 的 `command_json/response_json`，读取夹爪状态并拆分发布，供上层直接订阅。
通常无需单独启动（`tg_9801.launch.py` 已默认启动本节点）。

单独运行方式：

```bash
ros2 run tg_9801_pkg tg_state_broadcaster
```

默认发布话题（前缀 `tg_state/`）：

- `static_info`（`tg_9801_interfaces/msg/GripperStaticInfo`）
- `motion_state`（`tg_9801_interfaces/msg/GripperMotionState`）
- `finger_state`（`tg_9801_interfaces/msg/GripperFingerState`）
- `last_error`（`diagnostic_msgs/msg/DiagnosticStatus`）

以上话题均为强类型消息，字段可直接订阅使用，无需 JSON 解析。

发布频率（默认）：

- `finger_state`：50 Hz
- `motion_state`：10 Hz
- `static_info`：1 Hz

可通过参数调整：`finger_publish_hz`、`motion_publish_hz`、`static_publish_hz`。

常用参数：

- `poll_period`：状态刷新周期（秒）
- `protocol`：`modbus_rtu` / `rs485_custom`
- `device_addr`：夹爪地址
- `state_prefix`：发布话题前缀（默认 `tg_state`）
- `read_speed_register` / `read_follow_status` / `read_finger_data`：可选读取项

### 2.3 可直接调用的 ROS 客户端（`TG9801RosClient`）

用途：给团队成员提供类似 SDK 的 Python 调用方式，通过 ROS2 直接与 `tg_9801_node` 通信（服务 + 话题）。

模块位置：`tg_9801_pkg/tg_9801_ros_client.py`。

典型能力：

- 服务调用：`grip()` / `release()`
- 命令调用：`get_status()` / `get_force()` / `set_force()` / `get_id()` / `get_serial()`
- 其他调用：`move_position()` / `read_holding()` / `write_single()` / `send_raw()`

简单示例：

```python
import rclpy
from tg_9801_pkg import TG9801RosClient

rclpy.init()
node = TG9801RosClient()
ok, res = node.get_status(protocol='modbus_rtu', device_addr=1, timeout_sec=2.0)
print(ok, res)
node.destroy_node()
rclpy.shutdown()
```

可执行 demo：

```bash
ros2 run tg_9801_pkg tg_9801_ros_demo
```

---

## 3. 可执行节点（测试/调试工具）

### 3.1 `tg_9801_test_client`（ROS2 接口自动测试）

用途：验证服务器节点的服务、话题和命令动作是否可用。
运行：

```bash
ros2 run tg_9801_pkg tg_9801_test_client
```

全量动作测试：

```bash
ros2 run tg_9801_pkg tg_9801_test_client --run-all-actions --include-destructive
```

将已知不稳定动作降级为非关键项：

```bash
ros2 run tg_9801_pkg tg_9801_test_client --run-all-actions --include-destructive --non-critical-actions record_zero,read_input
```

### 3.2 `tg_9801_quick_test`（直连串口快速验证）

用途：不依赖 ROS2 话题/服务，直接按动作测协议通信。
示例：

```bash
ros2 run tg_9801_pkg tg_9801_quick_test --port /dev/ttyACM0 --protocol modbus_rtu --device-addr 1 --action get_status
```

### 3.3 `tg_9801_cli`（交互式串口工具）

用途：交互式选择协议/动作并发送，适合人工调试。
运行：

```bash
ros2 run tg_9801_pkg tg_9801_cli
```

### 3.4 `tg_9801_teleoperation_node`（Omni 按钮遥操作）

用途：仿照 `dg_omni_control_ui.py`，订阅 `OmniButtonEvent` 并通过 `set_grip` 服务遥操作夹爪。

- 灰键按下：夹取
- 白键按下：松开

运行：

```bash
ros2 run tg_9801_pkg tg_9801_teleoperation_node
```

常用参数：

- `button_topic`（默认 `/phantom/button`）
- `service_name`（默认 `set_grip`）
- `grey_active_low` / `white_active_low`（默认 `false`，即 1 表示按下）
- `debounce_sec`（默认 `0.15`）

---

## 4. 当前结论（基于已执行测试）

你当前最关心的三类交互在 `tg_9801_node` 上已验证可用：

- 读取夹爪状态：`get_status` 通过
- 控制夹爪开合：`set_grip`、`grip/release` 通过
- 读取夹爪受力：`get_force` 通过

说明：个别动作（如 `record_zero`、`read_input`）在部分设备上可能因寄存器/固件差异出现失败，这不影响上述核心交互功能可用性判断。
