# xmate_cr7_script_py

## 功能包描述
`xmate_cr7_script_py` 是一个 ROS 2 Python 功能包，提供与 XMate CR7 机器人服务器交互的客户端功能。通过调用 `coordinate/srv/StringScript` 服务，用户可以发送脚本指令与服务器通信来控制机器人执行各种操作。该功能包支持多种机器人操作，包括关节运动、笛卡尔运动、实时控制、状态读取和力传感器校准等。

## 节点功能描述

### 1. `cr7_script_client_node`
- **功能**：
  - 提供基础客户端功能，允许用户通过服务调用控制机器人。
  - 支持发送多种指令（如 `movej`、`movep`、`movel`、`stop` 等）。
- **服务依赖**：
  - `coordinate/srv/StringScript` 服务。
- **默认服务名**：`cr7_script`。

### 2. `cr7_script_demo_node`
- **功能**：
  - 演示cr7_script.py的使用功能，提供了所有功能的使用示例。
  - 默认执行以下操作：
    - `movej` 两次。
    - `movep` 两次。
    - `stop`。
    - `movel` 两次。
  - 可选启用的功能段落：
    - 跟随模式（`follow_pos_start` → 更新点位 → `follow_pos_stop`）。
    - 实时控制（关节模式和笛卡尔模式）。
    - 状态读取（如 `readp`、`readj`、`get_vel` 等）。
    - 运动学计算（正向和逆向运动学）。
- **服务依赖**：
  - `coordinate/srv/StringScript` 服务。
- **默认服务名**：`cr7_script`。

## 节点使用说明

### 1. 启动机械臂服务器
在工作区根目录下执行以下命令：
```bash
source install/setup.bash
ros2 launch xmate_cr7_script cr7_driver.launch.py force_poll_hz:=50 state_poll_hz:=50 
```

### 2. 运行 `cr7_script_client_node`
运行基础客户端节点，发送两条 `movej` 指令：
```bash
ros2 run xmate_cr7_script_py cr7_script_client_node --ros-args -p service_name:=cr7_script
```

### 3. 运行 `cr7_script_demo_node`
运行演示节点，默认执行 `movej`、`movep`、`stop` 和 `movel`：
```bash
ros2 run xmate_cr7_script_py cr7_script_demo_node --ros-args -p service_name:=cr7_script
```

启用所有可选段落（跟随模式、实时控制、状态读取、运动学计算）：
```bash
ros2 run xmate_cr7_script_py cr7_script_demo_node --ros-args \
  -p service_name:=cr7_script \
  -p enable_follow_demo:=true \
  -p enable_rt_joint_demo:=true \
  -p enable_rt_cart_demo:=true \
  -p enable_read_and_status_demo:=true \
  -p enable_kinematics_demo:=true
```

### 4. 参数说明
- `service_name`：指定服务名，默认为 `cr7_script`。
- `enable_follow_demo`：是否启用跟随模式演示，默认为 `false`。
- `enable_rt_joint_demo`：是否启用实时关节控制演示，默认为 `false`。
- `enable_rt_cart_demo`：是否启用实时笛卡尔控制演示，默认为 `false`。
- `enable_read_and_status_demo`：是否启用状态读取演示，默认为 `false`。
- `enable_kinematics_demo`：是否启用运动学计算演示，默认为 `false`。
