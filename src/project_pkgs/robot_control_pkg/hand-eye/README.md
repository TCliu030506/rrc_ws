# hand-eye

`hand-eye` 是一个 ROS 2 手眼标定功能包，用于采集 UR5 末端位姿和 ArUco 标记在相机坐标系下的位姿，并调用 OpenCV `calibrateHandEye` 计算相机到末端的手眼变换。

## 目录结构

- `launch/hand_eye.launch.py`：启动 RealSense、`ros2_aruco`、ArUco 位姿桥接、手眼标定节点和 `rqt_image_view`。
- `src/hand_eye.cpp`：手眼标定节点实现，负责订阅数据、记录样本和计算标定结果。
- `src/hand_eye_srv.cpp`：`handeye_calibration` 可执行入口。
- `include/hand-eye/hand_eye.h`：手眼标定节点类声明。
- `script/aruco_marker_pose_relay.py`：将 `ros2_aruco` 的目标 marker 位姿转发为原有的 `aruco_single/pose`。
- `script/hand_eye_client.py`：交互式客户端，通过服务发送记录和计算命令。
- `src/test_hand.cpp`：测试/调试用可执行文件。

## 主要节点

### `handeye_calibration`

由 `hand_eye.launch.py` 启动：

```bash
ros2 launch hand-eye hand_eye.launch.py
```

节点名为 `hand_eye_cali`，功能包括：

- 订阅机器人末端位姿。
- 订阅 ArUco 标记位姿。
- 接收记录/计算命令。
- 保存采样数据。
- 计算手眼变换矩阵。

### `hand_eye_client.py`

交互式服务客户端：

```bash
python3 src/project_pkgs/robot_control_pkg/hand-eye/script/hand_eye_client.py
```

输入：

- `1`：记录当前一组机器人末端位姿和 ArUco 位姿。
- `2`：使用已记录样本计算手眼标定结果。
- `0`：退出。

## 输入

### 订阅话题

- `robotstate` (`ur5_msg/msg/RobotState`)
  - 使用其中的 `carte_pos[0:6]` 作为 UR5 末端在基坐标系下的位姿。
  - 前 3 个值为位置，后 3 个值为旋转向量。

- `aruco_single/pose` (`geometry_msgs/msg/PoseStamped`)
  - ArUco 标记在相机坐标系下的位姿。
  - 由 `script/aruco_marker_pose_relay.py` 从 `ros2_aruco` 的 `aruco_markers` 转发得到。

### ArUco 配置

`launch/hand_eye.launch.py` 当前按以下标定板配置启动 `ros2_aruco`：

- `marker_id`: `0`
- `marker_size`: `0.04` m
- `aruco_dictionary_id`: `DICT_6X6_250`

如果标定板不是 `DICT_6X6_250` 生成的，启动时需要改成实际字典，例如：

```bash
ros2 launch hand-eye hand_eye.launch.py aruco_dictionary_id:=DICT_6X6_50
```

### 服务命令

- 服务名：`hand_eye_command`
- 服务类型：`coordinate/srv/StringScript`
- 支持命令：
  - `Rec`：记录当前样本。
  - `Cal`：计算手眼标定。

## 输出

节点启动时会创建两个文本文件：

- `pose_ur-<time>.txt`
  - 记录每次 `Rec` 时的 UR5 末端位姿。
  - `Cal` 后会追加输出 `Transform_handeye` 矩阵。

- `pose_aru-<time>.txt`
  - 记录每次 `Rec` 时的 ArUco 位姿。

当前代码中的保存路径由 `src/hand_eye.cpp` 的 `FILE_PATH` 决定：

```cpp
#define FILE_PATH "/Lab_WS/rrc_ws/src/project_pkgs/robot_control_pkg/hand-eye/data/"
```

最终路径会拼接为：

```text
$HOME/Lab_WS/rrc_ws/src/project_pkgs/robot_control_pkg/hand-eye/data/
```

## 标定流程

1. 启动相机、ArUco 检测和标定服务：

   ```bash
   ros2 launch hand-eye hand_eye.launch.py
   ```

2. 确认 `robotstate` 和 `aruco_single/pose` 都在正常发布。

3. 运行客户端：

   ```bash
   python3 src/project_pkgs/robot_control_pkg/hand-eye/script/hand_eye_client.py
   ```

4. 移动机械臂到多个不同姿态，每个姿态输入 `1` 记录一次。

5. 记录足够样本后输入 `2` 计算手眼矩阵。

6. 在终端输出和 `pose_ur-<time>.txt` 中查看 `Transform_handeye`。

## 注意事项

- 需要保证 ArUco 标记在记录时能被稳定检测到。
- 标定样本应覆盖多个不同位置和姿态，避免所有姿态过于接近。
- 记录样本前应确认机器人状态话题和 ArUco 位姿话题已经更新。
- 当前输出矩阵只打印和写入文本文件，尚未自动同步到 `pointcloud_planner/data/hand_eye_calibration.json`。
