# PointCloud SLAM C++

This package is a ROS2 implementation of a PointCloud SLAM system.

## Dependencies

This package depends on the following ROS2 packages and system libraries:

*   `rclcpp`
*   `pcl_conversions`
*   `sensor_msgs`
*   `ur5_msg`
*   `pcl_ros`
*   `std_msgs`
*   `libpcl-all-dev`
*   `libeigen3-dev`
*   `nlohmann-json-dev`

### Installing Dependencies

On a Debian-based system (like Ubuntu), you can install the system dependencies with the following command. *Note: The ROS2 distribution is assumed to be `foxy`. Please replace `foxy` with your ROS2 distribution if it's different (e.g., `galactic`, `humble`). The `ur5_msg` package is a custom message package and should be in your workspace.*

```bash
sudo apt-get update && sudo apt-get install -y \
    ros-foxy-rclcpp \
    ros-foxy-pcl-conversions \
    ros-foxy-sensor-msgs \
    ros-foxy-pcl-ros \
    ros-foxy-std-msgs \
    libpcl-all-dev \
    libeigen3-dev \
    nlohmann-json3-dev
```

## Building the Package

1.  Clone this package into your ROS2 workspace's `src` directory.
2.  Navigate to the root of your workspace.
3.  Build the package using `colcon`:

    ```bash
    colcon build --packages-select pointcloudslam_cpp --symlink-install
    ```

## 启动相机节点
```bash
ros2 run realsense2_camera realsense2_camera_node \
    --ros-args \
    -p pointcloud.enable:=true
```

ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p pointcloud.enable:=true \
  -p enable_gyro:=false \
  -p enable_accel:=false

## 启动点云规划节点

### 模式 1：平面模式（原流程）
```bash
source /home/lzh/zzrobot_ws/install/setup.bash
ros2 run pointcloudslam_cpp pcl_cloudslam --ros-args \
    -p planning_mode:=plane \
    -p probe_length_m:=0.18 \
    -p probe_width_m:=0.075 \
    -p probe_boundary_margin_m:=0.0
```

### 模式 2：圆柱预规划模式（新流程）
```bash
source /home/lzh/zzrobot_ws/install/setup.bash
ros2 run pointcloudslam_cpp pcl_cloudslam --ros-args \
    -p planning_mode:=cylinder_preplan \
    -p cylinder_view_distance_m:=0.35 \
    -p probe_length_m:=0.18 \
    -p probe_width_m:=0.075 \
    -p probe_boundary_margin_m:=0.0
```

圆柱预规划模式执行顺序：
1. 完成坐标系转换后，先进行第一次手动框选（预规划区域）。
2. 节点基于第一次 ROI 点云近似拟合圆柱中轴并自动旋转视角（视线平行中轴）。
3. 在新视角下进行第二次手动框选（最终路径规划区域）。
4. 节点继续执行网格生成和轨迹规划，输出 `path_map.txt`。

## 关键参数
- `planning_mode`：`plane` 或 `cylinder_preplan`
- `cylinder_view_distance_m`：圆柱模式下旋转视角观察距离（m）
- `x_step`：扫描网格在 x 方向步长（m）
- `y_step`：扫描网格在 y 方向步长（m），探头长边按该方向约束
- `ArroundDistance`：局部拟合采样半径（m）
- `probe_length_m`：探头长度（m）
- `probe_width_m`：探头宽度（m）
- `probe_boundary_margin_m`：探头边界额外安全余量（m）

## 输出文件
- 路径文件：`data/path_planning/path_map.txt`