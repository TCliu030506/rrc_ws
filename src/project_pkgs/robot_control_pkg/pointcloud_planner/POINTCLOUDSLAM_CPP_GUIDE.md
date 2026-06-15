# pointcloud_planner 功能包导读

本文档用于快速理解 `pointcloud_planner` 的包结构、运行流程和主要实现方式。原 `README.md` 偏向安装和运行命令；本文档偏向代码入门和二次开发。

## 1. 功能定位

`pointcloud_planner` 是一个 ROS 2 + PCL 的点云处理与扫描路径生成包。当前核心节点 `pcl_cloudslam` 从深度相机订阅点云，将点云从相机坐标系转换到机器人基坐标系，在 PCL 可视化窗口中手动框选 ROI，然后在 ROI 内生成栅格扫描点，并为每个扫描点估计表面高度和姿态，最终输出机器人 TCP 扫描轨迹。

它目前更接近“点云辅助路径规划”而不是完整 SLAM 系统：代码中没有回环检测、地图优化或多帧位姿图优化，主要工作是单帧点云采集、坐标变换、局部表面拟合和路径导出。

## 2. 目录结构

```text
pointcloud_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── POINTCLOUDSLAM_CPP_GUIDE.md
├── include/pointcloud_planner/
│   └── pcl_cloudslam.h
├── src/
│   ├── pcl_cloudslam.cpp
│   ├── pcd_write.cpp
│   ├── pcl_cluster.cpp
│   ├── pcl_select.cpp
│   └── curve_polyfit.cpp
└── data/
    ├── hand_eye_calibration.json
    ├── pcl_original.pcd
    ├── pcl_filter.pcd
    ├── pcl_trans.pcd
    ├── pcl_roi.pcd
    ├── pcl_border.pcd
    └── path_planning/
        ├── Trans_6_b.txt
        └── path_map.txt
```

关键目录说明：

- `include/pointcloud_planner/pcl_cloudslam.h`：声明主节点类 `pcl_cloudslam`，包括点云订阅、机器人状态订阅、ROI、路径规划和手眼标定加载接口。
- `src/pcl_cloudslam.cpp`：主流程实现，是最重要的文件。
- `src/pcd_write.cpp`：简单点云订阅器，把相机点云保存成 `asd.pcd`。
- `src/pcl_cluster.cpp`：读取 PCD 文件并可视化，主要用于离线查看点云。
- `src/pcl_select.cpp`：读取 PCD 并通过 PCL 的 AreaPicking 框选点云，输出 `areapicked.pcd`。
- `src/curve_polyfit.cpp`：离线多项式曲面拟合实验代码。
- `data/`：保存标定文件、处理中间点云和路径输出。

## 3. 构建和依赖

主要依赖：

- ROS 2：`rclcpp`, `sensor_msgs`, `std_msgs`, `pcl_conversions`, `pcl_ros`
- 工作区内消息：`ur5_msg`
- 系统库：PCL, Eigen3, nlohmann-json

构建命令：

```bash
colcon build --packages-select pointcloud_planner --symlink-install
```

构建成功后加载环境：

```bash
source install/setup.bash
```

## 4. 可执行程序

`CMakeLists.txt` 当前安装 5 个可执行程序：

```text
pcd_write
pcl_cluster
pcl_select
curve_polyfit
pcl_cloudslam
```

### pcl_cloudslam

主程序。运行后会：

1. 等待并缓存相机点云；
2. 读取机器人当前末端位姿；
3. 对点云进行下采样；
4. 将点云从相机坐标系转换到机器人基坐标系；
5. 弹出 PCL 可视化窗口，人工框选 ROI；
6. 根据 ROI 生成扫描网格；
7. 对每个网格点做局部平面拟合；
8. 结合探头长度、安装姿态、表面法向和平滑参数输出 TCP 轨迹。

运行示例：

```bash
ros2 run pointcloud_planner pcl_cloudslam --ros-args \
  -p planning_mode:=plane \
  -p probe_length_m:=0.18 \
  -p probe_width_m:=0.075
```

### pcd_write

订阅 `/camera/camera/depth/color/points`，将收到的点云保存为当前工作目录下的 `asd.pcd`。适合先确认相机点云是否正常。

```bash
ros2 run pointcloud_planner pcd_write
```

### pcl_cluster

从终端输入 PCD 文件名或路径，读取并显示点云。它支持绝对路径、相对路径和默认目录文件名。没有 `DISPLAY` 时会跳过可视化，只检查文件能否读取。

```bash
ros2 run pointcloud_planner pcl_cluster
```

### pcl_select

读取 `data/` 下指定 PCD 文件，通过 PCL 可视化窗口进行区域框选，输出 `data/areapicked.pcd`。这是独立的 ROI 选择实验工具，主流程中 `pcl_cloudslam` 已经内置 ROI 框选。

### curve_polyfit

读取 PCD 点云并做高阶多项式曲面拟合，主要用于早期实验和验证拟合思路。主流程目前使用的是局部平面拟合，不依赖这个程序输出。

## 5. 主流程数据流

主程序入口在 `pcl_cloudslam.cpp` 的 `main()`：

```text
rclcpp::init()
创建 pcl_cloudslam 节点
后台线程 spin 节点
等待 3 秒接收点云和机器人状态
pcl_filter()
roi_range()
grid()
path_plan()
rclcpp::shutdown()
```

详细流程如下。

### 5.1 点云接收

节点订阅：

```text
/camera/camera/depth/color/points
```

回调 `pcl_callback()` 将 `sensor_msgs::msg::PointCloud2` 转成 `pcl::PointCloud<pcl::PointXYZ>` 并缓存到成员变量 `cloud`。`is_record` 为 `true` 时持续更新，开始处理后会置为 `false`，固定当前帧点云。

### 5.2 机器人状态接收

节点订阅：

```text
robotstate
```

消息类型为：

```text
ur5_msg::msg::RobotState
```

代码通过 `robot_state_callback()` 缓存最新机器人状态，并用互斥锁保护。后续 `pose_transform()` 会调用 `get_current_robot_pose()` 获取当前关节位置、速度和末端位姿。

如果没有收到机器人状态，代码会使用默认位姿：

```text
[0.5, 0.2, 0.3, 0.0, 0.0, 1.57]
```

这能避免程序崩溃，但生成路径会基于默认位姿，不适合真实执行。

### 5.3 点云下采样

`pcl_filter()` 会先保存原始点云：

```text
data/pcl_original.pcd
```

然后使用 `pcl::VoxelGrid` 按 `Leafsize` 做体素下采样，并输出：

```text
data/pcl_filter.pcd
```

### 5.4 坐标变换

`pose_transform()` 完成两级坐标变换：

```text
相机坐标系点云
  -> camera_to_end_effector
末端坐标系点云
  -> end_to_base
机器人基坐标系点云
```

其中：

- `camera_to_end_effector` 来自 `data/hand_eye_calibration.json` 中的 `T_cam2gripper`。
- `end_to_base` 由当前机器人末端位姿计算得到。

转换后的点云保存为：

```text
data/pcl_trans.pcd
```

当前实现还会把使用的末端位姿写入：

```text
data/path_planning/Trans_6_b.txt
```

### 5.5 ROI 框选

`pcl_filter()` 在转换后会打开 PCL 可视化窗口，并注册 `areapickingcallback()`。用户在窗口中框选区域后，选中点云会保存为：

```text
data/pcl_roi.pcd
```

`planning_mode` 有两种：

- `plane`：框选一次，直接用该 ROI 规划路径。
- `cylinder_preplan`：第一次 ROI 用来估计圆柱中轴和视角，再切换视角后进行第二次 ROI 框选，第二次 ROI 用于最终路径规划。

### 5.6 ROI 有效区域收缩

`roi_range()` 会读取 `pcl_roi.pcd`，计算 ROI 的 x/y 范围。为了避免探头边缘越界，它会根据探头尺寸收缩有效范围：

```text
x 方向收缩 = probe_width_m / 2 + probe_boundary_margin_m
y 方向收缩 = probe_length_m / 2 + probe_boundary_margin_m
```

如果 ROI 太小，无法容纳探头足迹，程序会打印错误并跳过路径生成。

函数还会在原始 ROI 外扩 `BORDER_DIS` 后提取附近点云，保存为：

```text
data/pcl_border.pcd
```

后续局部拟合使用的是这个边界点云。

### 5.7 栅格路径生成

`grid()` 根据收缩后的 ROI 范围和步长生成蛇形扫描网格：

```text
x 方向按 x_step 递增
y 方向按 y_step 往返扫描
```

这种蛇形路径减少了行间跳转距离，适合连续扫描。

### 5.8 局部平面拟合和姿态生成

`path_plan()` 对每个网格点执行：

1. 在 `pcl_border.pcd` 中搜索距离小于 `ArroundDistance` 的邻域点；
2. 邻域点少于 4 个则跳过；
3. 用 SVD 拟合局部平面：

```text
z = a*x + b*y + c
```

4. 从平面系数计算局部法向；
5. 平滑法向，减少姿态跳变；
6. 基于切平面构造扫描参考坐标系；
7. 叠加工具安装姿态 `tool_mount_*`；
8. 用探头偏移 `tool_tip_*` 和 `surface_clearance` 计算目标 TCP 位置；
9. 输出 6 维路径点：

```text
x y z rx ry rz
```

其中 `rx ry rz` 是 rotation vector，也就是轴角向量。

### 5.9 后处理

路径写入前会做三类后处理：

- `max_dz_step`：限制相邻路径点 z 方向突变；
- `enable_turn_z_smoothing`：在蛇形路径折返点平滑 z；
- rotation vector unwrap：避免姿态表示在 `+pi/-pi` 等价位置发生跳变。

最终路径输出：

```text
data/path_planning/path_map.txt
```

## 6. 关键参数

常用参数如下。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `Leafsize` | `0.005` | VoxelGrid 下采样体素尺寸，单位 m |
| `ArroundDistance` | `0.01` | 每个网格点局部拟合的邻域半径，单位 m |
| `x_step` | `0.0035` | 扫描网格 x 方向步长，单位 m |
| `y_step` | `0.064` | 扫描网格 y 方向步长，单位 m |
| `normal_smoothing_alpha` | `0.2` | 法向平滑系数 |
| `tool_mount_rx` | `pi` | 探头/TCP 固定安装姿态 roll |
| `tool_mount_ry` | `0.0` | 探头/TCP 固定安装姿态 pitch |
| `tool_mount_rz` | `0.0` | 探头/TCP 固定安装姿态 yaw |
| `tool_tip_x` | `0.0` | 探头尖端相对 TCP 的 x 偏移 |
| `tool_tip_y` | `0.0` | 探头尖端相对 TCP 的 y 偏移 |
| `tool_tip_z` | `0.135` | 探头尖端相对 TCP 的 z 偏移 |
| `surface_clearance` | `0.005` | TCP 目标相对表面的安全离面量 |
| `max_dz_step` | `0.003` | 相邻路径点最大 z 跳变量 |
| `enable_turn_z_smoothing` | `true` | 是否平滑蛇形路径折返点 z |
| `turn_dy_threshold` | `1e-4` | 判断 y 方向折返的阈值 |
| `probe_length_m` | `0.18` | 探头长度，沿 y_step 方向约束 |
| `probe_width_m` | `0.075` | 探头宽度，沿 x 方向约束 |
| `probe_boundary_margin_m` | `0.0` | ROI 收缩时额外安全边界 |
| `planning_mode` | `plane` | `plane` 或 `cylinder_preplan` |
| `cylinder_view_distance_m` | `0.35` | 圆柱预规划模式第二视角距离 |

示例：

```bash
ros2 run pointcloud_planner pcl_cloudslam --ros-args \
  -p planning_mode:=cylinder_preplan \
  -p Leafsize:=0.004 \
  -p x_step:=0.004 \
  -p y_step:=0.06 \
  -p ArroundDistance:=0.012 \
  -p probe_length_m:=0.18 \
  -p probe_width_m:=0.075 \
  -p surface_clearance:=0.005
```

## 7. 输入输出文件

### hand_eye_calibration.json

路径：

```text
data/hand_eye_calibration.json
```

主节点会读取其中的 `T_cam2gripper` 作为相机到末端执行器的 4x4 齐次变换矩阵。如果读取失败，代码会使用单位矩阵并打印 warning。真实使用时必须保证该文件正确，否则点云会被转换到错误位置。

### 中间点云文件

```text
data/pcl_original.pcd  原始点云
data/pcl_filter.pcd    下采样后点云
data/pcl_trans.pcd     转到机器人基坐标系后的点云
data/pcl_roi.pcd       用户框选 ROI 点云
data/pcl_border.pcd    ROI 附近扩展边界点云
```

这些文件便于离线调试：可以用 `pcl_cluster` 或外部 PCL 工具检查每一步点云是否合理。

### path_map.txt

路径：

```text
data/path_planning/path_map.txt
```

每行一个目标 TCP 位姿：

```text
x y z rx ry rz
```

前三项是机器人基坐标系下的位置，后三项是 rotation vector 姿态。下游控制节点读取前需要确认单位、坐标系和姿态表示一致。

## 8. 快速上手流程

1. 启动深度相机点云：

```bash
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p pointcloud.enable:=true
```

2. 确认机器人状态话题存在：

```bash
ros2 topic echo /robotstate
```

3. 构建功能包：

```bash
colcon build --packages-select pointcloud_planner --symlink-install
source install/setup.bash
```

4. 运行主节点：

```bash
ros2 run pointcloud_planner pcl_cloudslam --ros-args \
  -p planning_mode:=plane
```

5. 在 PCL 窗口中框选 ROI。

6. 检查输出路径：

```bash
head data/path_planning/path_map.txt
```

## 9. 常见问题

### 没有弹出 PCL 可视化窗口

检查是否有图形界面环境：

```bash
echo $DISPLAY
```

PCLVisualizer 需要 GUI 环境。如果在 SSH 或无桌面环境中运行，需要开启 X11 转发或改成离线处理流程。

### 生成路径为空

常见原因：

- 没有正确框选 ROI；
- ROI 太小，被探头尺寸收缩后无有效区域；
- `ArroundDistance` 太小，局部拟合点数不足；
- `x_step` 或 `y_step` 设置过大，网格点太少。

### 点云位置明显不对

优先检查：

- `data/hand_eye_calibration.json` 的 `T_cam2gripper` 是否正确；
- `robotstate` 中 `carte_pos` 是否是当前末端在基坐标系下的位姿；
- `calculate_transform_matrix()` 期望的姿态表示是否和 `RobotState.carte_pos[3..5]` 一致。

### 姿态跳变

当前实现已经做了法向平滑、四元数符号连续性和 rotation vector unwrap。如果仍有跳变，可以尝试：

- 增大 `normal_smoothing_alpha`；
- 增大 `ArroundDistance`；
- 降低点云噪声；
- 检查 ROI 是否包含边缘、空洞或非目标表面。

## 10. 二次开发入口

优先阅读顺序：

1. `include/pointcloud_planner/pcl_cloudslam.h`：先看成员变量和函数边界。
2. `src/pcl_cloudslam.cpp` 的构造函数：理解话题、参数和标定加载。
3. `pcl_filter()`：理解点云固定、下采样、坐标变换和 ROI 交互。
4. `roi_range()` 与 `grid()`：理解有效扫描区域和蛇形路径。
5. `path_plan()`：理解局部拟合、姿态生成、探头补偿和路径后处理。

开发时建议先用已有中间 PCD 文件离线验证，再连接真实相机和机器人状态。涉及真实机器人执行前，必须检查 `path_map.txt` 的坐标系、姿态表示、路径边界和安全离面量。
