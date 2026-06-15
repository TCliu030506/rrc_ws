# PointCloud SLAM C++ 项目导读与修改指南

这份文档面向第一次接手本仓库的人：先帮你知道这个包在做什么、怎么运行，再带你读懂主程序的数据流，最后说明常见调整应该改哪些文件和参数。

## 1. 项目一句话概览

`pointcloud_planner` 是一个 ROS2 + PCL 的点云路径规划包。它从 RealSense 深度相机订阅点云，结合 UR5 机械臂当前末端位姿和手眼标定矩阵，把点云转换到机械臂基坐标系；然后通过 PCL 可视化窗口手动框选 ROI，在 ROI 内生成扫描网格，对每个网格点做局部平面拟合，最终输出机械臂 TCP 可执行的路径点文件 `path_map.txt`。

当前主流程不是持续在线规划，而是一次性流程：

```text
启动节点
  -> 等待并缓存点云和机器人状态
  -> 冻结一帧点云
  -> 体素降采样
  -> 相机坐标系点云转换到基坐标系
  -> 手动框选 ROI
  -> 计算有效扫描范围
  -> 生成往复式网格路径
  -> 局部拟合表面与法线
  -> 转换成 TCP 位姿
  -> 写出 path_map.txt
```

## 2. 技术栈与依赖

主要技术：

- ROS2：节点、参数、订阅相机点云和机器人状态。
- PCL：点云读写、体素滤波、ROI 框选、可视化。
- Eigen：矩阵、坐标变换、SVD 拟合、四元数/旋转向量。
- nlohmann-json：CMake 中依赖了它，但当前 `load_camera_transform()` 实际采用手写字符串解析。
- 自定义消息包 `ur5_msg`：提供 `ur5_msg::msg::RobotState`，字段里至少需要有 `joint_pos`、`joint_vel`、`carte_pos`。

ROS2 依赖在 `package.xml` 和 `CMakeLists.txt` 中声明，核心包括：

- `rclcpp`
- `sensor_msgs`
- `pcl_conversions`
- `pcl_ros`
- `std_msgs`
- `ur5_msg`
- `PCL`
- `Eigen3`
- `nlohmann_json`

## 3. 目录结构

```text
.
├── CMakeLists.txt
├── package.xml
├── README.md
├── docs/
│   └── PROJECT_GUIDE.md
├── include/
│   └── pointcloud_planner/
│       └── pcl_cloudslam.h
├── src/
│   ├── pcl_cloudslam.cpp      # 主程序：点云处理、ROI、网格、路径规划
│   ├── pcd_write.cpp          # 辅助工具：订阅点云并保存 PCD
│   ├── pcl_select.cpp         # 辅助工具：打开 PCD 并手动框选点云
│   ├── pcl_cluster.cpp        # 辅助工具：加载/显示 PCD，当前没有真正聚类逻辑
│   └── curve_polyfit.cpp      # 辅助实验：高阶曲面拟合示例，不参与主流程
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

## 4. 构建与运行

把仓库放进 ROS2 工作空间的 `src` 目录后，在工作空间根目录执行：

```bash
colcon build --packages-select pointcloud_planner
source install/setup.bash
```

启动 RealSense 点云：

```bash
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p pointcloud.enable:=true
```

启动主规划节点，平面模式：

```bash
ros2 run pointcloud_planner pcl_cloudslam --ros-args \
  -p planning_mode:=plane \
  -p probe_length_m:=0.18 \
  -p probe_width_m:=0.075 \
  -p probe_boundary_margin_m:=0.0
```

启动主规划节点，圆柱预规划模式：

```bash
ros2 run pointcloud_planner pcl_cloudslam --ros-args \
  -p planning_mode:=cylinder_preplan \
  -p cylinder_view_distance_m:=0.35 \
  -p probe_length_m:=0.18 \
  -p probe_width_m:=0.075 \
  -p probe_boundary_margin_m:=0.0
```

运行前需要确认：

- 相机点云话题存在：`/camera/camera/depth/color/points`
- 机器人状态话题存在：`robotstate`
- `ur5_msg` 已在同一工作空间中构建。
- Linux 图形环境可用，因为主流程依赖 `PCLVisualizer` 手动框选 ROI。
- `data/hand_eye_calibration.json` 路径能被程序找到。

## 5. 重要硬编码路径

主程序在 `src/pcl_cloudslam.cpp` 顶部使用了宏：

```cpp
#define FILE_PATH "/zzrobot_ws/src/ur5lzh_pkg (1)/ur5lzh_pkg/pointcloud_planner/data/"
#define PLAN_PATH "/zzrobot_ws/src/ur5lzh_pkg (1)/ur5lzh_pkg/pointcloud_planner/data/path_planning/"
```

程序会把它们拼到 `$HOME` 后面，所以实际路径类似：

```text
$HOME/zzrobot_ws/src/ur5lzh_pkg (1)/ur5lzh_pkg/pointcloud_planner/data/
```

如果你把工程放在其他工作空间，最先检查这里。否则会出现 PCD、JSON、`path_map.txt` 找不到或写到旧目录的问题。

建议后续改造方向：把 `FILE_PATH` 和 `PLAN_PATH` 改成 ROS2 参数，或使用 `ament_index_cpp` 获取包共享目录，避免每次换机器都改源码。

注意：辅助工具里的路径宏不完全一致：

- `pcl_select.cpp` 使用 `/zzrobot_ws/src/ur5lzh_pkg/pointcloud_planner/data/`
- `pcl_cluster.cpp` 默认指向 `/zzrobot_ws/src/ur5lzh_pkg/pointcloud_process/data/back_model/`
- `pcd_write.cpp` 直接保存到当前工作目录下的 `asd.pcd`

所以辅助工具运行前也要单独看路径。

## 6. ROS 输入输出

### 输入话题

`pcl_cloudslam` 订阅两个话题：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/camera/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | RealSense 深度相机点云 |
| `robotstate` | `ur5_msg/msg/RobotState` | UR5 当前关节和末端位姿 |

相机点云在 `pcl_callback()` 中转换为 `pcl::PointCloud<pcl::PointXYZ>` 并缓存到成员变量 `cloud`。

机器人状态在 `robot_state_callback()` 中缓存到 `latest_robot_state_`，读写通过 `std::mutex` 保护。

### 文件输出

| 文件 | 产生阶段 | 含义 |
|---|---|---|
| `pcl_original.pcd` | `pcl_filter()` | 冻结下来的原始点云 |
| `pcl_filter.pcd` | `pcl_filter()` | 体素降采样后的点云 |
| `pcl_trans.pcd` | `pose_transform()` | 转换到机械臂基坐标系的点云 |
| `pcl_roi.pcd` | `areapickingcallback()` | 手动框选出的 ROI 点云 |
| `pcl_border.pcd` | `roi_range()` | ROI 外扩 `BORDER_DIS` 后的局部拟合点云 |
| `Trans_6_b.txt` | `pose_transform()` | 当前机械臂末端位姿备份 |
| `path_map.txt` | `path_plan()` | 最终输出路径 |

`path_map.txt` 每行 6 列：

```text
x y z rx ry rz
```

其中 `x y z` 是目标 TCP 在基坐标系下的位置，`rx ry rz` 是旋转向量，也就是 `axis * angle`，不是普通 RPY 欧拉角。

## 7. 主类 `pcl_cloudslam`

声明位于 `include/pointcloud_planner/pcl_cloudslam.h`，实现位于 `src/pcl_cloudslam.cpp`。

核心成员变量：

| 成员 | 含义 |
|---|---|
| `cloud` | 最新缓存的相机点云 |
| `pcl_sub` | 点云订阅器 |
| `robot_state_sub_` | 机器人状态订阅器 |
| `latest_robot_state_` | 最近一次机器人状态 |
| `camera_to_end_effector` | 手眼标定矩阵，相机坐标系到末端坐标系 |
| `probe_offset` | 探头尖端相对 TCP 的偏移 |
| `viewer` | PCL 可视化窗口，用于显示点云和框选 ROI |
| `roi_range_x` / `roi_range_y` | ROI 被探头尺寸内缩后的有效扫描范围 |
| `grid_xy` | 规划出来的二维网格点序列 |
| `points_arround` | 某个网格点附近用于局部拟合的点 |

## 8. 主程序执行顺序

`main()` 中的顺序很直接：

```cpp
rclcpp::init(argc, argv);
auto pcl_handle = std::make_shared<pcl_cloudslam>();
std::thread spin_thread([&pcl_handle](){ rclcpp::spin(pcl_handle); });
sleep(3);

pcl_handle->pcl_filter();
pcl_handle->roi_range();
pcl_handle->grid();
pcl_handle->path_plan();

rclcpp::shutdown();
```

含义：

1. 创建节点。
2. 单独开线程执行 ROS2 `spin()`，让点云和机器人状态订阅回调能持续接收数据。
3. 等待 3 秒，给相机和机器人状态留出接收时间。
4. 执行一次完整路径规划。
5. 关闭 ROS2。

如果相机启动慢、点云为空或机器人状态还没收到，可以把 `sleep(3)` 调大，或者改成“收到有效点云和机器人状态后再继续”的等待逻辑。

## 9. 核心算法流程

### 9.1 点云缓存：`pcl_callback()`

只要 `is_record == true`，回调就持续把 ROS2 点云消息转换成 PCL 点云并覆盖缓存：

```text
PointCloud2 -> pcl::PointCloud<pcl::PointXYZ> -> cloud
```

后面 `pcl_filter()` 一开始会把 `is_record` 设为 `false`，相当于冻结当前这一帧。

### 9.2 滤波和 ROI 入口：`pcl_filter()`

这个函数做了几件事：

1. 停止继续记录点云。
2. 保存 `pcl_original.pcd`。
3. 用 `VoxelGrid` 按 `Leafsize` 做体素降采样。
4. 保存 `pcl_filter.pcd`。
5. 调用 `pose_transform()` 转坐标系。
6. 加载 `pcl_trans.pcd` 并用 PCL 窗口显示。
7. 注册框选回调 `areapickingcallback()`。
8. 等待用户框选 ROI。
9. 如果 `planning_mode == "cylinder_preplan"`，会进行第二次视角调整和 ROI 框选。

平面模式只框选一次。圆柱预规划模式会先用第一次 ROI 粗估圆柱方向和观察视角，再让你在新视角下框选最终规划 ROI。

### 9.3 坐标变换：`pose_transform()`

坐标变换链路是：

```text
相机点 cloud_filter
  -> camera_to_end_effector
  -> end_to_base
  -> 基坐标系点 cloud_trans
```

`camera_to_end_effector` 来自 `hand_eye_calibration.json` 中的 `T_cam2gripper`。

`end_to_base` 来自机器人状态 `carte_pos`，通过 `calculate_transform_matrix()` 转成 4x4 齐次矩阵。

重要细节：点云变换阶段不叠加探头尖端偏移。探头偏移只在最终输出 TCP 位姿时补偿一次，位置在 `path_plan()` 中计算：

```cpp
tcp_position = surface_point + smoothed_normal * surface_clearance
             - tcp_rotation * probe_offset;
```

### 9.4 手动 ROI：`areapickingcallback()`

PCL 可视化窗口里框选到的点会被保存为：

```text
pcl_roi.pcd
```

`is_areapicked` 被置为 `true` 后，等待循环结束，主流程继续。

在圆柱预规划模式下，第二次框选也会写到同一个 `pcl_roi.pcd`，因此最终路径使用的是第二次 ROI。

### 9.5 ROI 有效范围：`roi_range()`

函数先读取 `pcl_roi.pcd` 的原始边界：

```text
min_x, max_x, min_y, max_y
```

然后根据探头尺寸把 ROI 向内缩：

```text
shrink_x = 0.5 * probe_width_m  + probe_boundary_margin_m
shrink_y = 0.5 * probe_length_m + probe_boundary_margin_m
```

这里约定：

- 探头短边/宽度沿 x 方向约束。
- 探头长边/长度沿 y_step 方向约束。

如果 ROI 太小，小到扣掉探头尺寸后没有可用区域，会报错并跳过后续网格生成。

最后它还会从 `pcl_trans.pcd` 中截取 ROI 附近点云，保存为 `pcl_border.pcd`，供局部拟合使用。外扩距离由源码宏 `BORDER_DIS` 控制，当前是 `0.02 m`。

### 9.6 往复式网格：`grid()`

网格生成使用两个参数：

- `x_step`：列间距。
- `y_step`：每列内部点间距。

生成策略是蛇形/往复式：

```text
x = x_min: y 从小到大
x = x_min + x_step: y 从大到小
x = x_min + 2*x_step: y 从小到大
...
```

这样可以减少路径断点和大幅空走。

### 9.7 局部平面拟合：`polyfit()`

对每个网格点，`path_plan()` 会在 `pcl_border.pcd` 中找二维距离小于 `ArroundDistance` 的邻近点，放入 `points_arround`。

如果邻近点少于 4 个，该网格点会被跳过。

`polyfit()` 用局部平面拟合：

```text
z = a*x + b*y + c
```

拟合得到：

```text
surface_point = (x, y, z)
normal = normalize(-a, -b, 1)
```

这里的 `polyfit()` 名字像多项式拟合，但主流程里实际是局部平面拟合。另一个文件 `curve_polyfit.cpp` 才是高阶多项式实验。

### 9.8 姿态生成和路径后处理：`path_plan()`

`path_plan()` 对每个有效网格点生成 TCP 位姿：

1. 取局部法线。
2. 和上一点法线做方向一致性处理，避免法线突然翻转。
3. 用 `normal_smoothing_alpha` 平滑法线。
4. 把上一点的切向投影到当前切平面，构造最小扭转姿态。
5. 得到扫描坐标系 `scan_rotation`。
6. 乘上探头安装补偿 `tool_mount_rotation`，得到 TCP 姿态。
7. 转为连续旋转向量输出。
8. 用 `surface_clearance` 和 `probe_offset` 把表面点转换为 TCP 点。
9. 对 Z 方向突变做 `max_dz_step` 限制。
10. 对蛇形拐弯点做可选 Z 平滑。
11. 对旋转向量做 unwrap，减少 `+pi/-pi` 等价跳变。

最终写入 `path_map.txt`。

## 10. 坐标系和姿态格式

你需要特别确认三个坐标关系：

```text
camera frame
  -- T_cam2gripper -->
end-effector / gripper frame
  -- robotstate.carte_pos -->
base frame
```

### 手眼标定文件

`data/hand_eye_calibration.json` 必须包含：

```json
{
  "T_cam2gripper": [
    [r11, r12, r13, tx],
    [r21, r22, r23, ty],
    [r31, r32, r33, tz],
    [0.0, 0.0, 0.0, 1.0]
  ]
}
```

当前加载函数只查找 `"T_cam2gripper"` 并读取其后的 4x4 数字矩阵；其它字段不会影响主流程。

### 机器人末端位姿格式

`calculate_transform_matrix()` 当前把 `pose[3..5]` 优先解释成旋转向量：

```text
rotation_vector = axis * angle
```

只有当旋转向量长度非常接近 0 时，才退回按 RPY 解释。

这点很关键：如果你的 `robotstate.carte_pos[3..5]` 实际发布的是 RPY 欧拉角，而不是旋转向量，当前坐标变换会不符合预期。需要根据 UR5 状态消息的真实定义修改 `calculate_transform_matrix()`。

### 输出姿态格式

`path_map.txt` 的姿态列也是旋转向量，不是 RPY。下游控制器读取时必须按旋转向量处理。

## 11. 参数速查表

这些参数都在 `pcl_cloudslam` 构造函数中声明，可以通过 `ros2 run ... --ros-args -p name:=value` 覆盖。

| 参数 | 默认值 | 作用 | 调大/调小的影响 |
|---|---:|---|---|
| `Leafsize` | `0.005` | 体素降采样尺寸，单位 m | 调大点更少更快，但细节少；调小点更多更慢 |
| `ArroundDistance` | `0.01` | 每个网格点附近参与拟合的半径，单位 m | 调大拟合更稳但细节被抹平；调小更贴局部但容易点数不足 |
| `x_step` | `0.0035` | 扫描网格 x 方向步长，单位 m | 调小路径更密，点更多 |
| `y_step` | `0.064` | 扫描网格 y 方向步长，单位 m | 通常和探头长度/覆盖宽度相关 |
| `normal_smoothing_alpha` | `0.2` | 法线平滑权重 | 越大越跟随当前点；越小越平滑 |
| `tool_mount_rx` | `pi` | 探头安装补偿 roll | 用于修正 TCP 与探头坐标系安装差异 |
| `tool_mount_ry` | `0.0` | 探头安装补偿 pitch | 同上 |
| `tool_mount_rz` | `0.0` | 探头安装补偿 yaw | 同上 |
| `tool_tip_x` | `0.0` | 探头尖端相对 TCP 的 x 偏移 | 影响最终 TCP 位置 |
| `tool_tip_y` | `0.0` | 探头尖端相对 TCP 的 y 偏移 | 影响最终 TCP 位置 |
| `tool_tip_z` | `0.135` | 探头尖端相对 TCP 的 z 偏移 | 影响最终 TCP 位置 |
| `surface_clearance` | `0.005` | 沿表面法线离开物体的安全距离，单位 m | 调大更保守，离表面更远 |
| `max_dz_step` | `0.003` | 相邻路径点最大 Z 跳变，单位 m | 调小 Z 更连续，但可能压平真实高度变化 |
| `enable_turn_z_smoothing` | `true` | 是否平滑蛇形拐弯处 Z | 关闭后路径更原始 |
| `turn_dy_threshold` | `1e-4` | 判断 y 方向反向拐弯的阈值 | 一般不用动 |
| `probe_length_m` | `0.18` | 探头长度，沿 y_step 方向约束 | 调大后 ROI 内缩更多 |
| `probe_width_m` | `0.075` | 探头宽度，沿 x 方向约束 | 调大后 ROI 内缩更多 |
| `probe_boundary_margin_m` | `0.0` | 探头额外安全边界 | 调大后更不靠近 ROI 边缘 |
| `planning_mode` | `plane` | 规划模式：`plane` 或 `cylinder_preplan` | 圆柱模式会二次框选 |
| `cylinder_view_distance_m` | `0.35` | 圆柱模式第二视角观察距离，单位 m | 调大视角更远 |

## 12. 两种规划模式

### 平面模式 `plane`

适合普通近似平面或视角已经合适的表面：

```text
框选一次 ROI -> 直接规划
```

### 圆柱预规划模式 `cylinder_preplan`

适合圆柱或弧面目标：

```text
第一次框选：大致选中圆柱/弧面区域
  -> PCA 粗估圆柱中轴、ROI 法向、平均半径
  -> 自动调整 PCL 观察相机
第二次框选：在新视角下选最终规划区域
  -> 继续执行网格和路径规划
```

这不是严格圆柱模型拟合，而是利用 ROI 点云协方差主方向做视角辅助。第一次 ROI 点数少于 8 或特征不明显时，会退回使用第一次 ROI。

## 13. 各源码文件职责

### `src/pcl_cloudslam.cpp`

主程序。你读懂这个文件就基本读懂了项目。

关键函数：

| 函数 | 作用 |
|---|---|
| `pcl_cloudslam::pcl_cloudslam()` | 创建订阅器、声明参数、加载手眼矩阵、初始化探头偏移 |
| `pcl_callback()` | 接收并缓存相机点云 |
| `robot_state_callback()` | 接收并缓存机器人状态 |
| `pcl_filter()` | 冻结点云、降采样、显示点云、触发 ROI 选择 |
| `pose_transform()` | 点云从相机坐标系转到基坐标系 |
| `areapickingcallback()` | 保存 PCL 窗口框选出的 ROI |
| `roi_range()` | 计算探头尺寸约束后的有效扫描范围 |
| `grid()` | 生成蛇形扫描网格 |
| `polyfit()` | 对局部点云做平面拟合并返回表面点和法线 |
| `path_plan()` | 生成 TCP 位姿路径并写出 `path_map.txt` |
| `load_camera_transform()` | 从 JSON 中加载 `T_cam2gripper` |
| `calculate_transform_matrix()` | 把末端位姿数组转换为 4x4 齐次矩阵 |

### `include/pointcloud_planner/pcl_cloudslam.h`

主类声明。新增成员变量或公开方法时，需要同步修改这里。

### `src/pcd_write.cpp`

简单点云录制器：

- 订阅 `/camera/camera/depth/color/points`
- 转成 `pcl::PointCloud<pcl::PointXYZRGB>`
- 保存为当前工作目录下的 `asd.pcd`

适合快速确认相机点云有没有数据。

### `src/pcl_select.cpp`

独立 ROI 框选工具：

- 从标准输入读文件名。
- 按硬编码路径加载 `文件名.pcd`。
- PCL 窗口框选点。
- 保存为 `areapicked.pcd`。

它和主流程的 `pcl_roi.pcd` 不是同一个输出文件名。

### `src/pcl_cluster.cpp`

当前名字叫 cluster，但代码实际只做 PCD 加载和可视化，没有欧式聚类或分割逻辑。它支持三种输入：

- 绝对路径。
- 相对路径。
- 纯文件名，拼接默认 `FILE_PATH`。

如果没有 `DISPLAY`，会跳过可视化并正常退出，适合在无桌面环境下测试 PCD 是否能加载。

### `src/curve_polyfit.cpp`

实验性工具，用 21 个系数拟合五阶二维曲面 `z=f(x,y)`，并提供 x/y 偏导计算函数。当前主流程没有调用它。

如果你后续觉得局部平面拟合不够，可以参考这个文件，把 `pcl_cloudslam.cpp` 中的 `polyfit()` 改成二次或更高阶曲面拟合。

## 14. 常见修改怎么下手

### 修改相机话题

改 `src/pcl_cloudslam.cpp` 构造函数里的：

```cpp
pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/camera/depth/color/points", 10, ...
);
```

也要同步检查 `pcd_write.cpp`。

更好的方式是把话题名声明为 ROS2 参数。

### 修改机器人状态话题或字段

话题名在构造函数：

```cpp
robot_state_sub_ = this->create_subscription<ur5_msg::msg::RobotState>(
    "robotstate", 10, ...
);
```

字段读取在 `get_current_robot_pose()`：

```cpp
joint_pos[i] = latest_robot_state_->joint_pos[i];
joint_vel[i] = latest_robot_state_->joint_vel[i];
carte_pos[i] = latest_robot_state_->carte_pos[i];
```

如果你的消息字段名不同，要改这里。

### 修改手眼标定

替换 `data/hand_eye_calibration.json` 中的 `T_cam2gripper`。

如果文件格式变了，改 `load_camera_transform()`。目前这个函数没有真正用 `json` 对象解析，格式容错较弱。

### 调整扫描密度

优先改运行参数：

```bash
-p x_step:=0.004 \
-p y_step:=0.06
```

路径点数量大致随 `1 / (x_step * y_step)` 增加。太密会让拟合和下游执行都变慢。

### 调整 ROI 边界安全距离

优先改：

```bash
-p probe_length_m:=0.18 \
-p probe_width_m:=0.075 \
-p probe_boundary_margin_m:=0.01
```

如果想改变 `pcl_border.pcd` 外扩范围，改源码宏：

```cpp
#define BORDER_DIS 0.02
```

### 调整拟合稳定性

优先改：

```bash
-p ArroundDistance:=0.012
```

现象和方向：

- 经常出现 `The number of fit points is inadequate.`：适当调大 `ArroundDistance`，或调大 `Leafsize` 前先确认点云密度。
- 路径贴合太粗：适当调小 `ArroundDistance`，但要保证每个网格点附近仍有足够点。

### 调整姿态连续性

优先改：

```bash
-p normal_smoothing_alpha:=0.2 \
-p max_dz_step:=0.003 \
-p enable_turn_z_smoothing:=true
```

如果姿态在路径中突然翻转，重点看：

- `path_plan()` 中法线方向一致性。
- `rotation_matrix_to_rotvec_continuous()` 中四元数符号连续性。
- 下游控制器是否正确按旋转向量解释 `rx ry rz`。

### 调整探头安装关系

如果实际 TCP 坐标系和探头尖端不一致，改：

```bash
-p tool_mount_rx:=3.141592653589793 \
-p tool_mount_ry:=0.0 \
-p tool_mount_rz:=0.0 \
-p tool_tip_x:=0.0 \
-p tool_tip_y:=0.0 \
-p tool_tip_z:=0.135
```

`tool_mount_*` 改姿态，`tool_tip_*` 改位置偏移。

## 15. 常见问题排查

### 没生成 `path_map.txt`

按顺序检查：

1. 路径宏是否指向真实 `data/path_planning/`。
2. `pcl_original.pcd` 是否有点。
3. `pcl_filter.pcd` 是否有点。
4. `pcl_trans.pcd` 是否生成。
5. 是否完成 ROI 框选。
6. ROI 是否过小，被探头尺寸内缩后无有效区域。
7. `grid_xy` 是否为空。
8. 是否大量网格点附近点数少于 4。

### 点云显示位置不对

重点检查：

- `hand_eye_calibration.json` 是否是相机到末端，即 `T_cam2gripper`。
- `robotstate.carte_pos` 是否是基坐标系下末端位姿。
- `calculate_transform_matrix()` 对姿态格式的解释是否匹配真实消息。

### ROI 框选后路径偏离表面

重点检查：

- `ArroundDistance` 是否太大或太小。
- `pcl_border.pcd` 是否覆盖了 ROI 周围足够点。
- 点云是否已经被正确转换到基坐标系。
- `surface_clearance` 和 `tool_tip_z` 是否符合真实探头。

### 运行时 PCL 窗口打不开

主流程依赖 PCL 可视化窗口，需要图形环境。远程 SSH 时要配置 X11 转发或在带桌面的机器上运行。无图形环境下 `pcl_cluster.cpp` 有跳过可视化逻辑，但主程序没有。

### 机器人状态没收到

如果 `get_current_robot_pose()` 返回 false，代码会使用默认位姿：

```cpp
double default_pose[6] = {0.5, 0.2, 0.3, 0.0, 0.0, 1.57};
```

这能避免程序崩溃，但会导致点云转换不可信。正式运行前必须确认 `robotstate` 正常发布。

## 16. 建议的阅读顺序

第一次读代码可以按这个顺序：

1. `README.md`：先看启动命令和模式。
2. `CMakeLists.txt`：看会编译出哪些可执行程序。
3. `include/pointcloud_planner/pcl_cloudslam.h`：看主类有哪些状态。
4. `src/pcl_cloudslam.cpp` 的构造函数：看订阅、参数、标定。
5. `main()`：看一次性执行流程。
6. `pcl_filter()`：看点云冻结、滤波、ROI 入口。
7. `pose_transform()`：看坐标系转换。
8. `roi_range()` 和 `grid()`：看扫描范围和路径点生成。
9. `polyfit()` 和 `path_plan()`：看表面拟合、姿态和最终输出。
10. 辅助工具文件：按需要再看。

## 17. 后续改造建议

如果你要把这个项目变得更稳、更容易移植，推荐按优先级做这些事：

1. 把 `FILE_PATH`、`PLAN_PATH`、相机话题、机器人状态话题改成 ROS2 参数。
2. 用 `nlohmann::json` 正式解析 `hand_eye_calibration.json`，替换手写字符串解析。
3. 在继续规划前检查点云是否为空、机器人状态是否已收到、文件是否打开成功。
4. 为 `calculate_transform_matrix()` 明确姿态输入格式：旋转向量或 RPY 二选一，不要隐式猜。
5. 给主流程增加 launch 文件和参数 YAML，避免启动命令过长。
6. 把 PCD 中间文件写入可配置输出目录，或者允许只在调试模式下写文件。
7. 如果目标表面弯曲明显，把局部平面拟合升级为二次曲面拟合，并保留点数不足时的退化策略。

## 18. 快速修改清单

想调路径密度：改 `x_step`、`y_step`。

想调贴合程度：改 `ArroundDistance`、`Leafsize`。

想让路径离表面更远：改 `surface_clearance`。

想防止探头碰边：改 `probe_length_m`、`probe_width_m`、`probe_boundary_margin_m`。

想换相机：改点云话题和 `hand_eye_calibration.json`。

想换机械臂状态来源：改 `robotstate` 话题、`RobotState` 字段读取和姿态格式解析。

想换数据目录：先改 `FILE_PATH` 和 `PLAN_PATH`，再检查辅助工具各自的路径宏。
