# 工件局部坐标系路径规划调整方案

本文档用于指导后续代码调整：在不影响现有 `plane` 和 `cylinder_preplan` 运行逻辑的前提下，为项目新增一种“基于工件局部坐标系”的路径规划方式。目标是解决工件边界相对机械臂基坐标系旋转较大时，当前基于 `base X/Y` 的蛇形路径难以贴合工件边界的问题。

## 1. 背景问题

当前主程序位于 `src/pcl_cloudslam.cpp`，整体路径规划流程是：

```text
采集点云
  -> 点云降采样
  -> 通过手眼矩阵和机械臂末端位姿转到机械臂基坐标系
  -> 手动框选 ROI
  -> 读取 ROI 的 base x/y 最小最大值
  -> 在 base x/y 外接矩形中生成蛇形网格
  -> 对每个网格点做局部拟合
  -> 输出 base 坐标系下的 TCP 位姿 path_map.txt
```

现有关键函数：

| 函数 | 当前职责 |
|---|---|
| `pcl_filter()` | 冻结点云、降采样、坐标变换、触发 ROI 框选 |
| `roi_range()` | 从 `pcl_roi.pcd` 统计 `base x/y` 范围，并按探头尺寸内缩 |
| `grid()` | 在 `base x/y` 范围中生成蛇形路径 |
| `polyfit()` | 用 `base x/y/z` 做局部平面拟合 |
| `path_plan()` | 基于 `grid_xy` 生成 TCP 位姿并写出 `path_map.txt` |

当前方案隐含假设：

```text
工件边界基本和机械臂基坐标系 X/Y 对齐
```

如果方形或矩形工件相对机械臂基坐标系明显旋转，当前逻辑会使用工件在 `base X/Y` 平面上的轴对齐外接矩形进行规划。这会导致：

- ROI 外接矩形比真实工件区域大，角落可能落到工件外。
- 蛇形路径方向沿机械臂基坐标系，而不是沿工件长边/短边。
- 探头边界内缩按 `base X/Y` 进行，不等价于按工件边界内缩。
- 网格点附近可能没有足够点云，导致路径点被跳过，路径不连续。
- 即使输出能执行，路径覆盖方向也不符合工艺直觉。

## 2. 调整目标

新增一种规划模式，让路径在工件自身坐标系中生成，再转换回机械臂基坐标系输出。

推荐新增模式名：

```text
planning_mode:=oriented_plane
```

新增后应保持：

- `planning_mode:=plane` 行为不变。
- `planning_mode:=cylinder_preplan` 行为不变。
- 默认参数仍为 `planning_mode:=plane`。
- 最终输出文件仍为 `data/path_planning/path_map.txt`。
- `path_map.txt` 每行仍为 `x y z rx ry rz`，并且仍表示机械臂基坐标系下 TCP 位姿。
- 原有 `roi_range()`、`grid()`、`path_plan()` 可继续服务旧模式，避免破坏既有流程。

## 3. 核心原理

### 3.1 坐标系分离

新方案将“执行坐标系”和“规划坐标系”分开：

```text
执行坐标系：机械臂基坐标系 base
规划坐标系：工件局部坐标系 workpiece
```

点云仍然先通过现有 `pose_transform()` 转换到机械臂基坐标系，因为最终机械臂执行需要 base 坐标系下的目标位姿。

但路径网格不再直接使用 `base x/y`，而是使用工件局部坐标：

```text
U：工件表面内的主方向，通常沿工件长边
V：工件表面内的副方向，通常沿工件短边
N：工件表面法线方向
O：工件局部坐标系原点，通常取 ROI 点云中心
```

要求：

```text
U、V、N 两两正交
V = N x U
U = V x N
```

路径规划在 `U/V` 平面进行：

```text
ROI 点云 base 坐标
  -> 投影到 workpiece U/V/N
  -> 在 U/V 中计算边界与内缩
  -> 在 U/V 中生成蛇形路径
  -> 每个路径点转换回 base 坐标
  -> 输出 base 坐标系 TCP 位姿
```

### 3.2 坐标变换公式

给定 base 坐标系中的点：

```text
P_base = [x, y, z]
```

工件局部坐标系定义为：

```text
O_base：原点
U_base：U 轴单位向量
V_base：V 轴单位向量
N_base：N 轴单位向量
```

base 到 workpiece：

```text
d = P_base - O_base

u = dot(d, U_base)
v = dot(d, V_base)
n = dot(d, N_base)

P_workpiece = [u, v, n]
```

workpiece 到 base：

```text
P_base = O_base + u * U_base + v * V_base + n * N_base
```

### 3.3 局部路径生成

旧逻辑：

```text
在 base x/y 轴对齐矩形中生成蛇形路径
```

新逻辑：

```text
在 workpiece u/v 轴对齐矩形中生成蛇形路径
```

这意味着即使工件在 base 坐标系中是斜放的，只要局部坐标系估计正确，在工件坐标系中它仍然是“摆正”的矩形。

## 4. 推荐总体方案

建议采用“新增模式 + 新增函数 + 复用原输出”的方式，而不是直接改造旧函数。

### 4.1 新增模式

新增 ROS2 参数值：

```text
planning_mode:=oriented_plane
```

模式含义：

```text
从 ROI 点云估计工件局部坐标系，并在工件局部 U/V 平面内生成蛇形路径。
```

旧模式保持不变：

| 模式 | 行为 |
|---|---|
| `plane` | 现有 base X/Y 蛇形路径 |
| `cylinder_preplan` | 现有二次 ROI 框选 + base X/Y 蛇形路径 |
| `oriented_plane` | 新增工件局部 U/V 蛇形路径 |

### 4.2 工件坐标系估计方式

第一版建议支持 PCA 自动估计：

```text
workpiece_frame_method:=pca
```

从 `pcl_roi.pcd` 中估计：

1. 计算 ROI 点云中心 `O`。
2. 计算 ROI 点云协方差矩阵。
3. 对协方差矩阵做特征分解。
4. 最大特征值方向作为 `U`，即工件表面主方向。
5. 最小特征值方向作为 `N`，即工件表面法线。
6. `V = N x U`。
7. 重新正交化：`U = V x N`。

建议保留后续扩展：

```text
workpiece_frame_method:=manual_points
workpiece_frame_method:=manual_yaw
```

其中：

- `manual_points`：用户在 PCL 窗口中点两个点，连线方向作为 `U`。
- `manual_yaw`：用户通过参数指定 base 坐标系中的平面内方向。

第一版可以只实现 `pca`，但接口和参数命名应给后续手动模式留空间。

### 4.3 ROI 边界计算

旧逻辑中 `roi_range()` 统计：

```text
min_x, max_x, min_y, max_y
```

新逻辑应统计：

```text
min_u, max_u, min_v, max_v
```

流程：

```text
读取 pcl_roi.pcd
  -> 将每个 ROI 点从 base 转到 workpiece
  -> 统计 u/v 范围
  -> 根据探头 footprint 内缩边界
```

如果约定探头短边沿 `U`，长边沿 `V`，则：

```text
shrink_u = 0.5 * probe_width_m  + probe_boundary_margin_m
shrink_v = 0.5 * probe_length_m + probe_boundary_margin_m
```

得到有效规划范围：

```text
u_min = raw_u_min + shrink_u
u_max = raw_u_max - shrink_u
v_min = raw_v_min + shrink_v
v_max = raw_v_max - shrink_v
```

若内缩后无有效区域，应报错并跳过新模式路径生成，不应回写错误路径。

### 4.4 网格生成

旧逻辑中 `grid()` 生成：

```text
std::vector<Eigen::Vector2d> grid_xy;
```

新逻辑建议新增：

```text
std::vector<Eigen::Vector2d> grid_uv;
```

生成方式仍为蛇形路径：

```text
u = u_min: v 从小到大
u = u_min + u_step: v 从大到小
u = u_min + 2*u_step: v 从小到大
...
```

为了兼容旧参数，第一版可复用：

```text
x_step -> u_step
y_step -> v_step
```

但文档和日志中应明确：

```text
在 oriented_plane 模式下，x_step/y_step 表示工件局部 U/V 方向步长。
```

后续可新增更清晰的参数：

```text
u_step
v_step
```

### 4.5 局部表面拟合

旧 `polyfit()` 使用 base 坐标拟合：

```text
z = a*x + b*y + c
```

新模式建议在 workpiece 坐标系中拟合：

```text
n = a*u + b*v + c
```

原因：

- 对斜放工件而言，base `z` 不一定是最合理的高度方向。
- 工件局部法线 `N` 才是表面高度方向。
- 使用 `u/v/n` 拟合可以让路径贴合工件自身表面。

新拟合输出：

```text
surface_local = [u, v, fitted_n]
normal_local = normalize([-a, -b, 1])
```

转换回 base：

```text
surface_base = O + u * U + v * V + fitted_n * N

normal_base = normal_local.x * U
            + normal_local.y * V
            + normal_local.z * N
```

之后可复用现有 `path_plan()` 中关于姿态连续化、探头安装补偿、`surface_clearance`、`probe_offset`、旋转向量连续化等逻辑。

### 4.6 邻域搜索距离

旧逻辑用 base `x/y` 平面距离找附近点：

```text
sqrt((point.x - grid_x)^2 + (point.y - grid_y)^2) < ArroundDistance
```

新逻辑应使用 workpiece `u/v` 平面距离：

```text
sqrt((point.u - grid_u)^2 + (point.v - grid_v)^2) < ArroundDistance
```

这样邻域采样与新规划坐标系保持一致。

### 4.7 输出格式

新模式最终输出仍为：

```text
x y z rx ry rz
```

含义仍为：

```text
x/y/z：机械臂基坐标系下 TCP 位置
rx/ry/rz：机械臂基坐标系下 TCP 姿态旋转向量
```

不能把 `u/v/n` 直接写入 `path_map.txt`。局部坐标只用于中间规划，不改变下游执行接口。

## 5. 建议新增数据结构

建议在 `include/pointcloudslam_cpp/pcl_cloudslam.h` 中新增结构体：

```cpp
struct WorkpieceFrame
{
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d u_axis = Eigen::Vector3d::UnitX();
    Eigen::Vector3d v_axis = Eigen::Vector3d::UnitY();
    Eigen::Vector3d n_axis = Eigen::Vector3d::UnitZ();
    bool valid = false;
};
```

在 `pcl_cloudslam` 类中新增成员：

```cpp
WorkpieceFrame workpiece_frame_;
Eigen::Vector2d roi_range_u_;
Eigen::Vector2d roi_range_v_;
std::vector<Eigen::Vector2d> grid_uv_;
```

如需减少全局状态，也可以把这些作为新函数的输入输出，但考虑当前类已有 `roi_range_x/y`、`grid_xy` 成员，第一版按现有风格新增成员更容易落地。

## 6. 建议新增参数

在构造函数中新增参数：

```cpp
declare_parameter<std::string>("workpiece_frame_method", "pca");
declare_parameter<bool>("oriented_plane_fallback_to_plane", false);
declare_parameter<bool>("save_workpiece_debug_cloud", true);
```

可选新增参数：

```cpp
declare_parameter<double>("workpiece_min_extent_m", 0.02);
declare_parameter<double>("workpiece_min_normal_eigen_ratio", 1e-3);
declare_parameter<std::string>("oriented_scan_long_axis", "v");
```

参数说明：

| 参数 | 建议默认值 | 说明 |
|---|---:|---|
| `planning_mode` | `plane` | 新增支持 `oriented_plane` |
| `workpiece_frame_method` | `pca` | 工件坐标系估计方式 |
| `oriented_plane_fallback_to_plane` | `false` | 新模式估计失败时是否退回旧 `plane` |
| `save_workpiece_debug_cloud` | `true` | 是否保存调试点云/路径 |
| `workpiece_min_extent_m` | `0.02` | ROI 在 U/V 方向上的最小尺寸 |
| `oriented_scan_long_axis` | `v` | 探头长边或扫查长步方向约定 |

建议第一版不要自动退回 `plane`，而是失败时报错并停止新模式路径生成。这样可以避免用户以为生成的是局部坐标系路径，实际却悄悄变成旧路径。

## 7. 建议新增函数

建议在 `pcl_cloudslam.h` 中新增声明：

```cpp
bool estimate_workpiece_frame_from_roi();
Eigen::Vector3d base_to_workpiece(const Eigen::Vector3d& point_base) const;
Eigen::Vector3d workpiece_to_base(const Eigen::Vector3d& point_local) const;
void roi_range_workpiece();
void grid_workpiece();
bool polyfit_workpiece(const Eigen::Vector2d& point_uv, double (&pose_out)[6]);
void path_plan_workpiece();
```

函数职责：

| 函数 | 职责 |
|---|---|
| `estimate_workpiece_frame_from_roi()` | 从 `pcl_roi.pcd` 估计 `O/U/V/N` |
| `base_to_workpiece()` | base 点转换到 workpiece 坐标 |
| `workpiece_to_base()` | workpiece 点转换回 base 坐标 |
| `roi_range_workpiece()` | 计算 `u/v` 有效规划范围 |
| `grid_workpiece()` | 在 `u/v` 平面生成蛇形网格 |
| `polyfit_workpiece()` | 在 `u/v/n` 中做局部平面拟合 |
| `path_plan_workpiece()` | 生成局部坐标系路径并输出 base TCP 位姿 |

## 8. 主流程改造路径

当前 `main()` 中直接调用：

```cpp
pcl_handle->pcl_filter();
pcl_handle->roi_range();
pcl_handle->grid();
pcl_handle->path_plan();
```

建议改为读取模式后分支。可以新增一个公开函数：

```cpp
void run_planning_pipeline();
```

由它统一管理：

```text
pcl_filter()

if planning_mode == oriented_plane:
    estimate_workpiece_frame_from_roi()
    roi_range_workpiece()
    grid_workpiece()
    path_plan_workpiece()
else:
    roi_range()
    grid()
    path_plan()
```

如果不想新增 `run_planning_pipeline()`，也可以在 `main()` 中最小改动：

```cpp
pcl_handle->pcl_filter();

if (pcl_handle->is_oriented_plane_mode()) {
    pcl_handle->estimate_workpiece_frame_from_roi();
    pcl_handle->roi_range_workpiece();
    pcl_handle->grid_workpiece();
    pcl_handle->path_plan_workpiece();
} else {
    pcl_handle->roi_range();
    pcl_handle->grid();
    pcl_handle->path_plan();
}
```

但更推荐封装到类内部，避免 `main()` 暴露太多状态。

## 9. 分阶段实现建议

### 阶段 1：新增模式与结构，不改变旧模式

目标：

- 新增 `planning_mode:=oriented_plane`。
- 新增 `WorkpieceFrame` 结构。
- 新增参数和函数空壳。
- 确保 `plane` 和 `cylinder_preplan` 编译、运行行为不变。

验收：

```text
planning_mode:=plane 时，仍调用旧 roi_range/grid/path_plan。
planning_mode:=cylinder_preplan 时，仍保留旧二次框选逻辑。
planning_mode:=oriented_plane 时，进入新分支。
```

### 阶段 2：实现 PCA 工件坐标系估计

目标：

- 从 `pcl_roi.pcd` 读取 ROI 点。
- 计算 `origin/u_axis/v_axis/n_axis`。
- 进行单位化和正交化。
- 添加日志输出。

建议日志：

```text
Workpiece frame estimated:
origin=[...]
u_axis=[...]
v_axis=[...]
n_axis=[...]
extent_u=[min,max]
extent_v=[min,max]
```

验收：

```text
ROI 点数不足时报错。
PCA 失败时报错。
U/V/N 均为单位向量。
U/V/N 两两点积接近 0。
```

### 阶段 3：实现 U/V ROI 范围和网格

目标：

- 将 ROI 点投影到 `u/v/n`。
- 统计 `u/v` 边界。
- 按探头尺寸内缩。
- 生成 `grid_uv_`。

验收：

```text
工件斜放时，grid_uv_ 在局部坐标中仍是规则矩形蛇形路径。
内缩不足时能报错并跳过路径生成。
```

### 阶段 4：实现 workpiece 局部拟合

目标：

- 将 `pcl_border.pcd` 或 `pcl_trans.pcd` 点转换到 workpiece 坐标。
- 对每个 `grid_uv_` 点按 `u/v` 距离查邻域。
- 拟合 `n = a*u + b*v + c`。
- 输出 base 坐标系下的表面点和法线。

验收：

```text
斜放平面上，拟合得到的 normal_base 接近工件法线。
路径点 surface_base 落在工件表面附近。
```

### 阶段 5：复用姿态生成和输出

目标：

- 将 `path_plan()` 中姿态连续化逻辑抽取成可复用辅助函数，或在 `path_plan_workpiece()` 中先复制再整理。
- 保持 `surface_clearance`、`probe_offset`、`tool_mount_rotation` 生效。
- 输出格式保持 `path_map.txt` 不变。

验收：

```text
path_map.txt 每行仍为 6 列 base TCP 位姿。
姿态连续性处理仍生效。
Z 突变限制和拐弯点平滑仍生效或有等效逻辑。
```

### 阶段 6：补充调试输出和文档

建议输出：

| 文件 | 用途 |
|---|---|
| `pcl_workpiece_roi_debug.pcd` | 可选保存 ROI 点在 base 中的调试结果 |
| `path_map_workpiece_debug.txt` | 可选保存中间 `u/v/n` 路径 |
| `workpiece_frame.txt` | 保存估计出的 `O/U/V/N` |

注意这些文件不应替代正式 `path_map.txt`。

## 10. 兼容性要求

后续 agent 修改代码时必须满足：

1. 不改变 `plane` 默认模式。
2. 不改变 `cylinder_preplan` 的二次 ROI 框选行为。
3. 不改变旧模式下 `path_map.txt` 的含义和格式。
4. 不移除原有 `roi_range()`、`grid()`、`path_plan()`。
5. 新模式失败时不能生成看似正常但实际错误的路径。
6. 所有新参数必须有默认值。
7. 旧启动命令仍能运行。

## 11. 关键边界情况

### ROI 点数太少

若 ROI 点数少于拟合/估计所需下限，例如小于 8，应直接报错：

```text
Workpiece frame estimation failed: ROI has too few points.
```

### ROI 接近正方形

PCA 对正方形表面可能出现 `U/V` 方向不稳定，因为两个平面内特征值接近。

建议：

- 第一版记录警告。
- 后续增加 `manual_points` 模式指定 `U` 方向。

### 工件表面不是单一平面

如果 ROI 是强曲面，PCA 得到的是整体平均平面，局部路径仍可通过 `polyfit_workpiece()` 贴合，但 `U/V` 边界可能不能完全表示真实展开区域。

建议第一版定位为：

```text
适合平面或轻微弯曲方形/矩形表面。
```

强曲面或圆柱面可后续设计专门的参数化坐标系。

### 法线方向翻转

PCA 得到的 `N` 方向有正负二义性。

建议让 `N` 尽量朝向相机或沿已有点云法线方向保持一致。第一版可采用：

```text
如果 N 与相机观察方向相反，则翻转 N，同时重新计算 V。
```

如果暂不实现相机方向判断，至少要在路径姿态生成中沿用现有法线连续化逻辑，减少突然翻转。

## 12. 推荐伪代码

### 12.1 模式分支

```cpp
void pcl_cloudslam::run_planning_pipeline()
{
    pcl_filter();

    std::string planning_mode;
    get_parameter("planning_mode", planning_mode);

    if (planning_mode == "oriented_plane") {
        if (!estimate_workpiece_frame_from_roi()) {
            RCLCPP_ERROR(get_logger(), "Failed to estimate workpiece frame, abort oriented planning.");
            return;
        }
        roi_range_workpiece();
        grid_workpiece();
        path_plan_workpiece();
        return;
    }

    roi_range();
    grid();
    path_plan();
}
```

### 12.2 PCA 坐标系估计

```cpp
bool pcl_cloudslam::estimate_workpiece_frame_from_roi()
{
    load pcl_roi.pcd;
    if point_count < threshold:
        return false;

    origin = mean(points);
    covariance = sum((p - origin) * (p - origin).transpose());
    eigenvectors = solve(covariance);

    n_axis = eigenvector_of_smallest_eigenvalue;
    u_axis = eigenvector_of_largest_eigenvalue;

    u_axis = u_axis - dot(u_axis, n_axis) * n_axis;
    normalize(u_axis);
    v_axis = normalize(cross(n_axis, u_axis));
    u_axis = normalize(cross(v_axis, n_axis));

    workpiece_frame_.valid = true;
    return true;
}
```

### 12.3 U/V 网格

```cpp
void pcl_cloudslam::grid_workpiece()
{
    u = roi_range_u_.min;
    direction_forward = true;

    while (u <= roi_range_u_.max) {
        if (direction_forward) {
            for v from v_min to v_max step v_step:
                grid_uv_.push_back({u, v});
        } else {
            for v from v_max to v_min step -v_step:
                grid_uv_.push_back({u, v});
        }
        direction_forward = !direction_forward;
        u += u_step;
    }
}
```

### 12.4 局部拟合

```cpp
bool pcl_cloudslam::polyfit_workpiece(const Eigen::Vector2d& point_uv, double (&pose_out)[6])
{
    collect neighbor points by distance in u/v;
    fit n = a*u + b*v + c;

    surface_local = [u, v, fitted_n];
    normal_local = normalize([-a, -b, 1]);

    surface_base = workpiece_to_base(surface_local);
    normal_base = normal_local.x * U + normal_local.y * V + normal_local.z * N;

    pose_out[0..2] = surface_base;
    pose_out[3..5] = normal_base;
    return true;
}
```

## 13. 测试与验证建议

### 13.1 不回归旧模式

至少验证：

```bash
ros2 run pointcloudslam_cpp pcl_cloudslam --ros-args -p planning_mode:=plane
ros2 run pointcloudslam_cpp pcl_cloudslam --ros-args -p planning_mode:=cylinder_preplan
```

检查：

- 能进入原有 ROI 框选流程。
- 能生成 `path_map.txt`。
- 输出格式仍为 6 列。

### 13.2 验证新模式方向

准备一个相对基坐标系旋转明显的矩形平面点云。

运行：

```bash
ros2 run pointcloudslam_cpp pcl_cloudslam --ros-args \
  -p planning_mode:=oriented_plane \
  -p workpiece_frame_method:=pca
```

检查：

- 日志中输出的 `u_axis` 与工件长边方向接近。
- `grid_uv_` 中路径是规则蛇形。
- 转换回 base 后的路径沿工件边界方向，而不是沿 base X/Y。
- `path_map.txt` 中点位落在工件表面范围内。

### 13.3 验证探头内缩

选择一个较小 ROI，设置较大的：

```text
probe_length_m
probe_width_m
probe_boundary_margin_m
```

检查：

- 如果 ROI 不足以容纳探头，应报错。
- 如果 ROI 足够，路径点不应贴近工件边界。

### 13.4 验证拟合稳定性

调整：

```text
ArroundDistance
normal_smoothing_alpha
max_dz_step
```

检查：

- 邻域点不足时有清晰警告。
- 法线不应频繁翻转。
- 相邻路径点的姿态和高度变化应连续。

## 14. 推荐交给 agent 的实施顺序

后续让 agent 调整代码时，建议按以下顺序下发任务：

1. 添加 `oriented_plane` 模式分支，不改旧模式。
2. 添加 `WorkpieceFrame`、局部坐标变换函数和参数。
3. 实现 PCA 工件坐标系估计，并输出日志。
4. 实现 `roi_range_workpiece()` 和 `grid_workpiece()`。
5. 实现 `polyfit_workpiece()` 和 `path_plan_workpiece()`。
6. 抽取或复用现有姿态连续化、探头补偿和路径后处理逻辑。
7. 更新 `README.md` 和 `docs/PROJECT_GUIDE.md`，说明新模式。
8. 增加测试或至少提供可复现实验数据与验证脚本。

## 15. 最小可用版本范围

第一版最小可用版本应包含：

- 新增 `planning_mode:=oriented_plane`。
- 使用 PCA 从 ROI 估计工件局部坐标系。
- 在 `U/V` 中计算 ROI 范围和蛇形网格。
- 在 `U/V/N` 中做局部平面拟合。
- 输出 base 坐标系下 `path_map.txt`。
- 旧模式不受影响。

第一版可以暂不包含：

- 手动两点指定 `U` 方向。
- 多边形 ROI 内点过滤。
- 最小面积旋转矩形。
- 圆柱面展开参数化。
- 可视化显示局部坐标轴。

这些可以作为后续增强项。

## 16. 结论

当前程序的问题不是点云没有转换到机械臂基坐标系，而是路径规划坐标系选得过于固定：直接使用了机械臂基坐标系的 `X/Y`。当工件相对基坐标系旋转时，`base X/Y` 外接矩形不能准确表达工件真实边界。

推荐新增 `oriented_plane` 模式，通过 ROI 点云估计工件局部坐标系 `O/U/V/N`，在 `U/V` 平面中生成蛇形路径，再转换回机械臂基坐标系输出。这样既能保持下游执行接口不变，又能让路径方向与工件边界一致，并且不破坏现有 `plane` 和 `cylinder_preplan` 流程。

