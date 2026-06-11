# 凹曲面 ROI 提取方法实现说明

## 1. 实现目标

在现有 ROS2 + PCL 点云路径规划程序中，新增一个“凹曲面 ROI 精筛”功能。

当前流程中，用户通过 PCLVisualizer 手动框选粗 ROI，并保存为：

```text
pcl_roi.pcd
```

现在需要在这个粗 ROI 的基础上，自动去除：

1. 蓝色平台平面；
2. 工件侧面圆柱曲面及不需要保留的圆弧过渡区域；
3. 孤立噪声点和边缘干扰点；

最终只保留待扫查的凹曲面点云，并保存为：

```text
pcl_roi_refined.pcd
```

该功能暂时只负责 ROI 提取，不处理工件坐标系建立，也不处理凹曲面路径规划。

---

## 2. 方法总体思路

实际工件放置在蓝色平台上，待扫查凹曲面比平台表面高约 4–6 cm。深度相机拍摄视角与现场照片类似。

因此可以采用以下分割流程：

```text
手动粗框选 ROI
    ↓
RANSAC 拟合并去除平台平面
    ↓
对剩余点云估计法向
    ↓
根据法向与平台法向夹角去除工件侧面圆柱面和部分圆弧过渡区
    ↓
欧式聚类或离群点滤波去除孤立噪声
    ↓
保留目标凹曲面点云
    ↓
输出 pcl_roi_refined.pcd
```

该方法不依赖颜色信息，仍然使用当前程序中的 `pcl::PointXYZ` 点云即可。

需要注意：目标凹曲面、圆弧过渡区和侧面圆柱面在点云上是连续连接的，因此不能依赖欧式聚类把它们分开。该方案的核心分割依据是局部表面法向与平台法向的夹角；欧式聚类只用于清理法向筛选后的孤立噪声或小片残留点。

---

## 3. 建议新增函数

在 `pcl_cloudslam` 类中新增函数：

```cpp
void pcl_cloudslam::refine_roi_by_platform_and_normal();
```

函数输入：

```text
file_path + "pcl_roi.pcd"
```

函数输出：

```text
file_path + "pcl_roi_no_platform.pcd"
file_path + "pcl_roi_no_side.pcd"
file_path + "pcl_roi_refined.pcd"
```

其中前两个文件用于调试，最后一个文件用于后续路径规划。

---

## 4. 程序调用位置

当前主流程类似：

```cpp
pcl_handle->pcl_filter();
pcl_handle->roi_range();
pcl_handle->grid();
pcl_handle->path_plan();
```

新增凹曲面模式时，建议只在该模式下插入 ROI 精筛步骤：

```cpp
pcl_handle->pcl_filter();
pcl_handle->refine_roi_by_platform_and_normal();  // 仅 concave_surface 模式调用
pcl_handle->roi_range();
pcl_handle->grid();
pcl_handle->path_plan();
```

同时，后续 `roi_range()` 中读取 ROI 文件时，不能全局从：

```cpp
pcl::io::loadPCDFile(file_path + "pcl_roi.pcd", *cloud_roi);
```

直接改为：

```cpp
pcl::io::loadPCDFile(file_path + "pcl_roi_refined.pcd", *cloud_roi);
```

否则会影响现有 `plane` 和 `cylinder_preplan` 两种模式。推荐做成按模式选择输入：

```text
planning_mode == plane
    roi_range() 继续读取 pcl_roi.pcd

planning_mode == cylinder_preplan
    roi_range() 继续读取 pcl_roi.pcd

planning_mode == concave_surface
    先执行 refine_roi_by_platform_and_normal()
    roi_range() 读取 pcl_roi_refined.pcd
```

不建议在精筛后直接覆盖 `pcl_roi.pcd`，因为会降低调试可追溯性，也容易误影响其他规划模式。

---

## 5. 详细算法步骤

### 5.1 读取粗 ROI

读取手动框选得到的粗 ROI：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile(file_path + "pcl_roi.pcd", *cloud_roi);
```

需要检查：

```text
cloud_roi 是否为空
点数是否足够
文件是否读取成功
```

如果读取失败，直接报错并返回，不继续执行后续路径规划。

---

### 5.2 提取粗 ROI 视角四角平台候选点

不建议直接使用全部粗 ROI 点云拟合平台平面。粗 ROI 中可能包含大量凹曲面、侧壁和圆弧过渡区点，如果这些工件点数量多于平台点，RANSAC 拟合到的最大平面不一定是蓝色平台。

推荐优先使用用户粗框选 ROI 在当前视角下的四个角落邻域点云作为平台候选点。用户框选时应让四个角落尽量落在蓝色平台区域，工件主体位于框选区域中部。

注意：这里的“四个角落”不是四个单点，而是四个小块点云 patch。单点深度噪声较大，无法稳定拟合平面。

建议流程：

```text
读取 pcl_roi.pcd
    ↓
根据粗 ROI 在当前视角下的 2D 边界确定左上、右上、左下、右下四个角
    ↓
从每个角落取一个小矩形 patch
    ↓
合并四个角落 patch，得到平台候选点云
    ↓
保存 pcl_roi_platform_candidates.pcd 供调试
```

推荐初始参数：

```text
corner_patch_ratio = 0.15
platform_candidate_min_points = 80
```

含义：

```text
corner_patch_ratio 表示每个角落 patch 的宽度和高度占粗 ROI 视角 2D 包围框的比例。
platform_candidate_min_points 表示四角候选点合并后至少需要的点数。
```

调参方向：

```text
如果四角候选点太少：增大 corner_patch_ratio
如果角落 patch 混入工件边缘：减小 corner_patch_ratio，或粗框选时留出更多平台边界
```

如果当前实现阶段暂时无法获得手动框选矩形的屏幕坐标，可以先用粗 ROI 点云在当前观察平面上的投影边界近似四个角落。后续如果需要更严格的“屏幕视角四角”，再在 ROI 框选回调中额外保存选择矩形和相机视角信息。

### 5.3 使用四角候选点 RANSAC 拟合平台平面

使用 PCL 的 `SACSegmentation` 对四角平台候选点拟合平面。该平面应为蓝色平台表面。

推荐使用：

```cpp
pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(platform_plane_threshold);
```

推荐初始参数：

```text
platform_plane_threshold = 0.005 m
```

可调范围：

```text
0.003 ~ 0.008 m
```

输出平台平面方程：

```text
a*x + b*y + c*z + d = 0
```

平台单位法向：

```cpp
Eigen::Vector3d n_platform(a, b, c);
n_platform.normalize();
```

需要增加失败回退机制：

```text
如果四角候选点数量不足
    回退到全粗 ROI RANSAC 拟合平台

如果四角候选点 RANSAC 内点数量太少
    回退到全粗 ROI RANSAC 拟合平台

如果拟合出的平台平面不合理
    报错并停止凹曲面 ROI 精筛
```

建议新增调试输出：

```text
四角候选点总数
平台 RANSAC 输入来源：corner_patches 或 full_roi_fallback
平台平面内点数量
平台平面法向
```

---

### 5.4 根据高度去除平台点

对粗 ROI 中每个点计算到平台平面的有符号距离：

```text
h = a*x + b*y + c*z + d
```

注意：平台法向方向可能朝上，也可能朝下。需要根据工件点相对平台的位置自动修正符号。

推荐做法：

1. 计算所有点到平台的距离；
2. 如果大部分非平台点的距离为负，则将平面法向和 d 同时取反；
3. 保证工件位于平台的正方向。

然后按高度过滤：

```text
保留 height_min < h < height_max 的点
```

推荐初始参数：

```text
height_min = 0.015 m
height_max = 0.090 m
```

由于待扫查凹曲面比平台高约 4–6 cm，所以这个范围能去除平台，同时保留工件上表面区域。

输出中间文件：

```text
pcl_roi_no_platform.pcd
```

---

### 5.5 对去平台后的点云估计法向

对 `pcl_roi_no_platform.pcd` 估计法向。

推荐使用：

```cpp
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
ne.setInputCloud(cloud_no_platform);
ne.setSearchMethod(tree);
ne.setKSearch(normal_k);
```

推荐初始参数：

```text
normal_k = 40
```

可调范围：

```text
30 ~ 60
```

如果点云密度不稳定，也可以改用半径搜索：

```text
normal_radius = 0.008 ~ 0.020 m
```

---

### 5.6 根据法向去除侧面圆柱曲面和部分圆弧过渡区

工件侧面圆柱面近似为竖直侧壁，其局部法向接近水平；待扫查凹曲面整体更接近上表面，其法向与平台法向具有较大的投影关系。

目标凹曲面与侧面圆柱面之间存在连续圆弧过渡，点云上通常不会断开。因此本步骤不是尝试把连续点云聚类分开，而是用法向夹角主动定义要保留的凹曲面主体边界，并允许删除一部分不需要扫查的圆弧过渡区。

对每个点计算：

```text
s = abs(n_i · n_platform)
```

其中：

```text
n_i         当前点局部法向
n_platform 平台法向
```

判断逻辑：

```text
s 接近 1：该点所在表面接近平台方向，倾向于凹曲面主体区域
s 接近 0：该点所在表面接近竖直侧面，倾向于圆柱侧壁
中间值：倾向于圆弧过渡区，是否保留由 normal_dot_min 决定
```

保留条件：

```text
abs(n_i · n_platform) > normal_dot_min
```

推荐初始参数：

```text
normal_dot_min = 0.55
```

该参数可理解为允许保留的最大倾斜角：

```text
normal_dot_min = cos(max_tilt_angle)

0.35 -> 约 69°
0.50 -> 60°
0.55 -> 约 57°
0.65 -> 约 49°
0.75 -> 约 41°
```

调参方向：

```text
如果侧壁或圆弧过渡区混入太多：增大到 0.65 或 0.75
如果凹曲面边缘被误删：减小到 0.45 或 0.50
```

输出中间文件：

```text
pcl_roi_no_side.pcd
```

---

### 5.7 欧式聚类或离群点滤波清理残留噪声

对 `pcl_roi_no_side.pcd` 做欧式聚类，去除孤立噪声和小片残留点。由于凹曲面、圆弧过渡区和侧壁原本连续连接，欧式聚类不用于区分凹曲面和侧壁，只用于清理法向筛选后的残留噪声。

推荐使用：

```cpp
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
ec.setClusterTolerance(cluster_tolerance);
ec.setMinClusterSize(min_cluster_size);
ec.setMaxClusterSize(max_cluster_size);
```

推荐初始参数：

```text
cluster_tolerance = 0.008 m
min_cluster_size = 100
max_cluster_size = 250000
```

保留策略：

优先保留最大点数的 cluster，作为最终凹曲面 ROI。

如果后续发现最大 cluster 偶尔不是目标凹曲面，可以增加“用户点选种子点”策略：保留包含种子点或距离种子点最近的 cluster。

最终输出：

```text
pcl_roi_refined.pcd
```

---

## 6. 建议新增 ROS2 参数

在构造函数中新增以下参数：

```cpp
declare_parameter<double>("platform_plane_threshold", 0.005);
declare_parameter<double>("roi_height_min", 0.015);
declare_parameter<double>("roi_height_max", 0.090);
declare_parameter<int>("roi_normal_k", 40);
declare_parameter<double>("roi_normal_dot_min", 0.55);
declare_parameter<double>("roi_cluster_tolerance", 0.008);
declare_parameter<int>("roi_min_cluster_size", 100);
declare_parameter<int>("roi_max_cluster_size", 250000);
```

运行时可通过命令行调整：

```bash
ros2 run pointcloudslam_cpp pcl_cloudslam --ros-args \
  -p platform_plane_threshold:=0.005 \
  -p roi_height_min:=0.015 \
  -p roi_height_max:=0.090 \
  -p roi_normal_k:=40 \
  -p roi_normal_dot_min:=0.55 \
  -p roi_cluster_tolerance:=0.008 \
  -p roi_min_cluster_size:=100
```

---

## 7. 推荐初始参数

第一版可以先使用：

```text
platform_plane_threshold = 0.005
roi_height_min = 0.015
roi_height_max = 0.090
roi_normal_k = 40
roi_normal_dot_min = 0.55
roi_cluster_tolerance = 0.008
roi_min_cluster_size = 100
roi_max_cluster_size = 250000
```

后续根据提取效果调参。

---

## 8. 调参规则

### 8.1 平台没有完全去除

调整方向：

```text
增大 platform_plane_threshold
增大 roi_height_min
```

例如：

```text
roi_height_min: 0.015 → 0.020
```

---

### 8.2 凹曲面被删掉太多

调整方向：

```text
减小 roi_height_min
增大 roi_height_max
减小 roi_normal_dot_min
增大 cluster_tolerance
```

例如：

```text
roi_normal_dot_min: 0.55 → 0.45
```

---

### 8.3 侧面圆柱面或圆弧过渡区混入太多

调整方向：

```text
增大 roi_normal_dot_min
减小 roi_height_max
```

例如：

```text
roi_normal_dot_min: 0.55 → 0.65
```

---

### 8.4 最终 ROI 断裂成多块

调整方向：

```text
增大 cluster_tolerance
减小 roi_normal_dot_min
增大 normal_k
```

例如：

```text
cluster_tolerance: 0.008 → 0.012
```

---

### 8.5 噪声点较多

调整方向：

```text
增大 min_cluster_size
减小 cluster_tolerance
增加 StatisticalOutlierRemoval
```

可选增加：

```cpp
pcl::StatisticalOutlierRemoval<pcl::PointXYZ>
```

---

## 9. 可视化调试要求

实现完成后，建议每一步都保存 PCD 文件，方便用 PCLVisualizer 检查：

```text
pcl_roi.pcd              原始手动粗选 ROI
pcl_roi_no_platform.pcd  去平台后的结果
pcl_roi_no_side.pcd      去侧壁后的结果
pcl_roi_refined.pcd      最终凹曲面 ROI
```

同时在日志中输出：

```text
原始 ROI 点数
平台平面内点数量
去平台后点数
去侧面后点数
聚类数量
最大 cluster 点数
最终 ROI 点数
```

---

## 10. 验收标准

第一版实现成功的标准：

1. 手动粗框选区域可以包含平台、工件侧壁和完整凹曲面；
2. 运行精筛函数后，`pcl_roi_refined.pcd` 中应基本只剩待扫查凹曲面；
3. 蓝色平台点应基本被去除；
4. 工件侧面圆柱面和不需要扫查的圆弧过渡区应大部分被去除；
5. 输出点云应保持连续，不应只剩零散点；
6. 若提取失败，程序应输出明确错误日志，不应直接崩溃；
7. 原始粗 ROI 与各阶段中间结果应保留，便于调试。

---

## 11. 当前阶段暂不处理的内容

本次实现只处理 ROI 提取，不处理以下问题：

```text
工件坐标系确定
凹曲面几何参数化
凹曲面路径规划
机器人末端姿态规划
恒力接触扫查控制
```

这些内容后续在 ROI 提取稳定后再继续实现。
