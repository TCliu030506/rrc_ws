#include "../include/pointcloud_planner/pcl_cloudslam.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define BORDER_DIS 0.02

namespace {

Eigen::Vector3d project_to_tangent_plane(const Eigen::Vector3d &vector_in, const Eigen::Vector3d &normal)
{
    return vector_in - vector_in.dot(normal) * normal;
}

Eigen::Vector3d choose_fallback_tangent(const Eigen::Vector3d &normal)
{
    Eigen::Vector3d ref_x(1.0, 0.0, 0.0);
    Eigen::Vector3d tangent = project_to_tangent_plane(ref_x, normal);
    if (tangent.norm() < 1e-8) {
        Eigen::Vector3d ref_y(0.0, 1.0, 0.0);
        tangent = project_to_tangent_plane(ref_y, normal);
    }
    if (tangent.norm() < 1e-8) {
        tangent = Eigen::Vector3d::UnitX();
    }
    return tangent.normalized();
}

Eigen::Vector3d rotation_matrix_to_rotvec_continuous(
    const Eigen::Matrix3d &rotation,
    bool has_previous_quaternion,
    const Eigen::Quaterniond &previous_quaternion,
    Eigen::Quaterniond *current_quaternion_out)
{
    Eigen::Quaterniond quaternion(rotation);
    quaternion.normalize();

    // q and -q represent the same rotation. Enforce sign continuity to avoid
    // discontinuous orientation representation in the exported path.
    if (has_previous_quaternion && quaternion.dot(previous_quaternion) < 0.0) {
        quaternion.coeffs() *= -1.0;
    }

    if (current_quaternion_out != nullptr) {
        *current_quaternion_out = quaternion;
    }

    Eigen::AngleAxisd aa(quaternion);
    if (!std::isfinite(aa.angle()) || aa.angle() < 1e-8) {
        return Eigen::Vector3d::Zero();
    }
    return aa.axis() * aa.angle();
}

Eigen::Matrix3d rpy_to_rotation_matrix(double rx, double ry, double rz)
{
    Eigen::AngleAxisd roll_angle(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(rz, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quaternion = yaw_angle * pitch_angle * roll_angle;
    return quaternion.toRotationMatrix();
}

}



pcl_cloudslam::pcl_cloudslam():rclcpp::Node("pcl_cloudslam_node"),viewer(new pcl::visualization::PCLVisualizer("Pointcloud"))
{
    RCLCPP_INFO(this->get_logger(), "点云SLAM节点启动");
    const auto package_share_dir = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("pointcloud_planner")
    );
    const auto data_dir = package_share_dir / "data";
    const auto path_planning_dir = data_dir / "path_planning";
    std::filesystem::create_directories(path_planning_dir);
    file_path = data_dir.string() + "/";
    plan_path = path_planning_dir.string() + "/";
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    is_record = true;
    is_areapicked = false;
    pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/camera/depth/color/points", 10, std::bind(&pcl_cloudslam::pcl_callback, this, std::placeholders::_1));
        // 创建 robotstate 订阅者
    robot_state_sub_ = this->create_subscription<ur5_msg::msg::RobotState>(
        "robotstate", 10, std::bind(&pcl_cloudslam::robot_state_callback, this, std::placeholders::_1));    // 创建 robotstate 订阅者

	viewer->setBackgroundColor(0,0,0);

    declare_parameter<double>("Leafsize", 0.005);
    declare_parameter<double>("ArroundDistance", 0.01);
    declare_parameter<double>("x_step",0.0035);
    declare_parameter<double>("y_step",0.064);
    declare_parameter<double>("normal_smoothing_alpha", 0.2);
    declare_parameter<double>("tool_mount_rx", 3.141592653589793);
    declare_parameter<double>("tool_mount_ry", 0.0);
    declare_parameter<double>("tool_mount_rz", 0.0);
    declare_parameter<double>("tool_tip_x", 0.0);
    declare_parameter<double>("tool_tip_y", 0.0);
    // declare_parameter<double>("tool_tip_z", 0.21468);
    declare_parameter<double>("tool_tip_z", 0.0);
    declare_parameter<double>("surface_clearance", 0.005);
    declare_parameter<double>("max_dz_step", 0.003);
    declare_parameter<bool>("enable_turn_z_smoothing", true);
    declare_parameter<double>("turn_dy_threshold", 1e-4);
    declare_parameter<double>("probe_length_m", 0.18);
    declare_parameter<double>("probe_width_m", 0.075);
    declare_parameter<double>("probe_boundary_margin_m", 0.0);
    declare_parameter<std::string>("planning_mode", "plane");
    declare_parameter<double>("cylinder_view_distance_m", 0.35);
    // 加载相机变换矩阵
    std::string camera_transform_path = file_path + "hand_eye_calibration.json";
    RCLCPP_INFO(this->get_logger(), "Loading camera transform from: %s", camera_transform_path.c_str());
    
    if (!load_camera_transform(camera_transform_path)) {
        RCLCPP_WARN(this->get_logger(), "Failed to load camera transform, using identity matrix");
        camera_to_end_effector = Eigen::Matrix4d::Identity();
    }
    double tool_tip_x;
    double tool_tip_y;
    double tool_tip_z;
    get_parameter("tool_tip_x", tool_tip_x);
    get_parameter("tool_tip_y", tool_tip_y);
    get_parameter("tool_tip_z", tool_tip_z);
    probe_offset << tool_tip_x, tool_tip_y, tool_tip_z;
    RCLCPP_INFO(
        this->get_logger(),
        "Tool tip offset in TCP frame: [%.5f, %.5f, %.5f]",
        probe_offset(0), probe_offset(1), probe_offset(2)
    );
}


pcl_cloudslam::~pcl_cloudslam(){}

void pcl_cloudslam::pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if(is_record)
    {
        pcl::fromROSMsg(*msg, *cloud);
        //pcl::io::savePCDFileASCII (file_path+"pcl_original.pcd", *cloud_update);
        //RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",msg->height,msg->width);
    }

}

void pcl_cloudslam::pcl_filter()
{
    // save original pointcloud
    is_record=false;
    //
    pcl::io::savePCDFileASCII (file_path+"pcl_original.pcd", *cloud);

    // read saved pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_path+"pcl_original.pcd",*cloud_original);
    std::cout << "PointCloud before filtering has: " << cloud_original->points.size() << " data points." << std::endl; //*
    viewer->addPointCloud(cloud_original, "cloud");
    viewer->spinOnce(3000);
    //filter
    float leaf;
    get_parameter("Leafsize",leaf);
    RCLCPP_INFO(get_logger(),"Leafsize: %lf", leaf);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud_original);
	vg.setLeafSize(leaf, leaf, leaf);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*
    pcl::io::savePCDFileASCII (file_path+"pcl_filter.pcd", *cloud_filtered);
    viewer->removePointCloud("cloud");
    viewer->addPointCloud(cloud_filtered, "cloud");
    viewer->spinOnce (2000);

    pose_transform();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_path+"pcl_trans.pcd",*cloud_trans);
    // 在将点云添加到可视化器前，将视角设置为深度相机正视图（以便方便区域划分）
    double pose_end[6];
    double joint_pos[6];
    double joint_vel[6];
    bool success = get_current_robot_pose(joint_pos, joint_vel, pose_end);
    Eigen::Matrix4d end_to_base = Eigen::Matrix4d::Identity();
    if (success) {
        end_to_base = calculate_transform_matrix(pose_end);
    } else {
        // 使用默认位姿以避免崩溃
        double default_pose[6] = {0.5, 0.2, 0.3, 0.0, 0.0, 1.57};
        end_to_base = calculate_transform_matrix(default_pose);
    }

    // 计算相机在基坐标系下的位置与朝向（camera optical axis 为 (0,0,1)）
    Eigen::Vector4d cam_in_end = camera_to_end_effector * Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector3d cam_pos_base = (end_to_base * cam_in_end).head<3>();
    Eigen::Vector3d cam_dir_base = end_to_base.block<3,3>(0,0) * (camera_to_end_effector.block<3,3>(0,0) * Eigen::Vector3d(0.0, 0.0, 1.0));
    Eigen::Vector3d cam_up_base = end_to_base.block<3,3>(0,0) * (camera_to_end_effector.block<3,3>(0,0) * Eigen::Vector3d(0.0, -1.0, 0.0));

    Eigen::Vector3d view_point = cam_pos_base + cam_dir_base;

    viewer->removePointCloud("cloud");
    viewer->setCameraPosition(
        cam_pos_base(0), cam_pos_base(1), cam_pos_base(2),
        view_point(0), view_point(1), view_point(2),
        cam_up_base(0), cam_up_base(1), cam_up_base(2)
    );
    viewer->addPointCloud(cloud_trans, "cloud");
    viewer->registerAreaPickingCallback(&pcl_cloudslam::areapickingcallback,*this);
    while (!viewer->wasStopped () && !is_areapicked)
    {
        viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    std::string planning_mode;
    double cylinder_view_distance_m;
    get_parameter("planning_mode", planning_mode);
    get_parameter("cylinder_view_distance_m", cylinder_view_distance_m);
    cylinder_view_distance_m = std::max(0.1, cylinder_view_distance_m);

    const bool use_cylinder_preplan = (planning_mode == "cylinder_preplan");
    if (use_cylinder_preplan) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi_pre(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile(file_path + "pcl_roi.pcd", *cloud_roi_pre) != 0 || cloud_roi_pre->points.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Cylinder preplan mode enabled, but first ROI is insufficient. Keep first ROI for planning.");
        } else {
            Eigen::Vector3d center(0.0, 0.0, 0.0);
            for (const auto &point : cloud_roi_pre->points) {
                center += Eigen::Vector3d(point.x, point.y, point.z);
            }
            center /= static_cast<double>(cloud_roi_pre->points.size());

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (const auto &point : cloud_roi_pre->points) {
                Eigen::Vector3d d(point.x - center(0), point.y - center(1), point.z - center(2));
                covariance += d * d.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(covariance);
            if (eig_solver.info() != Eigen::Success) {
                RCLCPP_WARN(this->get_logger(), "Cylinder axis estimation failed. Keep first ROI for planning.");
            } else {
                Eigen::Vector3d axis = eig_solver.eigenvectors().col(2).normalized();
                Eigen::Vector3d roi_normal = eig_solver.eigenvectors().col(0).normalized();
                if (axis.norm() < 1e-8) {
                    RCLCPP_WARN(this->get_logger(), "Cylinder axis norm too small. Keep first ROI for planning.");
                } else {
                    double mean_radius = 0.0;
                    for (const auto &point : cloud_roi_pre->points) {
                        Eigen::Vector3d d(point.x - center(0), point.y - center(1), point.z - center(2));
                        Eigen::Vector3d radial = d - d.dot(axis) * axis;
                        mean_radius += radial.norm();
                    }
                    mean_radius /= static_cast<double>(cloud_roi_pre->points.size());

                    const double view_distance = std::max(cylinder_view_distance_m, 3.0 * mean_radius);

                    // 新逻辑：先取首次 ROI 的切平面，再让该平面上的切线方向在视图中水平。
                    // 视线沿切平面法向；切线方向取圆柱中轴在切平面上的投影。
                    Eigen::Vector3d view_dir = roi_normal;
                    if (view_dir.norm() < 1e-8) {
                        view_dir = Eigen::Vector3d(0.0, 0.0, 1.0);
                    }
                    view_dir.normalize();

                    Eigen::Vector3d tangent_dir = axis - axis.dot(view_dir) * view_dir;
                    if (tangent_dir.norm() < 1e-8) {
                        tangent_dir = eig_solver.eigenvectors().col(1);
                        tangent_dir = tangent_dir - tangent_dir.dot(view_dir) * view_dir;
                    }
                    if (tangent_dir.norm() < 1e-8) {
                        tangent_dir = view_dir.unitOrthogonal();
                    }
                    tangent_dir.normalize();

                    // PCL 相机中：screen_right = view_dir x up。
                    // 要让切线水平，则令 screen_right = tangent_dir => up = tangent_dir x view_dir。
                    Eigen::Vector3d up = tangent_dir.cross(view_dir);
                    if (up.norm() < 1e-8) {
                        up = view_dir.unitOrthogonal();
                    }
                    up.normalize();

                    Eigen::Vector3d view_from = center - view_dir * view_distance;
                    Eigen::Vector3d view_to = center;
                    viewer->setCameraPosition(
                        view_from(0), view_from(1), view_from(2),
                        view_to(0), view_to(1), view_to(2),
                        up(0), up(1), up(2)
                    );

                    is_areapicked = false;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Cylinder preplan mode: second manual ROI selection started. roi_normal(view_dir)=[%.4f, %.4f, %.4f], tangent_horizontal=[%.4f, %.4f, %.4f], axis=[%.4f, %.4f, %.4f], mean_radius=%.4f m",
                        view_dir(0), view_dir(1), view_dir(2),
                        tangent_dir(0), tangent_dir(1), tangent_dir(2),
                        axis(0), axis(1), axis(2),
                        mean_radius
                    );
                    while (!viewer->wasStopped () && !is_areapicked)
                    {
                        viewer->spinOnce (100);
                    }
                }
            }
        }
    }

    viewer->removePointCloud("cloud");


}


void pcl_cloudslam::pose_transform()
{
    RCLCPP_INFO(this->get_logger(), "开始执行坐标变换");
    double pose_end[6];
    
    // 使用 get_current_robot_pose 获取实际位姿
    double joint_pos[6];
    double joint_vel[6];
    bool success = get_current_robot_pose(joint_pos, joint_vel, pose_end);
    
    if (!success) {
        RCLCPP_WARN(this->get_logger(), "未能获取机器人位姿，使用默认位姿");
        // 如果获取失败，使用默认位姿
        double default_pose[6] = {0.5, 0.2, 0.3, 0.0, 0.0, 1.57};
        std::copy(default_pose, default_pose + 6, pose_end);
    }
    
    // 将位姿写入文件（可选）
    std::ofstream path_file(plan_path+"Trans_6_b.txt", std::ios::trunc);
    if (!path_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
    }
    for (size_t i = 0; i < 5; i++) {
        path_file << pose_end[i] << " ";
    }
    path_file << pose_end[5] << std::endl;
    path_file.close();
    
    // 日志输出获取到的位姿
    RCLCPP_INFO(this->get_logger(), "使用位姿: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                pose_end[0], pose_end[1], pose_end[2], 
                pose_end[3], pose_end[4], pose_end[5]);
    
    // 后续的点云变换代码
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point_cloud;
    pcl::io::loadPCDFile(file_path+"pcl_filter.pcd",*cloud_ori);
    
    // 计算末端到基坐标系的变换矩阵
    Eigen::Matrix4d end_to_base = calculate_transform_matrix(pose_end);
    
    // 对每个点应用变换
    for(const auto& point : cloud_ori->points)
    {
       // 1. 相机坐标系到末端坐标系
        Eigen::Vector4d temp_camera(point.x, point.y, point.z, 1);
        Eigen::Vector4d temp_end = camera_to_end_effector * temp_camera;
        
        // 2. 末端坐标系到基坐标系
        // 这里保留表面点的真实空间位置，不在点云阶段叠加工具尖端偏移。
        // TCP 到探头尖端的固定偏移只在 path_plan() 输出目标 TCP 时补偿一次。
        Eigen::Vector4d temp_base = end_to_base * temp_end;
        
        point_cloud.x = temp_base(0);
        point_cloud.y = temp_base(1);
        point_cloud.z = temp_base(2);
        cloud_trans->push_back(point_cloud);
    }
    
    pcl::io::savePCDFileASCII (file_path+"pcl_trans.pcd", *cloud_trans);
}

void pcl_cloudslam::areapickingcallback(const pcl::visualization::AreaPickingEvent &event,void *userdata)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr  secloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_trans(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(file_path+"pcl_trans.pcd",*cloud_trans);
    std::cout<<"Into here"<<std::endl;
    std::vector<int> indices;
    event.getPointsIndices(indices);
    pcl::IndicesPtr ind_plane=std::make_shared<std::vector<int>>(indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_trans);                       //导入点云数据
    extract.setIndices(ind_plane);                      //设置点云索引
    extract.setNegative(false);                  //设置为false，选择索引指向的点导出
    extract.filter(*secloud);                         //输出所选点云
    std::cout<<"Nums selected\t"<<secloud->points.size()<<std::endl;
    pcl::io::savePCDFile(file_path+"pcl_roi.pcd",*secloud);

    is_areapicked = true;
}


void pcl_cloudslam::roi_range()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_path+"pcl_roi.pcd",*cloud_roi);

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    // 遍历点云，计算每个坐标的最小值和最大值
    for (const auto& point : cloud_roi->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.x > max_x) max_x = point.x;
        if (point.y > max_y) max_y = point.y;
    }

    double probe_length_m;
    double probe_width_m;
    double probe_boundary_margin_m;
    get_parameter("probe_length_m", probe_length_m);
    get_parameter("probe_width_m", probe_width_m);
    get_parameter("probe_boundary_margin_m", probe_boundary_margin_m);

    probe_length_m = std::max(0.0, probe_length_m);
    probe_width_m = std::max(0.0, probe_width_m);
    probe_boundary_margin_m = std::max(0.0, probe_boundary_margin_m);

    // 约束：探头长边沿 y_step（即 ROI 的 y 方向），短边沿 x 方向。
    // 因此分别按半宽/半长在 x/y 方向内缩。
    const double shrink_x = 0.5 * probe_width_m + probe_boundary_margin_m;
    const double shrink_y = 0.5 * probe_length_m + probe_boundary_margin_m;

    const double roi_x_min = min_x + shrink_x;
    const double roi_x_max = max_x - shrink_x;
    const double roi_y_min = min_y + shrink_y;
    const double roi_y_max = max_y - shrink_y;

    if (roi_x_min >= roi_x_max || roi_y_min >= roi_y_max) {
        RCLCPP_ERROR(
            get_logger(),
            "ROI too small for probe footprint: raw_x=[%.4f, %.4f], raw_y=[%.4f, %.4f], probe=(L=%.4f, W=%.4f), shrink_x=%.4f, shrink_y=%.4f. No valid path region.",
            min_x, max_x, min_y, max_y, probe_length_m, probe_width_m, shrink_x, shrink_y
        );
        roi_range_x(0) = 1.0;
        roi_range_x(1) = 0.0;
        roi_range_y(0) = 1.0;
        roi_range_y(1) = 0.0;
        return;
    }

    roi_range_x(0) = roi_x_min;
    roi_range_x(1) = roi_x_max;
    roi_range_y(0) = roi_y_min;
    roi_range_y(1) = roi_y_max;

    RCLCPP_INFO(
        get_logger(),
        "Probe-aware ROI shrink applied (length along y_step): probe=(L=%.4f, W=%.4f), margin=%.4f, shrink_x=%.4f, shrink_y=%.4f, effective_x=[%.4f, %.4f], effective_y=[%.4f, %.4f]",
        probe_length_m,
        probe_width_m,
        probe_boundary_margin_m,
        shrink_x,
        shrink_y,
        roi_range_x(0),
        roi_range_x(1),
        roi_range_y(0),
        roi_range_y(1)
    );

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_path+"pcl_trans.pcd",*cloud_trans);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_border(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud_trans->points) {
        if (point.x < min_x-BORDER_DIS) continue;
        if (point.y < min_y-BORDER_DIS) continue;
        if (point.x > max_x+BORDER_DIS) continue;
        if (point.y > max_y+BORDER_DIS) continue;

        cloud_border->push_back(point);
    }
    pcl::io::savePCDFile(file_path+"pcl_border.pcd",*cloud_border);

}

void pcl_cloudslam::grid()
{
    if (roi_range_x(0) >= roi_range_x(1) || roi_range_y(0) >= roi_range_y(1)) {
        RCLCPP_ERROR(get_logger(), "Invalid ROI range, skip grid generation.");
        return;
    }

    double x_temp = roi_range_x(0);
    double y_temp = roi_range_y(0);
    bool flag_direction = false;
    double x_step;
    double y_step;
    get_parameter("x_step",x_step);
    get_parameter("y_step",y_step);
    while (x_temp<=roi_range_x(1))
    {
        if(!flag_direction)
        {
            while (y_temp<=roi_range_y(1))
            {
                Eigen::Vector2d point_temp(x_temp,y_temp);
                grid_xy.push_back(point_temp);
                y_temp += y_step;
            }
            y_temp -= y_step;;
            flag_direction = true;
        }
        else
        {
            while (y_temp>=roi_range_y(0))
            {
                Eigen::Vector2d point_temp(x_temp,y_temp);
                grid_xy.push_back(point_temp);
                y_temp -= y_step;
            }
            y_temp += y_step;;
            flag_direction = false;
        }
        x_temp +=x_step;
    }

}

void pcl_cloudslam::polyfit(Eigen::Vector2d point_xy,double (&pose_out)[6])
{
    int n = points_arround.size();
    Eigen::VectorXd point_x(n),point_y(n),point_z(n),point_1(n);
    Eigen::MatrixXd matrix(n,3);
    //Eigen::Vector3d coefficients;
    
    for(int i=0;i<n;i++)
    {
        point_x[i] = points_arround[i](0);
        point_y[i] = points_arround[i](1);
        point_z[i] = points_arround[i](2);
        point_1[i] = 1;
    }
    matrix<<point_x,point_y ,point_1;
    //SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(matrix, Eigen::ComputeThinV | Eigen::ComputeThinU);
    // 构建SVD分解结果
    auto coefficients = svd_holder.solve(point_z);
    
    // coefficients(0)*x + coefficients(1)*y + (-1)*z + coefficients(2) = 0
    pose_out[0] = point_xy(0);
    pose_out[1] = point_xy(1);
    pose_out[2] = coefficients(0)*point_xy(0) + coefficients(1)*point_xy(1) + coefficients(2);

    // 输出局部单位法线，后续在 path_plan() 中做连续姿态传播。
    Eigen::Vector3d normal(-coefficients(0), -coefficients(1), 1.0);
    double norm_len = normal.norm();
    if (norm_len < 1e-8) {
        normal = Eigen::Vector3d(0, 0, 1);
    } else {
        normal /= norm_len;
    }

    pose_out[3] = normal(0);
    pose_out[4] = normal(1);
    pose_out[5] = normal(2);

}

void pcl_cloudslam::path_plan()
{
    if (grid_xy.empty()) {
        RCLCPP_ERROR(get_logger(), "Grid is empty, skip path planning.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_path+"pcl_border.pcd",*cloud_trans);
    std::cout<<"The range of roi in x is ["<<roi_range_x(0)<<","<<roi_range_x(1)<<"]."<<std::endl;
    std::cout<<"The range of roi in y is ["<<roi_range_y(0)<<","<<roi_range_y(1)<<"]."<<std::endl;
    std::cout<<"The number of point in grid is "<<grid_xy.size()<<"."<<std::endl;
    double distance;
    double normal_smoothing_alpha;
    double tool_mount_rx;
    double tool_mount_ry;
    double tool_mount_rz;
    double surface_clearance;
    double max_dz_step;
    bool enable_turn_z_smoothing;
    double turn_dy_threshold;
    get_parameter("ArroundDistance",distance);
    get_parameter("normal_smoothing_alpha", normal_smoothing_alpha);
    get_parameter("tool_mount_rx", tool_mount_rx);
    get_parameter("tool_mount_ry", tool_mount_ry);
    get_parameter("tool_mount_rz", tool_mount_rz);
    get_parameter("surface_clearance", surface_clearance);
    get_parameter("max_dz_step", max_dz_step);
    get_parameter("enable_turn_z_smoothing", enable_turn_z_smoothing);
    get_parameter("turn_dy_threshold", turn_dy_threshold);
    normal_smoothing_alpha = std::clamp(normal_smoothing_alpha, 0.0, 1.0);
    max_dz_step = std::max(0.0, max_dz_step);
    turn_dy_threshold = std::max(1e-8, turn_dy_threshold);
    Eigen::Matrix3d tool_mount_rotation = rpy_to_rotation_matrix(tool_mount_rx, tool_mount_ry, tool_mount_rz);
    RCLCPP_INFO(get_logger(),"Distance: %3lf",distance);
    RCLCPP_INFO(get_logger(),"Normal smoothing alpha: %3lf",normal_smoothing_alpha);
    RCLCPP_INFO(get_logger(),"Tool mount RPY: [%.3f, %.3f, %.3f]", tool_mount_rx, tool_mount_ry, tool_mount_rz);
    RCLCPP_INFO(get_logger(),"Surface clearance: %.4f m", surface_clearance);
    RCLCPP_INFO(get_logger(),"Path post-process: max_dz_step=%.4f m, turn_z_smoothing=%s, turn_dy_threshold=%.6f", max_dz_step, enable_turn_z_smoothing ? "on" : "off", turn_dy_threshold);
    std::ofstream path_file(plan_path+"path_map.txt", std::ios::trunc);
    if (!path_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
    }
    std::vector<Eigen::Matrix<double, 6, 1>> path_poses;
    bool has_previous_frame = false;
    Eigen::Matrix3d previous_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d previous_smoothed_normal = Eigen::Vector3d::UnitZ();
    bool has_previous_tcp_quaternion = false;
    Eigen::Quaterniond previous_tcp_quaternion = Eigen::Quaterniond::Identity();

for (const auto& point : grid_xy) {
        points_arround.clear();
        for(const auto& point_in_cloud : cloud_trans->points)
        {
            Eigen::Vector2d point_temp(point_in_cloud.x,point_in_cloud.y);
            if((point_temp-point).norm()<distance)
            {
                Eigen::Vector3d point_arround(point_in_cloud.x,point_in_cloud.y,point_in_cloud.z);
                points_arround.push_back(point_arround);
            }
        }
        if(points_arround.size()<4)
        {
            RCLCPP_WARN(get_logger(),"The number of fit points is inadequate.");
            continue;
        }
        else{
            RCLCPP_INFO(get_logger(),"The number of fit points is %ld.",points_arround.size());
            double fit_pose[6];
            polyfit(point,fit_pose);

            Eigen::Vector3d normal(fit_pose[3], fit_pose[4], fit_pose[5]);
            if (has_previous_frame && normal.dot(previous_rotation.col(2)) < 0.0) {
                normal = -normal;
            }

            Eigen::Vector3d smoothed_normal = normal;
            if (has_previous_frame) {
                smoothed_normal = (1.0 - normal_smoothing_alpha) * previous_smoothed_normal +
                                  normal_smoothing_alpha * normal;
                if (smoothed_normal.norm() < 1e-8) {
                    smoothed_normal = normal;
                } else {
                    smoothed_normal.normalize();
                }
                if (smoothed_normal.dot(previous_rotation.col(2)) < 0.0) {
                    smoothed_normal = -smoothed_normal;
                }
            }

            Eigen::Vector3d x_axis;
            if (has_previous_frame) {
                // 将上一点的切向投影到当前切平面，得到最小扭转姿态。
                x_axis = project_to_tangent_plane(previous_rotation.col(0), smoothed_normal);
                if (x_axis.norm() < 1e-8) {
                    x_axis = project_to_tangent_plane(previous_rotation.col(1), smoothed_normal);
                }
            } else {
                x_axis = choose_fallback_tangent(smoothed_normal);
            }

            if (x_axis.norm() < 1e-8) {
                x_axis = choose_fallback_tangent(smoothed_normal);
            }
            x_axis.normalize();

            Eigen::Vector3d y_axis = smoothed_normal.cross(x_axis);
            if (y_axis.norm() < 1e-8) {
                x_axis = choose_fallback_tangent(smoothed_normal);
                y_axis = smoothed_normal.cross(x_axis);
            }
            y_axis.normalize();
            x_axis = y_axis.cross(smoothed_normal).normalized();

            Eigen::Matrix3d scan_rotation;
            scan_rotation.col(0) = x_axis;
            scan_rotation.col(1) = y_axis;
            scan_rotation.col(2) = smoothed_normal;

            // scan_rotation 定义的是扫查参考系；还需乘上 TCP 到探头的固定安装补偿。
            Eigen::Matrix3d tcp_rotation = scan_rotation * tool_mount_rotation;
            Eigen::Quaterniond current_tcp_quaternion;
            Eigen::Vector3d rot_vec = rotation_matrix_to_rotvec_continuous(
                tcp_rotation,
                has_previous_tcp_quaternion,
                previous_tcp_quaternion,
                &current_tcp_quaternion
            );
            double pose_out[6] = {
                fit_pose[0], fit_pose[1], fit_pose[2],
                rot_vec(0), rot_vec(1), rot_vec(2)
            };

            // fit_pose 位置表示表面点；给一个沿局部法线的安全离面量，避免直接碰撞目标。
            Eigen::Vector3d surface_point(fit_pose[0], fit_pose[1], fit_pose[2]);
            Eigen::Vector3d tcp_position = surface_point + smoothed_normal * surface_clearance - tcp_rotation * probe_offset;
            pose_out[0] = tcp_position(0);
            pose_out[1] = tcp_position(1);
            pose_out[2] = tcp_position(2);

            previous_rotation = scan_rotation;
            previous_smoothed_normal = smoothed_normal;
            has_previous_frame = true;
            previous_tcp_quaternion = current_tcp_quaternion;
            has_previous_tcp_quaternion = true;
            
            Eigen::Matrix<double, 6, 1> pose_vec;
            pose_vec << pose_out[0], pose_out[1], pose_out[2], pose_out[3], pose_out[4], pose_out[5];
            path_poses.push_back(pose_vec);
        }
    }

    // 1) Clamp adjacent dz jumps to reduce sudden lift/drop between points.
    if (max_dz_step > 1e-9) {
        for (size_t i = 1; i < path_poses.size(); ++i) {
            double dz = path_poses[i](2) - path_poses[i - 1](2);
            if (std::abs(dz) > max_dz_step) {
                path_poses[i](2) = path_poses[i - 1](2) + std::copysign(max_dz_step, dz);
            }
        }
    }

    // 2) At zigzag turning points, smooth Z locally to suppress corner lift.
    if (enable_turn_z_smoothing && path_poses.size() >= 3) {
        size_t turn_count = 0;
        for (size_t i = 1; i + 1 < path_poses.size(); ++i) {
            double dy0 = path_poses[i](1) - path_poses[i - 1](1);
            double dy1 = path_poses[i + 1](1) - path_poses[i](1);
            if (std::abs(dy0) > turn_dy_threshold && std::abs(dy1) > turn_dy_threshold && dy0 * dy1 < 0.0) {
                double z_smoothed = (path_poses[i - 1](2) + 2.0 * path_poses[i](2) + path_poses[i + 1](2)) / 4.0;
                path_poses[i](2) = z_smoothed;
                ++turn_count;
            }
        }
        RCLCPP_INFO(get_logger(), "Turn-point Z smoothing applied at %zu corners", turn_count);

        // Re-apply dz clamp after smoothing to keep continuity strict.
        if (max_dz_step > 1e-9) {
            for (size_t i = 1; i < path_poses.size(); ++i) {
                double dz = path_poses[i](2) - path_poses[i - 1](2);
                if (std::abs(dz) > max_dz_step) {
                    path_poses[i](2) = path_poses[i - 1](2) + std::copysign(max_dz_step, dz);
                }
            }
        }
    }

    // 3) Unwrap rotvec representation to avoid +/-pi equivalent jumps in exported file.
    if (path_poses.size() >= 2) {
        const double two_pi = 2.0 * M_PI;
        for (size_t i = 1; i < path_poses.size(); ++i) {
            Eigen::Vector3d prev_rv(path_poses[i - 1](3), path_poses[i - 1](4), path_poses[i - 1](5));
            Eigen::Vector3d curr_rv(path_poses[i](3), path_poses[i](4), path_poses[i](5));

            double angle = curr_rv.norm();
            if (angle < 1e-8) {
                continue;
            }

            Eigen::Vector3d axis = curr_rv / angle;
            std::array<Eigen::Vector3d, 3> candidates = {
                curr_rv,
                curr_rv - two_pi * axis,
                curr_rv + two_pi * axis
            };

            double best_dist = std::numeric_limits<double>::max();
            Eigen::Vector3d best_rv = curr_rv;
            for (const auto &cand : candidates) {
                double d = (cand - prev_rv).norm();
                if (d < best_dist) {
                    best_dist = d;
                    best_rv = cand;
                }
            }

            path_poses[i](3) = best_rv(0);
            path_poses[i](4) = best_rv(1);
            path_poses[i](5) = best_rv(2);
        }
    }

    for (const auto &pose_vec : path_poses) {
        path_file << pose_vec(0) << " " << pose_vec(1) << " " << pose_vec(2) << " "
                  << pose_vec(3) << " " << pose_vec(4) << " " << pose_vec(5) << std::endl;
    }

    path_file.close();

}

void pcl_cloudslam::robot_state_callback(const ur5_msg::msg::RobotState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    latest_robot_state_ = msg;
}

ur5_msg::msg::RobotState::SharedPtr pcl_cloudslam::get_latest_robot_state()
{
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    return latest_robot_state_;
}

bool pcl_cloudslam::get_current_robot_pose(double (&joint_pos)[6], double (&joint_vel)[6], double (&carte_pos)[6])
{
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    if (!latest_robot_state_) {
        return false;
    }
    
    // 使用正确的字段名
    for (int i = 0; i < 6; i++) {
        joint_pos[i] = latest_robot_state_->joint_pos[i];
        joint_vel[i] = latest_robot_state_->joint_vel[i];
        carte_pos[i] = latest_robot_state_->carte_pos[i];
    }
    
    return true;
}

bool pcl_cloudslam::load_camera_transform(const std::string& file_path)
{
    try {
        // 读取JSON文件
        std::ifstream ifs(file_path);
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera transform file: %s", file_path.c_str());
            return false;
        }
        
        // 读取文件内容
        std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        
        // 解析JSON格式
        size_t t_cam2gripper_start = content.find("\"T_cam2gripper\":");
        if (t_cam2gripper_start == std::string::npos) {
            RCLCPP_ERROR(this->get_logger(), "T_cam2gripper section not found in calibration file");
            return false;
        }
        
        // 找到变换矩阵的开始位置
        size_t matrix_start = content.find("[", t_cam2gripper_start);
        if (matrix_start == std::string::npos) {
            RCLCPP_ERROR(this->get_logger(), "Invalid T_cam2gripper format");
            return false;
        }
        
        // 解析4x4变换矩阵
        std::istringstream iss(content.substr(matrix_start));
        char c;
        int row = 0, col = 0;
        
        while (iss >> c && row < 4) {
            if (c == '[') {
                col = 0;
                while (iss >> c && col < 4) {
                    if (c == '-' || std::isdigit(c)) {
                        iss.unget();
                        double value;
                        iss >> value;
                        camera_to_end_effector(row, col) = value;
                        col++;
                    }
                }
                row++;
            }
        }
        
        if (row < 4) {
            RCLCPP_ERROR(this->get_logger(), "Incomplete T_cam2gripper matrix, expected 4 rows, got %d", row);
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Camera transform matrix loaded successfully");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading camera transform: %s", e.what());
        return false;
    }
}
Eigen::Matrix4d pcl_cloudslam::calculate_transform_matrix(const double pose[6])
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    
    // 提取位置信息
    transform(0, 3) = pose[0];  // x
    transform(1, 3) = pose[1];  // y
    transform(2, 3) = pose[2];  // z
    
    // pose[3..5] 优先按 rotation-vector (axis * angle) 解析，若无效则退回为 roll-pitch-yaw (Z-Y-X)
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    Eigen::Vector3d rot_vec(pose[3], pose[4], pose[5]);
    double angle = rot_vec.norm();
    if (std::isfinite(angle) && angle > 1e-8) {
        Eigen::Vector3d axis = rot_vec / angle;
        Eigen::AngleAxisd aa(angle, axis);
        R = aa.toRotationMatrix();
    } else {
        // 回退：按 roll-pitch-yaw (Z-Y-X) 解析
        double roll = pose[3];
        double pitch = pose[4];
        double yaw = pose[5];
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        R = q.toRotationMatrix();
    }

    transform.block<3, 3>(0, 0) = R;
    
    return transform;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto pcl_handle = std::make_shared<pcl_cloudslam>();
    std::thread spin_thread([&pcl_handle](){ rclcpp ::spin(pcl_handle);});
    sleep(3);

    pcl_handle->pcl_filter();
    pcl_handle->roi_range();
    pcl_handle->grid();
    pcl_handle->path_plan();
    
    rclcpp::shutdown();
    return 0;
}
