#include <rclcpp/rclcpp.hpp>
#include "ur5_msg/msg/robot_state.hpp" 
#include <mutex>

#include <iostream>
#include <vector>
#include <string>
#include <thread>

#include "sensor_msgs/msg/point_cloud2.hpp"
// #include "puncture_model/puncture_frame.h"
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
//cluster
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>


class pcl_cloudslam: public rclcpp::Node
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
    std::string file_path;
    std::string plan_path;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    bool is_record;
    bool is_areapicked;
    double pose_6_b[6];
    Eigen::Vector2d roi_range_x;
    Eigen::Vector2d roi_range_y;
    std::vector<Eigen::Vector2d> grid_xy;
    std::vector<Eigen::Vector3d> points_arround;

    // 机器人状态订阅
    rclcpp::Subscription<ur5_msg::msg::RobotState>::SharedPtr robot_state_sub_;
    ur5_msg::msg::RobotState::SharedPtr latest_robot_state_;
    std::mutex robot_state_mutex_;

    // 机器人状态回调函数
    void robot_state_callback(const ur5_msg::msg::RobotState::SharedPtr msg);
    // 相机到末端的变换矩阵
    Eigen::Matrix4d camera_to_end_effector;
    // 探头相对于末端执行器的偏移量（单位：米）
    Eigen::Vector3d probe_offset;
    Eigen::Matrix4d T_base_from_workpiece_;
    Eigen::Matrix4d T_workpiece_from_base_;
    Eigen::Vector3d workpiece_origin_in_base_;
    Eigen::Vector3d workpiece_x_axis_in_base_;
    Eigen::Vector3d workpiece_y_axis_in_base_;
    Eigen::Vector3d workpiece_z_axis_in_base_;
    Eigen::Vector3d last_platform_normal_in_base_;
    bool has_workpiece_frame_ = false;
    bool has_platform_normal_ = false;

public:
    pcl_cloudslam();
    ~pcl_cloudslam();

    bool pcl_filter();
    bool refine_roi_by_platform_and_normal();
    bool fit_local_workpiece_frame_from_refined_roi();
    bool transform_refined_roi_to_workpiece_frame();
    bool path_plan_concave();
    void roi_range(); 
    void grid();
    void polyfit(Eigen::Vector2d point_xy, double (&pose_out)[6]);
    void pose_transform();
    void path_plan();
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void areapickingcallback(const pcl::visualization::AreaPickingEvent &event, void *userdata);
        // ===== 新增：对外提供机械臂状态接口 =====
    /**
     * @brief 获取最新接收到的机器人状态（线程安全）
     * @return 共享指针，若从未收到消息则为 nullptr
     */
    ur5_msg::msg::RobotState::SharedPtr get_latest_robot_state();

    /**
     * @brief 获取当前关节位置、速度及末端位姿（数组形式）
     * @param joint_pos 输出关节位置数组（长度6）
     * @param joint_vel 输出关节速度数组（长度6）
     * @param carte_pos 输出末端笛卡尔位姿数组（前3平移，后3轴角，长度6）
     * @return true 表示成功获取，false 表示尚未收到消息
     */
    bool get_current_robot_pose(double (&joint_pos)[6], double (&joint_vel)[6], double (&carte_pos)[6]);
    /**
     * @brief 根据末端位姿计算变换矩阵
     * @param pose 末端位姿 [x, y, z, roll, pitch, yaw]
     * @return 4x4 齐次变换矩阵
     */
    Eigen::Matrix4d calculate_transform_matrix(const double pose[6]);
    /**
     * @brief 加载相机到末端的变换矩阵
     * @param json_path JSON文件路径
     * @return true 表示成功加载，false 表示失败
     */
    bool load_camera_transform(const std::string& json_path);
};
