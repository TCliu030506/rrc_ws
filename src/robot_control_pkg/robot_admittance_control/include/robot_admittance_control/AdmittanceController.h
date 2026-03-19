#ifndef ADMITTANCECONTROLLER_H   // 防止头文件被重复包含
#define ADMITTANCECONTROLLER_H

#include <rclcpp/rclcpp.hpp>

// #include "cartesian_state_msgs/PoseTwist.h"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include <std_msgs/msg/float32.hpp>


//
// AdmittanceController 类 //
//
// 该类实现了Ridgeback+UR5平台的顺应性控制器。
// 控制器主循环：
// 1) 读取机器人状态（机械臂+平台）
// 2) 计算并发送期望的速度指令到机械臂
//
// 通信接口：
// - 通过ROS话题发送期望速度（Twist）
// - 低层控制器由ROS control实现
//
// 说明：
// 该节点本应作为ROS控制器实现，但受限于当前ROS control实现，采用节点方式。
//
// 用法示例见下方注释。
// AdmittanceController admittance_controller(nh, frequency,
//                                           cmd_topic_platform,
//                                           state_topic_platform,
//                                           cmd_topic_arm,
//                                           topic_arm_twist_world,
//                                           topic_wrench_u_e,
//                                           topic_wrench_u_c,
//                                           state_topic_arm,
//                                           wrench_topic,
//                                           wrench_control_topic,
//                                           laser_front_topic,
//                                           laser_rear_topic,
//                                           M_p, M_a, D, D_p, D_a, K, d_e,
//                                           wrench_filter_factor,
//                                           force_dead_zone_thres,
//                                           torque_dead_zone_thres,
//                                           obs_distance_thres,
//                                           self_detect_thres);
// admittance_controller.run();

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;  //使用typedef为现有类型创建别名，定义易于记忆的类型名
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;


class AdmittanceController
{
public:
  AdmittanceController(
    rclcpp::Node::SharedPtr node,
    double frequency,
    std::string topic_arm_pose,
    std::string topic_arm_twist,
    std::string topic_external_wrench,
    std::string topic_control_wrench,
    std::string topic_arm_command,
    std::vector<double> M_a,
    std::vector<double> D_a,
    std::vector<double> K_a,               // 新增
    std::vector<double> workspace_limits,
    double arm_max_vel,
    double arm_max_acc);

protected:
  // --- ROS 相关变量 ---
  rclcpp::Node::SharedPtr node_;  // ROS2节点句柄
  std::shared_ptr<rclcpp::Rate> loop_rate_;  // 控制循环频率

  // --- 订阅者 ---
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_arm_pose_;      // 机械臂末端位姿订阅
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_arm_pose_arm_;  // 机械臂基坐标系下末端位姿
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_arm_twist_;    // 机械臂末端速度订阅
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_desired_pose_;   // 期望位姿订阅
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_desired_twist_; // 期望速度订阅
  rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr sub_desired_accel_; // 期望加速度订阅
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_external_; // 力/力矩传感器订阅
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_control_;  // 控制输入力/力矩订阅

  // --- 发布者 ---
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm_cmd_; // 机械臂速度指令发布
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_arm_pose_cmd_; // 机械臂位置指令发布

  // --- 输入信号 ---
  Vector6d wrench_external_; // 外部力/力矩（传感器测量，robotiq_force_torque_frame_id坐标系）
  Vector6d wrench_control_;  // 控制器输出力/力矩（ur5_arm_base_link坐标系）
  Vector6d desired_twist_;   // 轨迹规划器输出的期望速度
  Vector6d desired_accel_;   // 轨迹规划器输出的期望加速度
  Vector3d desired_position_;// 轨迹规划器输出的期望位置
  Quaterniond desired_orientation_; // 轨迹规划器输出的期望姿态

  // --- 顺应性参数 ---
  Matrix6d M_a_, D_a_, K_a_; // 机械臂期望质量、阻尼矩阵、刚度矩阵（新增）

  // --- 输出指令 ---
  Vector6d arm_desired_twist_; // 机械臂期望速度（Twist）
  Vector6d arm_desired_twist_filtered_;     // 平滑后的机械臂期望速度
  Vector3d arm_command_position_;           // 动力学计算后的期望位置 x_cmd
  Quaterniond arm_command_orientation_;     // 动力学计算后的期望姿态 q_cmd
  Vector3d arm_command_position_filtered_;       // 平滑后的期望位置
  Quaterniond arm_command_orientation_filtered_; // 平滑后的期望姿态
  Vector6d admittance_twist_;              // 虚拟速度 x_dot_a（6维）
  Vector6d admittance_displacement_;       // 虚拟位移 x_a（6维）
  Vector6d track_error_integral_;          // 轨迹误差积分
  Vector6d track_kp_;                      // 轨迹误差比例增益
  Vector6d track_ki_;                      // 轨迹误差积分增益
  Vector6d track_integral_limit_;          // 轨迹误差积分限幅
  bool enable_output_smoothing_{true};     // 是否启用导纳输出速度平滑
  double twist_smoothing_alpha_linear_{0.25};    // 线速度低通系数[0,1]
  double twist_smoothing_alpha_angular_{0.20};   // 角速度低通系数[0,1]
  double pose_smoothing_alpha_linear_{0.25};     // 位置低通系数[0,1]
  double pose_smoothing_alpha_angular_{0.20};    // 姿态slerp系数[0,1]
  bool output_smoothing_initialized_{false};      // 滤波状态是否已初始化
  bool pose_smoothing_initialized_{false};        // 位姿滤波状态是否已初始化

  // --- 工作空间限制 ---
  Vector6d workspace_limits_; // 工作空间边界
  double arm_max_vel_;        // 最大速度
  double arm_max_acc_;        // 最大加速度

  // --- 状态变量 ---
  Vector3d     arm_real_position_;        // 机械臂末端实际位置
  Quaterniond  arm_real_orientation_;     // 机械臂末端实际姿态
  Vector6d     arm_real_twist_;           // 机械臂末端实际速度

  Vector3d     arm_real_position_arm_;    // 机械臂基坐标系下末端位置
  Quaterniond  arm_real_orientation_arm_; // 机械臂基坐标系下末端姿态
  Vector3d     base_position_;            // 基座位置

  Vector7d     ee_pose_world_;            // 世界坐标系下末端位姿
  Vector6d     ee_twist_world_;           // 世界坐标系下末端速度

  // --- 坐标变换 ---
  Matrix6d rotation_base_; // base_link到world的变换
  Matrix6d rotation_tool_; // 工具坐标系变换

  // --- 参考坐标系参数 ---
  std::string base_frame_;           // 控制参考基坐标系
  std::string arm_base_frame_;       // 机械臂基坐标系
  std::string ft_sensor_frame_;      // 力传感器坐标系
  std::string control_wrench_frame_; // 控制输入力坐标系

  // --- TF监听 ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;              // TF缓存
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF监听器

  // --- 状态标志 ---
  bool ft_arm_ready_;     // 力传感器TF就绪
  bool arm_world_ready_;  // 机械臂到世界TF就绪
  bool world_arm_ready_;  // 世界到机械臂TF就绪
  bool desired_pose_ready_;
  bool desired_twist_ready_;
  bool desired_accel_ready_;

  // --- 初始化相关方法 ---
  void wait_for_transformations(); // 等待TF变换全部就绪

  // --- 控制主流程 ---
  void compute_admittance();       // 顺应性动力学计算
  void apply_pose_smoothing(const Vector3d & raw_position, const Quaterniond & raw_orientation);
  void apply_twist_smoothing(const Vector6d & raw_twist);

  // --- 回调函数 ---
  void pose_arm_callback(const geometry_msgs::msg::Pose::SharedPtr msg);         // 机械臂末端位姿回调
  void pose_arm_arm_callback(const geometry_msgs::msg::Pose::SharedPtr msg);     // 基坐标系下末端位姿回调
  void twist_arm_callback(const geometry_msgs::msg::Twist::SharedPtr msg);       // 末端速度回调
  void desired_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void desired_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void desired_accel_callback(const geometry_msgs::msg::Accel::SharedPtr msg);
  void wrench_external_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg); // 力/力矩传感器回调
  void wrench_control_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);  // 控制输入回调

  // --- 工具函数 ---
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
      std::string from_frame,
      std::string to_frame, bool getT);

  // 新增：获取wrench的完整6x6伴随矩阵（含平移）
  bool get_wrench_transform(Matrix6d & wrench_transform,
      const std::string & from_frame,
      const std::string & to_frame);

  void limit_to_workspace();      // 工作空间限制
  void send_commands_to_robot();  // 发布速度指令到机械臂

public:
  void run(); // 控制主循环
};

#endif // ADMITTANCECONTROLLER_H

