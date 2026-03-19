#include <rclcpp/rclcpp.hpp>  // ROS2 C++客户端库
#include "robot_admittance_control/AdmittanceController.h"  // 导入顺应性控制器头文件


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  // 初始化ROS2

  // 创建节点对象
  auto node = std::make_shared<rclcpp::Node>("admittance_controller_node");
  double frequency = 2000.0;  // 控制循环频率（Hz）

  // 声明参数变量
  std::string topic_arm_pose;
  std::string topic_arm_twist;
  std::string topic_arm_command;
  std::string topic_external_wrench;
  std::string topic_control_wrench;
  std::string topic_desired_pose;
  std::string topic_desired_twist;
  std::string topic_desired_accel;
  std::string topic_ds_velocity;
  std::string topic_external_wrench_arm_frame;
  std::string topic_control_external_arm_frame;
  std::string topic_admittance_ratio;

  std::vector<double> M_a;            // 机械臂虚拟质量矩阵
  std::vector<double> D_a;            // 机械臂虚拟阻尼矩阵
  std::vector<double> K_a;            // 机械臂虚拟刚度矩阵（新增）
  std::vector<double> workspace_limits; // 工作空间边界

  double arm_max_vel;  // 最大速度
  double arm_max_acc;  // 最大加速度

  // 声明参数（可通过launch或yaml文件传入）
  node->declare_parameter("topic_arm_pose", "/UR10arm/ee_pose");
  node->declare_parameter("topic_arm_twist", "/UR10arm/ee_twist");
  node->declare_parameter("topic_external_wrench", "/robotiq_force_torque_wrench");
  node->declare_parameter("topic_control_wrench", "/arm_admittance_control/control_wrench");
  node->declare_parameter("topic_arm_command", "/UR10arm/desired_twist");
  node->declare_parameter("topic_desired_pose", "/desired_pose");
  node->declare_parameter("topic_desired_twist", "/desired_twist");
  node->declare_parameter("topic_desired_accel", "/desired_accel");
  node->declare_parameter("base_frame", "base");
  node->declare_parameter("arm_base_frame", "base");
  node->declare_parameter("ft_sensor_frame", "sensor_frame");
  node->declare_parameter("control_wrench_frame", "arm_base_link");
  node->declare_parameter("mass_arm", std::vector<double>{});
  node->declare_parameter("damping_arm", std::vector<double>{});
  node->declare_parameter("stiffness_arm", std::vector<double>{});  // 新增
  node->declare_parameter("workspace_limits", std::vector<double>{});
  node->declare_parameter("arm_max_vel", 0.5);
  node->declare_parameter("arm_max_acc", 1.0);

  // ========== 从参数服务器加载参数 ==========

  // 话题名参数
  node->get_parameter("topic_arm_pose", topic_arm_pose);
  node->get_parameter("topic_arm_twist", topic_arm_twist);
  node->get_parameter("topic_external_wrench", topic_external_wrench);
  node->get_parameter("topic_control_wrench", topic_control_wrench);
  node->get_parameter("topic_arm_command", topic_arm_command);
  node->get_parameter("topic_desired_pose", topic_desired_pose);
  node->get_parameter("topic_desired_twist", topic_desired_twist);
  node->get_parameter("topic_desired_accel", topic_desired_accel);

  // 顺应性参数
  node->get_parameter("mass_arm", M_a);
  if (M_a.size() != 36) {
    RCLCPP_ERROR(node->get_logger(), "Parameter mass_arm必须为36维（6x6矩阵展平）");
    return -1;
  }

  node->get_parameter("damping_arm", D_a);
  if (D_a.size() != 36) {
    RCLCPP_ERROR(node->get_logger(), "Parameter damping_arm必须为36维（6x6矩阵展平）");
    return -1;
  }

  node->get_parameter("stiffness_arm", K_a);  // 新增
  if (K_a.size() != 36) {                     // 新增
    RCLCPP_ERROR(node->get_logger(), "Parameter stiffness_arm必须为36维（6x6矩阵展平）");
    return -1;
  }


  // 安全参数
  node->get_parameter("workspace_limits", workspace_limits);
  if (workspace_limits.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Parameter workspace_limits必须为6维（x/y/z上下限）");
    return -1;
  }

  node->get_parameter("arm_max_vel", arm_max_vel);
  node->get_parameter("arm_max_acc", arm_max_acc);

  // ========== 构造顺应性控制器对象 ==========
  AdmittanceController admittance_controller(
    node,
    frequency,
    topic_arm_pose,
    topic_arm_twist,
    topic_external_wrench,
    topic_control_wrench,
    topic_arm_command,
    M_a, D_a, K_a,    // 这里新增 K_a
    workspace_limits,
    arm_max_vel, arm_max_acc);

  // ========== 启动控制主循环 ==========
  admittance_controller.run();

  rclcpp::shutdown();  // 关闭ROS2
  return 0;
}
