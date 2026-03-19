#include "robot_admittance_control/AdmittanceController.h"  // 头文件
#include <chrono>   // 时间相关
#include <thread>   // 线程休眠


// 构造函数：初始化所有参数、订阅者、发布者、TF监听器等
AdmittanceController::AdmittanceController(
    rclcpp::Node::SharedPtr node,
    double frequency,
    std::string topic_arm_pose,
    std::string topic_arm_twist,
    std::string topic_external_wrench,
    std::string topic_control_wrench,
    std::string topic_arm_command,
    std::vector<double> M_a,
    std::vector<double> D_a,
    std::vector<double> K_a,   // 新增
    std::vector<double> workspace_limits,
    double arm_max_vel,
    double arm_max_acc)
  : node_(std::move(node)),
    loop_rate_(std::make_shared<rclcpp::Rate>(frequency)),
    M_a_(M_a.data()),
    D_a_(D_a.data()),
    K_a_(K_a.data()),           // 新增
    workspace_limits_(workspace_limits.data()),
    arm_max_vel_(arm_max_vel),
    arm_max_acc_(arm_max_acc)
{
  // 初始化TF监听器
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!node_->has_parameter("base_frame")) {
    node_->declare_parameter("base_frame", "base");
  }
  if (!node_->has_parameter("arm_base_frame")) {
    node_->declare_parameter("arm_base_frame", "base");
  }
  if (!node_->has_parameter("ft_sensor_frame")) {
    node_->declare_parameter("ft_sensor_frame", "sensor_frame");
  }
  if (!node_->has_parameter("control_wrench_frame")) {
    node_->declare_parameter("control_wrench_frame", "base");
  }
  node_->get_parameter("base_frame", base_frame_);
  node_->get_parameter("arm_base_frame", arm_base_frame_);
  node_->get_parameter("ft_sensor_frame", ft_sensor_frame_);
  node_->get_parameter("control_wrench_frame", control_wrench_frame_);

  if (!node_->has_parameter("topic_desired_pose")) {
    node_->declare_parameter("topic_desired_pose", "/desired_pose");
  }
  if (!node_->has_parameter("topic_desired_twist")) {
    node_->declare_parameter("topic_desired_twist", "/desired_twist");
  }
  if (!node_->has_parameter("topic_desired_accel")) {
    node_->declare_parameter("topic_desired_accel", "/desired_accel");
  }
  if (!node_->has_parameter("track_kp")) {
    node_->declare_parameter("track_kp", std::vector<double>{1.0, 1.0, 1.0, 0.3, 0.3, 0.3});
  }
  if (!node_->has_parameter("track_ki")) {
    node_->declare_parameter("track_ki", std::vector<double>{0.05, 0.05, 0.05, 0.0, 0.0, 0.0});
  }
  if (!node_->has_parameter("track_integral_limit")) {
    node_->declare_parameter("track_integral_limit", std::vector<double>{0.10, 0.10, 0.10, 0.20, 0.20, 0.20});
  }
  if (!node_->has_parameter("enable_output_smoothing")) {
    node_->declare_parameter("enable_output_smoothing", true);
  }
  if (!node_->has_parameter("twist_smoothing_alpha_linear")) {
    node_->declare_parameter("twist_smoothing_alpha_linear", 0.25);
  }
  if (!node_->has_parameter("twist_smoothing_alpha_angular")) {
    node_->declare_parameter("twist_smoothing_alpha_angular", 0.20);
  }
  if (!node_->has_parameter("pose_smoothing_alpha_linear")) {
    node_->declare_parameter("pose_smoothing_alpha_linear", 0.25);
  }
  if (!node_->has_parameter("pose_smoothing_alpha_angular")) {
    node_->declare_parameter("pose_smoothing_alpha_angular", 0.20);
  }
  std::string topic_desired_pose;
  std::string topic_desired_twist;
  std::string topic_desired_accel;
  std::vector<double> track_kp;
  std::vector<double> track_ki;
  std::vector<double> track_integral_limit;
  node_->get_parameter("topic_desired_pose", topic_desired_pose);
  node_->get_parameter("topic_desired_twist", topic_desired_twist);
  node_->get_parameter("topic_desired_accel", topic_desired_accel);
  node_->get_parameter("track_kp", track_kp);
  node_->get_parameter("track_ki", track_ki);
  node_->get_parameter("track_integral_limit", track_integral_limit);
  node_->get_parameter("enable_output_smoothing", enable_output_smoothing_);
  node_->get_parameter("twist_smoothing_alpha_linear", twist_smoothing_alpha_linear_);
  node_->get_parameter("twist_smoothing_alpha_angular", twist_smoothing_alpha_angular_);
  node_->get_parameter("pose_smoothing_alpha_linear", pose_smoothing_alpha_linear_);
  node_->get_parameter("pose_smoothing_alpha_angular", pose_smoothing_alpha_angular_);

  // 平滑系数约束到[0,1]：0=完全沿用上一次输出(最平滑但滞后最大)，1=不过滤(最跟手)
  if (twist_smoothing_alpha_linear_ < 0.0) twist_smoothing_alpha_linear_ = 0.0;
  if (twist_smoothing_alpha_linear_ > 1.0) twist_smoothing_alpha_linear_ = 1.0;
  if (twist_smoothing_alpha_angular_ < 0.0) twist_smoothing_alpha_angular_ = 0.0;
  if (twist_smoothing_alpha_angular_ > 1.0) twist_smoothing_alpha_angular_ = 1.0;
  if (pose_smoothing_alpha_linear_ < 0.0) pose_smoothing_alpha_linear_ = 0.0;
  if (pose_smoothing_alpha_linear_ > 1.0) pose_smoothing_alpha_linear_ = 1.0;
  if (pose_smoothing_alpha_angular_ < 0.0) pose_smoothing_alpha_angular_ = 0.0;
  if (pose_smoothing_alpha_angular_ > 1.0) pose_smoothing_alpha_angular_ = 1.0;

  if (track_kp.size() != 6 || track_ki.size() != 6 || track_integral_limit.size() != 6) {
    throw std::runtime_error("track_kp/track_ki/track_integral_limit must be length 6");
  }
  for (int i = 0; i < 6; ++i) {
    track_kp_(i) = track_kp[static_cast<size_t>(i)];
    track_ki_(i) = track_ki[static_cast<size_t>(i)];
    track_integral_limit_(i) = std::abs(track_integral_limit[static_cast<size_t>(i)]);
  }

  // --- 订阅机械臂状态、力传感器等话题 ---
  sub_arm_pose_ = node_->create_subscription<geometry_msgs::msg::Pose>(
    topic_arm_pose,
    rclcpp::QoS(10),
    std::bind(&AdmittanceController::pose_arm_callback, this, std::placeholders::_1));

  std::string topic_arm_pose_arm = topic_arm_pose; // 临时使用与topic_arm_pose相同的数据
  sub_arm_pose_arm_ = node_->create_subscription<geometry_msgs::msg::Pose>(
    topic_arm_pose_arm,
    rclcpp::QoS(10),
    std::bind(&AdmittanceController::pose_arm_arm_callback, this, std::placeholders::_1));

  sub_arm_twist_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    topic_arm_twist,
    rclcpp::QoS(10),
    std::bind(&AdmittanceController::twist_arm_callback, this, std::placeholders::_1));

  sub_desired_pose_ = node_->create_subscription<geometry_msgs::msg::Pose>(
    topic_desired_pose,
    rclcpp::QoS(10),
    std::bind(&AdmittanceController::desired_pose_callback, this, std::placeholders::_1));

  sub_desired_twist_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    topic_desired_twist,
    rclcpp::QoS(10),
    std::bind(&AdmittanceController::desired_twist_callback, this, std::placeholders::_1));

  sub_desired_accel_ = node_->create_subscription<geometry_msgs::msg::Accel>(
    topic_desired_accel,
    rclcpp::QoS(10),
    std::bind(&AdmittanceController::desired_accel_callback, this, std::placeholders::_1));

  sub_wrench_external_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    topic_external_wrench,
    rclcpp::QoS(5),
    std::bind(&AdmittanceController::wrench_external_callback, this, std::placeholders::_1));

  sub_wrench_control_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    topic_control_wrench,
    rclcpp::QoS(5),
    std::bind(&AdmittanceController::wrench_control_callback, this, std::placeholders::_1));

  // --- 发布机械臂速度指令 ---
  pub_arm_cmd_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_arm_command, rclcpp::QoS(5));
  // 新增：发布期望位置（参数化话题名）
  std::string topic_arm_pose_command = "/arm_desired_pose";
  if (!node_->has_parameter("topic_arm_pose_command")) {
    node_->declare_parameter("topic_arm_pose_command", topic_arm_pose_command);
  }
  node_->get_parameter("topic_arm_pose_command", topic_arm_pose_command);
  pub_arm_pose_cmd_ = node_->create_publisher<geometry_msgs::msg::Pose>(topic_arm_pose_command, rclcpp::QoS(5));

  // pub_wrench_external_    = nh_.advertise<geometry_msgs::WrenchStamped>(
  //                            topic_external_wrench_arm_frame, 5);
  // pub_wrench_control_     = nh_.advertise<geometry_msgs::WrenchStamped>(
  //                            topic_control_wrench_arm_frame, 5);

  RCLCPP_INFO(node_->get_logger(), "Arm max vel: %.3f max acc: %.3f", arm_max_vel_, arm_max_acc_);
  RCLCPP_INFO(node_->get_logger(),
    "Output smoothing: enable=%d twist_alpha_linear=%.3f twist_alpha_angular=%.3f pose_alpha_linear=%.3f pose_alpha_angular=%.3f",
    enable_output_smoothing_,
    twist_smoothing_alpha_linear_, twist_smoothing_alpha_angular_,
    pose_smoothing_alpha_linear_, pose_smoothing_alpha_angular_);


  // 初始化各类状态变量
  wrench_external_.setZero();   // 外部力/力矩清零
  wrench_control_.setZero();    // 控制力/力矩清零
  ee_pose_world_.setZero();     // 世界坐标系下末端位姿清零
  ee_twist_world_.setZero();    // 世界坐标系下末端速度清零
  arm_real_position_.setZero(); // 机械臂末端位置清零
  desired_twist_.setZero();
  desired_accel_.setZero();
  desired_position_.setZero();
  desired_orientation_.setIdentity();
  arm_command_position_.setZero();
  arm_command_orientation_.setIdentity();
  arm_command_position_filtered_.setZero();
  arm_command_orientation_filtered_.setIdentity();

  // 等待机械臂状态数据到来
  while (rclcpp::ok() && !arm_real_position_(0)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Waiting for the state of the arm...");
    rclcpp::spin_some(node_);
    loop_rate_->sleep();
  }

  // 初始化积分器和TF标志
  arm_desired_twist_.setZero();
  arm_desired_twist_filtered_.setZero();
  admittance_twist_.setZero();
  admittance_displacement_.setZero();      // 新增
  track_error_integral_.setZero();
  output_smoothing_initialized_ = false;
  pose_smoothing_initialized_ = false;
  ft_arm_ready_ = false;
  arm_world_ready_ = false;
  world_arm_ready_ = false;
  desired_pose_ready_ = false;
  desired_twist_ready_ = false;
  desired_accel_ready_ = false;

  // 等待所有TF变换就绪
  wait_for_transformations();
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
// 控制主循环：不断计算顺应性动力学、限制工作空间并发布指令
void AdmittanceController::run() {
  RCLCPP_INFO(node_->get_logger(), "Running the admittance control loop .................");
  while (rclcpp::ok()) {
    compute_admittance();      // 计算顺应性动力学
    limit_to_workspace();      // 限制速度/位置在工作空间内
    send_commands_to_robot();  // 发布速度指令
    rclcpp::spin_some(node_);  // 处理回调
    loop_rate_->sleep();       // 控制循环频率
  }
}


///////////////////////////////////////////////////////////////
///////////////////// Admittance Dynamics /////////////////////
///////////////////////////////////////////////////////////////
// 顺应性动力学计算：轨迹前馈 + 虚拟位移导纳（x_cmd = x_d + x_a）
void AdmittanceController::compute_admittance()
{
  // 检查是否收到了完整的期望轨迹话题，如果没有则暂时不计算导纳，直接发布当前位姿作为指令，并重置滤波器状态
  if (!(desired_pose_ready_ && desired_twist_ready_ && desired_accel_ready_)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "Waiting desired trajectory topics: pose=%d twist=%d accel=%d",
      desired_pose_ready_, desired_twist_ready_, desired_accel_ready_);
    arm_desired_twist_.setZero();
    arm_desired_twist_filtered_.setZero();
    arm_command_position_ = arm_real_position_;
    arm_command_orientation_ = arm_real_orientation_;
    arm_command_orientation_.normalize();
    pose_smoothing_initialized_ = false;
    output_smoothing_initialized_ = false;
    return;
  }

  // 虚拟导纳：M*xdd_a + D*xd_a + K*x_a = F_ext + F_ctrl
  Vector6d admittance_virtual_acc = M_a_.inverse() * (
      wrench_external_ + wrench_control_
      - D_a_ * admittance_twist_
      - K_a_ * admittance_displacement_);

  const double dt = std::chrono::duration<double>(loop_rate_->period()).count();

  // 限制虚拟导纳加速度，保证系统稳定和安全
  double a_acc_norm = (admittance_virtual_acc.segment(0, 3)).norm();
  if (a_acc_norm > arm_max_acc_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "Admittance generates high arm acceleration! norm: %.4f",
      a_acc_norm);
    admittance_virtual_acc.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // 积分虚拟状态：xdd_a -> xd_a -> x_a
  admittance_twist_ += admittance_virtual_acc * dt;
  admittance_displacement_ += admittance_twist_ * dt;

  // 动力学位置输出：x_cmd = x_d + x_a
  // 其中 x_a(0:3) 是位置虚拟位移，x_a(3:6) 是小旋转向量（轴角）
  const Vector3d raw_arm_command_position = desired_position_ + admittance_displacement_.segment(0, 3);
  const Vector3d delta_rotvec = admittance_displacement_.segment(3, 3);
  const double delta_angle = delta_rotvec.norm();
  Quaterniond delta_q = Quaterniond::Identity();
  if (delta_angle > 1e-12) {
    const Vector3d delta_axis = delta_rotvec / delta_angle;
    delta_q = Quaterniond(Eigen::AngleAxisd(delta_angle, delta_axis));
  }
  Quaterniond raw_arm_command_orientation = delta_q * desired_orientation_;
  raw_arm_command_orientation.normalize();

  apply_pose_smoothing(raw_arm_command_position, raw_arm_command_orientation);

  // 外环跟踪：v_track = Kp*e + Ki*int(e)
  Vector6d tracking_pose_error = Vector6d::Zero();
  tracking_pose_error.segment(0, 3) = desired_position_ - arm_real_position_;
  Eigen::Quaterniond q_err = desired_orientation_ * arm_real_orientation_.conjugate();
  q_err.normalize();
  if (q_err.w() < 0.0) {
    q_err.coeffs() *= -1.0;
  }
  Eigen::AngleAxisd aa(q_err);
  tracking_pose_error.segment(3, 3) = aa.axis() * aa.angle();

  track_error_integral_ += tracking_pose_error * dt;
  for (int i = 0; i < 6; ++i) {
    if (track_error_integral_(i) > track_integral_limit_(i)) {
      track_error_integral_(i) = track_integral_limit_(i);
    } else if (track_error_integral_(i) < -track_integral_limit_(i)) {
      track_error_integral_(i) = -track_integral_limit_(i);
    }
  }
  Vector6d track_twist = track_kp_.cwiseProduct(tracking_pose_error)
                       + track_ki_.cwiseProduct(track_error_integral_);

  // 速度指令：轨迹参考速度 + 虚拟导纳速度 + 跟踪纠偏速度
  // 原始导纳输出（未平滑）：轨迹参考 + 导纳补偿 + 跟踪纠偏
  Vector6d raw_arm_desired_twist = desired_twist_ + admittance_twist_ + track_twist;

  apply_twist_smoothing(raw_arm_desired_twist);

}

void AdmittanceController::apply_twist_smoothing(const Vector6d & raw_twist)
{
  // 一阶低通平滑：
  // y_k = alpha * x_k + (1 - alpha) * y_{k-1}
  // 线速度和角速度使用独立alpha，便于分别平衡“抖动抑制”和“跟手性”
  if (enable_output_smoothing_) {
    if (!output_smoothing_initialized_) {
      // 首次进入滤波时直接对齐，避免从零初始化导致的启动瞬态
      arm_desired_twist_filtered_ = raw_twist;
      output_smoothing_initialized_ = true;
    } else {
      arm_desired_twist_filtered_.segment(0, 3) =
        twist_smoothing_alpha_linear_ * raw_twist.segment(0, 3)
        + (1.0 - twist_smoothing_alpha_linear_) * arm_desired_twist_filtered_.segment(0, 3);
      arm_desired_twist_filtered_.segment(3, 3) =
        twist_smoothing_alpha_angular_ * raw_twist.segment(3, 3)
        + (1.0 - twist_smoothing_alpha_angular_) * arm_desired_twist_filtered_.segment(3, 3);
    }
    arm_desired_twist_ = arm_desired_twist_filtered_;
  } else {
    // 关闭平滑时直通输出，同时同步滤波状态，便于后续无冲击地重新开启
    arm_desired_twist_ = raw_twist;
    arm_desired_twist_filtered_ = raw_twist;
    output_smoothing_initialized_ = true;
  }
}

void AdmittanceController::apply_pose_smoothing(
  const Vector3d & raw_position,
  const Quaterniond & raw_orientation)
{
  if (!enable_output_smoothing_) {
    arm_command_position_ = raw_position;
    arm_command_orientation_ = raw_orientation;
    arm_command_orientation_.normalize();
    arm_command_position_filtered_ = raw_position;
    arm_command_orientation_filtered_ = arm_command_orientation_;
    pose_smoothing_initialized_ = true;
    return;
  }

  if (!pose_smoothing_initialized_) {
    arm_command_position_filtered_ = raw_position;
    arm_command_orientation_filtered_ = raw_orientation;
    arm_command_orientation_filtered_.normalize();
    pose_smoothing_initialized_ = true;
  } else {
    arm_command_position_filtered_ =
      pose_smoothing_alpha_linear_ * raw_position +
      (1.0 - pose_smoothing_alpha_linear_) * arm_command_position_filtered_;

    Quaterniond target_orientation = raw_orientation;
    if (arm_command_orientation_filtered_.dot(target_orientation) < 0.0) {
      target_orientation.coeffs() *= -1.0;
    }
    arm_command_orientation_filtered_ = arm_command_orientation_filtered_.slerp(
      pose_smoothing_alpha_angular_, target_orientation);
    arm_command_orientation_filtered_.normalize();
  }

  arm_command_position_ = arm_command_position_filtered_;
  arm_command_orientation_ = arm_command_orientation_filtered_;
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
// 机械臂末端位姿回调，更新实际位置和姿态
void AdmittanceController::pose_arm_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg) {
  arm_real_position_ << msg->position.x, msg->position.y, msg->position.z;
  arm_real_orientation_.coeffs() << msg->orientation.x,
                               msg->orientation.y,
                               msg->orientation.z,
                               msg->orientation.w;
}

// 机械臂基坐标系下末端位姿回调
void AdmittanceController::pose_arm_arm_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg) {
  arm_real_position_arm_ << msg->position.x, msg->position.y, msg->position.z;
  arm_real_orientation_arm_.coeffs() << msg->orientation.x,
                               msg->orientation.y,
                               msg->orientation.z,
                               msg->orientation.w;
}

// 机械臂末端速度回调，更新速度状态
void AdmittanceController::twist_arm_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg) {
  arm_real_twist_ << msg->linear.x, msg->linear.y,
                  msg->linear.z, msg->angular.x, msg->angular.y,
                  msg->angular.z;
}

void AdmittanceController::desired_pose_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg) {
  desired_position_ << msg->position.x, msg->position.y, msg->position.z;
  desired_orientation_.coeffs() << msg->orientation.x,
                                   msg->orientation.y,
                                   msg->orientation.z,
                                   msg->orientation.w;
  desired_pose_ready_ = true;
}

void AdmittanceController::desired_twist_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg) {
  desired_twist_ << msg->linear.x, msg->linear.y,
                    msg->linear.z, msg->angular.x, msg->angular.y,
                    msg->angular.z;
  desired_twist_ready_ = true;
}

void AdmittanceController::desired_accel_callback(
  const geometry_msgs::msg::Accel::SharedPtr msg) {
  desired_accel_ << msg->linear.x, msg->linear.y,
                    msg->linear.z, msg->angular.x, msg->angular.y,
                    msg->angular.z;
  desired_accel_ready_ = true;
}

// 力/力矩传感器回调，将传感器数据变换到base_link坐标系
void AdmittanceController::wrench_external_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  Vector6d wrench_ft_frame;
  if (ft_arm_ready_) {
    // 读取FT传感器自身坐标系下的力/力矩
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,
                    msg->wrench.force.z, msg->wrench.torque.x,
                    msg->wrench.torque.y, msg->wrench.torque.z;
    const std::string source_frame = msg->header.frame_id.empty() ? ft_sensor_frame_ : msg->header.frame_id;

    // 坐标变换到控制参考坐标系（使用完整6x6伴随矩阵）
    if (source_frame == base_frame_) {
      rotation_tool_.setZero();
      rotation_tool_.topLeftCorner(3, 3) = Matrix3d::Identity();
      rotation_tool_.bottomRightCorner(3, 3) = Matrix3d::Identity();
    } else if (!get_wrench_transform(rotation_tool_, base_frame_, source_frame)) {
      return;
    }
    wrench_external_ <<  rotation_tool_  * wrench_ft_frame;
  }
}

// 控制输入力/力矩回调
void AdmittanceController::wrench_control_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  if (msg->header.frame_id == control_wrench_frame_) {
    wrench_control_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                    msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  }
  else  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "wrench_control_callback: frame_id=%s, expected=%s",
      msg->header.frame_id.c_str(), control_wrench_frame_.c_str());
  }
}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
// 发布速度指令到机械臂
void AdmittanceController::send_commands_to_robot() {
  geometry_msgs::msg::Twist arm_twist_cmd;
  arm_twist_cmd.linear.x  = arm_desired_twist_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_(5);
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 100,
    "Desired linear velocity: %.4f %.4f %.4f",
    arm_twist_cmd.linear.x, arm_twist_cmd.linear.y, arm_twist_cmd.linear.z);
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 100,
    "Desired angular velocity: %.4f %.4f %.4f",
    arm_twist_cmd.angular.x, arm_twist_cmd.angular.y, arm_twist_cmd.angular.z);
  pub_arm_cmd_->publish(arm_twist_cmd);

  // 同步发布动力学计算后的期望位置（用于位置执行链，如 servoL）
  geometry_msgs::msg::Pose arm_pose_cmd;
  arm_pose_cmd.position.x = arm_command_position_(0);
  arm_pose_cmd.position.y = arm_command_position_(1);
  arm_pose_cmd.position.z = arm_command_position_(2);
  arm_pose_cmd.orientation.x = arm_command_orientation_.x();
  arm_pose_cmd.orientation.y = arm_command_orientation_.y();
  arm_pose_cmd.orientation.z = arm_command_orientation_.z();
  arm_pose_cmd.orientation.w = arm_command_orientation_.w();
  pub_arm_pose_cmd_->publish(arm_pose_cmd);
}


// 工作空间限制：限制速度、角速度、末端位置等，防止超出安全范围
void AdmittanceController::limit_to_workspace() {
  double norm_vel_des = (arm_desired_twist_.segment(0, 3)).norm(); // 线速度模
  // 限制最大速度
  if (norm_vel_des > arm_max_vel_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Admittance generate fast arm movements! velocity norm: %.4f", norm_vel_des);
    arm_desired_twist_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);
  }
  // 速度过小直接归零，抑制抖动（用绝对值）
  if (norm_vel_des < 1e-5)
    arm_desired_twist_.segment(0,3).setZero();
  // 限制角速度过小归零（用绝对值）
  if (std::abs(arm_desired_twist_(3)) < 1e-5)
      arm_desired_twist_(3) = 0;
  if (std::abs(arm_desired_twist_(4)) < 1e-5)
      arm_desired_twist_(4) = 0;
  if (std::abs(arm_desired_twist_(5)) < 1e-5)
      arm_desired_twist_(5) = 0;    
  // 限制角速度最大值
  if (arm_desired_twist_(3) > 10.0)
      arm_desired_twist_(3) = 10.0;
  else if (arm_desired_twist_(3) < -10.0)
      arm_desired_twist_(3) = -10.0;
  if (arm_desired_twist_(4) > 10.0)
      arm_desired_twist_(4) = 10.0;
  else if (arm_desired_twist_(4) < -10.0)
      arm_desired_twist_(4) = -10.0;
  if (arm_desired_twist_(5) > 10.0)
      arm_desired_twist_(5) = 10.0;    
  else if (arm_desired_twist_(5) < -10.0)
      arm_desired_twist_(5) = -10.0;
  // 工作空间边界限制
  double ee_base_norm   = (arm_real_position_arm_).norm();
  double rec_operating_limit = 1.15; // 真实机器人
  double dist_limit = rec_operating_limit - ee_base_norm; 
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 100,
    "||x_ee-w_limit||: %.4f", dist_limit);
  if (ee_base_norm >= rec_operating_limit){
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 100,
      "Out of operational workspace limit!");
    base_position_ << 0.33000, 0.00000, 0.48600;
    Vector3d repulsive_field = -(arm_real_position_- base_position_);
    arm_desired_twist_.segment(0,3) = 0.05*(repulsive_field/ee_base_norm);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 100,
      "Bringing robot back slowly with uniform repulsive velocity field!");
  }
  // Z轴下限保护
  if (arm_real_position_(2) < workspace_limits_(4))
      arm_desired_twist_(2) += 0.3;
  // 工作空间连续调制
  double workspace_fct = dist_limit;
  if (dist_limit > 0.05)
      workspace_fct = 1;
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 100,
    "Workspace Scaling function: %.4f", workspace_fct);
}


//////////////////////
/// INITIALIZATION ///
//////////////////////
// 等待所有TF变换就绪，保证后续坐标变换正确
void AdmittanceController::wait_for_transformations() {
  Matrix6d rot_matrix;
  rotation_base_.setZero();
  rotation_tool_.setZero();

  if (base_frame_ == arm_base_frame_) {
    rotation_base_.topLeftCorner(3, 3) = Matrix3d::Identity();
    rotation_base_.bottomRightCorner(3, 3) = Matrix3d::Identity();
    base_position_.setZero();
    arm_world_ready_ = true;
    world_arm_ready_ = true;
  } else {
    // 等待base_frame到arm_base_frame的TF
    while (rclcpp::ok() && !get_rotation_matrix(rotation_base_, base_frame_, arm_base_frame_, 1)) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    arm_world_ready_ = true;

    // 等待arm_base_frame到base_frame的TF
    while (rclcpp::ok() && !get_rotation_matrix(rot_matrix, arm_base_frame_, base_frame_, 0)) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    world_arm_ready_ = true;
  }

  if (ft_sensor_frame_ == base_frame_) {
    rotation_tool_.topLeftCorner(3, 3) = Matrix3d::Identity();
    rotation_tool_.bottomRightCorner(3, 3) = Matrix3d::Identity();
    ft_arm_ready_ = true;
  } else {
    // 力传感器坐标系允许后续在回调中动态等待，不阻塞整个控制器启动
    ft_arm_ready_ = true;
    RCLCPP_WARN(node_->get_logger(),
      "FT frame (%s) differs from base frame (%s). Will transform at runtime.",
      ft_sensor_frame_.c_str(), base_frame_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "The Force/Torque sensor is ready to use.");
}




////////////
/// UTIL ///
////////////

// 获取坐标变换矩阵（6x6），可选获取平移量
bool AdmittanceController::get_rotation_matrix(Matrix6d & rotation_matrix,
    std::string from_frame,
    std::string to_frame, bool getT) {
  Matrix3d rotation_from_to;
  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);
    const auto &q = transform.transform.rotation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    rotation_from_to = quat.toRotationMatrix();
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
    if (getT==1)
      base_position_ << transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z;
  }
  catch (const tf2::TransformException & ex) {
    rotation_matrix.setZero();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Waiting for TF from: %s to: %s (%s)", from_frame.c_str(), to_frame.c_str(), ex.what());
    return false;
  }
  return true;
}

// 新增：获取wrench变换的完整6x6伴随矩阵（含平移耦合）
bool AdmittanceController::get_wrench_transform(Matrix6d & wrench_transform,
    const std::string & from_frame,
    const std::string & to_frame) {
  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);

    const auto &q = transform.transform.rotation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    Matrix3d R = quat.toRotationMatrix();

    const double px = transform.transform.translation.x;
    const double py = transform.transform.translation.y;
    const double pz = transform.transform.translation.z;

    Matrix3d p_hat;
    p_hat <<     0, -pz,  py,
              pz,   0, -px,
             -py,  px,   0;

    wrench_transform.setZero();
    wrench_transform.topLeftCorner(3, 3) = R;
    wrench_transform.bottomRightCorner(3, 3) = R;
    wrench_transform.bottomLeftCorner(3, 3) = p_hat * R;

    return true;
  } catch (const tf2::TransformException & ex) {
    wrench_transform.setZero();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Waiting for TF from: %s to: %s (%s)", from_frame.c_str(), to_frame.c_str(), ex.what());
    return false;
  }
}
