#include <chrono>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class ControlWrenchPublisherNode : public rclcpp::Node
{
public:
  ControlWrenchPublisherNode()
  : Node("control_wrench_publisher_node"), start_time_(now()), last_phase_(-1)
  {
    declare_parameter<std::string>("topic_control_wrench", "/arm_admittance_control/control_wrench");
    declare_parameter<std::string>("frame_id", "asm_ee_site");
    declare_parameter<double>("publish_rate", 100.0);

    // 正弦力参数（三个轴共用）
    declare_parameter<double>("sin_amp", 10.0);       // 振幅 (N)
    declare_parameter<double>("sin_freq", 0.1);      // 频率 (Hz)

    // 时间分段配置
    declare_parameter<double>("initial_delay", 5.0);   // 初始静态时间 (s)
    declare_parameter<double>("phase_duration", 10.0); // 每个正弦阶段时长 (s)

    // 静态力参数（用于非正弦阶段）
    declare_parameter<double>("force_x", 0.0);
    declare_parameter<double>("force_y", 0.0);
    declare_parameter<double>("force_z", 0.0);
    declare_parameter<double>("torque_x", 0.0);
    declare_parameter<double>("torque_y", 0.0);
    declare_parameter<double>("torque_z", 0.0);

    topic_control_wrench_ = get_parameter("topic_control_wrench").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    double publish_rate = get_parameter("publish_rate").as_double();
    if (publish_rate <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid publish_rate %.3f, fallback to 100Hz", publish_rate);
      publish_rate = 100.0;
    }

    pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(topic_control_wrench_, rclcpp::QoS(10));

    const auto period_ns = static_cast<int64_t>(1e9 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::nanoseconds(period_ns),
      std::bind(&ControlWrenchPublisherNode::publish_wrench, this));

    RCLCPP_INFO(
      get_logger(),
      "Control wrench publisher started. topic=%s frame_id=%s rate=%.1fHz",
      topic_control_wrench_.c_str(), frame_id_.c_str(), publish_rate);
    RCLCPP_INFO(get_logger(), "Phase sequence: Static(0-5s) -> X-sine -> Y-sine -> Z-sine (repeat)");
  }

private:
  void publish_wrench()
  {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    // 计算当前时间（秒）
    double elapsed = (now() - start_time_).seconds();
    double initial_delay = get_parameter("initial_delay").as_double();
    double phase_duration = get_parameter("phase_duration").as_double();
    double sin_amp = get_parameter("sin_amp").as_double();
    double sin_freq = get_parameter("sin_freq").as_double();

    double fx = 0.0, fy = 0.0, fz = 0.0;
    int current_phase = -1;
    std::string phase_name = "Static";

    // 判断当前阶段
    if (elapsed < initial_delay) {
      // 阶段0: 0-5s 静态力（全为0）
      current_phase = 0;
      fx = get_parameter("force_x").as_double();
      fy = get_parameter("force_y").as_double();
      fz = get_parameter("force_z").as_double();
    } else {
      // 计算正弦阶段（从5s开始，每10s一个阶段，循环）
      double phase_time = elapsed - initial_delay;
      int phase_index = static_cast<int>(phase_time / phase_duration) % 3;
      
      // 计算当前阶段内的相对时间（用于正弦计算）
      double phase_elapsed = phase_time - phase_index * phase_duration;
      double sin_value = sin_amp * std::sin(2.0 * M_PI * sin_freq * phase_elapsed);

      switch (phase_index) {
        case 0:
          // 阶段1: X轴正弦
          current_phase = 1;
          phase_name = "X-sine";
          fx = sin_value;
          break;
        case 1:
          // 阶段2: Y轴正弦
          current_phase = 2;
          phase_name = "Y-sine";
          fy = sin_value;
          break;
        case 2:
          // 阶段3: Z轴正弦
          current_phase = 3;
          phase_name = "Z-sine";
          fz = sin_value;
          break;
      }
    }

    // 输出阶段切换信息
    if (current_phase != last_phase_) {
      RCLCPP_INFO(get_logger(), "Phase %d: %s (elapsed=%.1fs)", current_phase, phase_name.c_str(), elapsed);
      last_phase_ = current_phase;
    }

    msg.wrench.force.x = fx;
    msg.wrench.force.y = fy;
    msg.wrench.force.z = fz;
    msg.wrench.torque.x = get_parameter("torque_x").as_double();
    msg.wrench.torque.y = get_parameter("torque_y").as_double();
    msg.wrench.torque.z = get_parameter("torque_z").as_double();

    pub_->publish(msg);
  }

private:
  std::string topic_control_wrench_;
  std::string frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  int last_phase_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlWrenchPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}