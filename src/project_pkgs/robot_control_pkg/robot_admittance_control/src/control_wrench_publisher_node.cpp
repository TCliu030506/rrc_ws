#include <chrono>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class ControlWrenchPublisherNode : public rclcpp::Node
{
public:
  ControlWrenchPublisherNode()
  : Node("control_wrench_publisher_node")
  {
    declare_parameter<std::string>("topic_control_wrench", "/arm_admittance_control/control_wrench");
    declare_parameter<std::string>("frame_id", "arm_base_link");
    declare_parameter<double>("publish_rate", 100.0);

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
    RCLCPP_INFO(get_logger(), "Default control wrench is all zeros. Use ROS params to adjust force/torque.");
  }

private:
  void publish_wrench()
  {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.wrench.force.x = get_parameter("force_x").as_double();
    msg.wrench.force.y = get_parameter("force_y").as_double();
    msg.wrench.force.z = get_parameter("force_z").as_double();
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlWrenchPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
