#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ur5_control_cpp/urscript.h"

class TwistToUrscriptSpeedlNode : public rclcpp::Node
{
public:
  TwistToUrscriptSpeedlNode()
  : Node("twist_to_urscript_speedl_node"),
    last_msg_time_(this->now())
  {
    declare_parameter<std::string>("topic_arm_command", "/UR5/desired_twist");
    declare_parameter<double>("speedl_acc", 0.5);
    declare_parameter<double>("speedl_time", 0.08);
    declare_parameter<double>("publish_rate", 125.0);
    declare_parameter<double>("command_timeout", 0.2);
    declare_parameter<bool>("zero_on_timeout", true);
    declare_parameter<bool>("send_stop_on_exit", true);
    declare_parameter<double>("max_linear_speed", 0.5);
    declare_parameter<double>("max_angular_speed", 0.6);
    declare_parameter<bool>("skip_repeated_zero_command", true);
    declare_parameter<double>("zero_command_epsilon", 1e-6);

    topic_arm_command_ = get_parameter("topic_arm_command").as_string();
    speedl_acc_ = get_parameter("speedl_acc").as_double();
    speedl_time_ = get_parameter("speedl_time").as_double();
    publish_rate_ = get_parameter("publish_rate").as_double();
    command_timeout_ = get_parameter("command_timeout").as_double();
    zero_on_timeout_ = get_parameter("zero_on_timeout").as_bool();
    send_stop_on_exit_ = get_parameter("send_stop_on_exit").as_bool();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    skip_repeated_zero_command_ = get_parameter("skip_repeated_zero_command").as_bool();
    zero_command_epsilon_ = get_parameter("zero_command_epsilon").as_double();

    if (publish_rate_ <= 0.0) {
      publish_rate_ = 125.0;
    }

    script_pub_ = std::make_shared<ur_script::script_pub>();

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      topic_arm_command_, rclcpp::QoS(10),
      std::bind(&TwistToUrscriptSpeedlNode::twist_callback, this, std::placeholders::_1));

    const auto period_ns = static_cast<int64_t>(1e9 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::nanoseconds(period_ns),
      std::bind(&TwistToUrscriptSpeedlNode::publish_speedl, this));

    RCLCPP_INFO(
      get_logger(),
      "twist->URScript bridge started. topic=%s rate=%.1fHz acc=%.3f t=%.3f",
      topic_arm_command_.c_str(), publish_rate_, speedl_acc_, speedl_time_);
  }

  ~TwistToUrscriptSpeedlNode() override
  {
    if (send_stop_on_exit_ && script_pub_) {
      script_pub_->urscript_publish_stopl(std::max(0.1, speedl_acc_));
    }
  }

private:
  static double clamp_abs(double value, double limit)
  {
    if (limit <= 0.0) {
      return value;
    }
    return std::clamp(value, -limit, limit);
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_twist_ = *msg;
    last_msg_time_ = now();
    got_twist_ = true;
  }

  void publish_speedl()
  {
    if (!script_pub_) {
      return;
    }

    geometry_msgs::msg::Twist cmd{};
    if (got_twist_) {
      const double dt = (now() - last_msg_time_).seconds();
      if (dt > command_timeout_ && zero_on_timeout_) {
        cmd = geometry_msgs::msg::Twist();
      } else {
        cmd = latest_twist_;
      }
    }

    std::array<double, 6> vel = {
      clamp_abs(static_cast<double>(cmd.linear.x), max_linear_speed_),
      clamp_abs(static_cast<double>(cmd.linear.y), max_linear_speed_),
      clamp_abs(static_cast<double>(cmd.linear.z), max_linear_speed_),
      clamp_abs(static_cast<double>(cmd.angular.x), max_angular_speed_),
      clamp_abs(static_cast<double>(cmd.angular.y), max_angular_speed_),
      clamp_abs(static_cast<double>(cmd.angular.z), max_angular_speed_)};

    double speedl_vel[6] = {
      vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]
    };

    const bool is_zero_cmd =
      (std::fabs(speedl_vel[0]) <= zero_command_epsilon_) &&
      (std::fabs(speedl_vel[1]) <= zero_command_epsilon_) &&
      (std::fabs(speedl_vel[2]) <= zero_command_epsilon_) &&
      (std::fabs(speedl_vel[3]) <= zero_command_epsilon_) &&
      (std::fabs(speedl_vel[4]) <= zero_command_epsilon_) &&
      (std::fabs(speedl_vel[5]) <= zero_command_epsilon_);

    if (skip_repeated_zero_command_ && is_zero_cmd && zero_command_latched_) {
      return;
    }

    script_pub_->urscript_publish_speedl(speedl_vel, speedl_acc_, speedl_time_);

    if (skip_repeated_zero_command_) {
      zero_command_latched_ = is_zero_cmd;
    }
  }

private:
  std::string topic_arm_command_;
  double speedl_acc_{0.5};
  double speedl_time_{0.08};
  double publish_rate_{125.0};
  double command_timeout_{0.2};
  bool zero_on_timeout_{true};
  bool send_stop_on_exit_{true};
  double max_linear_speed_{0.5};
  double max_angular_speed_{0.6};
  bool skip_repeated_zero_command_{true};
  double zero_command_epsilon_{1e-6};

  bool got_twist_{false};
  bool zero_command_latched_{false};
  geometry_msgs::msg::Twist latest_twist_{};
  rclcpp::Time last_msg_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ur_script::script_pub> script_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToUrscriptSpeedlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
