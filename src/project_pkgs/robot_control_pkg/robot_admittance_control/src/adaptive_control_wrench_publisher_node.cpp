#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ControlWrenchPublisherNode : public rclcpp::Node
{
public:
  enum class ForceState
  {
    FREE_SPACE = 0,
    CONTACT_TRANSITION = 1,
    FORCE_TRACKING = 2,
  };

  ControlWrenchPublisherNode()
  : Node("control_wrench_publisher_node")
  {
    declare_parameter<std::string>("topic_control_wrench", "/arm_admittance_control/control_wrench");
    declare_parameter<std::string>("topic_compensated_wrench", "/wrench_compensated");
    declare_parameter<std::string>("frame_id", "asm_ee_site");
    declare_parameter<double>("publish_rate", 100.0);
    declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
    declare_parameter<std::string>("normal_axis", "z");
    declare_parameter<double>("normal_axis_sign", 1.0);

    declare_parameter<double>("contact_force_threshold", 2.0);
    declare_parameter<double>("release_force_threshold", 1.0);
    declare_parameter<double>("force_target", 10.0);
    declare_parameter<double>("force_rate_limit", 20.0);
    declare_parameter<double>("force_tracking_epsilon", 0.5);

    declare_parameter<double>("force_x", 0.0);
    declare_parameter<double>("force_y", 0.0);
    declare_parameter<double>("force_z", 0.0);
    declare_parameter<double>("torque_x", 0.0);
    declare_parameter<double>("torque_y", 0.0);
    declare_parameter<double>("torque_z", 0.0);

    topic_control_wrench_ = get_parameter("topic_control_wrench").as_string();
    topic_compensated_wrench_ = get_parameter("topic_compensated_wrench").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
    normal_axis_ = get_parameter("normal_axis").as_string();
    normal_axis_sign_ = get_parameter("normal_axis_sign").as_double();

    contact_force_threshold_ = get_parameter("contact_force_threshold").as_double();
    release_force_threshold_ = get_parameter("release_force_threshold").as_double();
    force_target_ = get_parameter("force_target").as_double();
    force_rate_limit_ = get_parameter("force_rate_limit").as_double();
    force_tracking_epsilon_ = get_parameter("force_tracking_epsilon").as_double();

    if (normal_axis_sign_ >= 0.0) {
      normal_axis_sign_ = 1.0;
    } else {
      normal_axis_sign_ = -1.0;
    }

    if (normal_axis_ != "x" && normal_axis_ != "y" && normal_axis_ != "z") {
      RCLCPP_WARN(get_logger(), "Invalid normal_axis '%s', fallback to z", normal_axis_.c_str());
      normal_axis_ = "z";
    }

    double publish_rate = get_parameter("publish_rate").as_double();
    if (publish_rate <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid publish_rate %.3f, fallback to 100Hz", publish_rate);
      publish_rate = 100.0;
    }

    pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(topic_control_wrench_, rclcpp::QoS(10));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    compensated_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic_compensated_wrench_,
      rclcpp::QoS(10),
      std::bind(&ControlWrenchPublisherNode::on_compensated_wrench, this, std::placeholders::_1));

    const auto period_ns = static_cast<int64_t>(1e9 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::nanoseconds(period_ns),
      std::bind(&ControlWrenchPublisherNode::publish_wrench, this));

    last_publish_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Control wrench publisher started. pub_topic=%s sensor_topic=%s frame_id=%s rate=%.1fHz",
      topic_control_wrench_.c_str(), topic_compensated_wrench_.c_str(), frame_id_.c_str(), publish_rate);
    RCLCPP_INFO(
      get_logger(),
      "State-machine params: axis=%s sign=%.0f F_th=%.2fN F_release=%.2fN F_target=%.2fN dF_max=%.2fN/s eps=%.2fN",
      normal_axis_.c_str(), normal_axis_sign_, contact_force_threshold_, release_force_threshold_,
      force_target_, force_rate_limit_, force_tracking_epsilon_);
  }

private:
  void on_compensated_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received compensated wrench without frame_id, cannot transform.");
      return;
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
        frame_id_,
        msg->header.frame_id,
        msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));

      transformed_sensor_wrench_ = transform_wrench_to_target(msg->wrench, tf);
      has_transformed_sensor_wrench_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform compensated wrench from %s to %s: %s",
        msg->header.frame_id.c_str(), frame_id_.c_str(), ex.what());
    }
  }

  geometry_msgs::msg::Wrench transform_wrench_to_target(
    const geometry_msgs::msg::Wrench & src,
    const geometry_msgs::msg::TransformStamped & tf) const
  {
    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);
    tf2::Matrix3x3 rot(q);

    const tf2::Vector3 force_src(src.force.x, src.force.y, src.force.z);
    const tf2::Vector3 torque_src(src.torque.x, src.torque.y, src.torque.z);

    const tf2::Vector3 force_target = rot * force_src;
    const tf2::Vector3 torque_rotated = rot * torque_src;

    // translation is target->source vector expressed in target frame.
    const tf2::Vector3 p_target(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);
    const tf2::Vector3 torque_target = torque_rotated + p_target.cross(force_target);

    geometry_msgs::msg::Wrench out;
    out.force.x = force_target.x();
    out.force.y = force_target.y();
    out.force.z = force_target.z();
    out.torque.x = torque_target.x();
    out.torque.y = torque_target.y();
    out.torque.z = torque_target.z();
    return out;
  }

  double get_normal_force_abs(const geometry_msgs::msg::Wrench & wrench) const
  {
    double value = 0.0;
    if (normal_axis_ == "x") {
      value = wrench.force.x;
    } else if (normal_axis_ == "y") {
      value = wrench.force.y;
    } else {
      value = wrench.force.z;
    }
    return std::abs(value);
  }

  void set_normal_control_force(geometry_msgs::msg::Wrench & wrench, double signed_force) const
  {
    if (normal_axis_ == "x") {
      wrench.force.x = signed_force;
    } else if (normal_axis_ == "y") {
      wrench.force.y = signed_force;
    } else {
      wrench.force.z = signed_force;
    }
  }

  void update_state_machine(double dt)
  {
    const double abs_normal_force = has_transformed_sensor_wrench_
      ? get_normal_force_abs(transformed_sensor_wrench_) : 0.0;

    if (state_ == ForceState::FREE_SPACE) {
      force_ref_ = 0.0;
      if (abs_normal_force > contact_force_threshold_) {
        state_ = ForceState::CONTACT_TRANSITION;
        RCLCPP_INFO(
          get_logger(),
          "State transition: FREE_SPACE -> CONTACT_TRANSITION (|F_n|=%.3f)",
          abs_normal_force);
      }
      return;
    }

    if (state_ == ForceState::CONTACT_TRANSITION) {
      const double delta = force_target_ - force_ref_;
      const double max_step = std::max(0.0, force_rate_limit_) * std::max(0.0, dt);
      if (delta > max_step) {
        force_ref_ += max_step;
      } else if (delta < -max_step) {
        force_ref_ -= max_step;
      } else {
        force_ref_ = force_target_;
      }

      if (std::abs(force_target_ - force_ref_) < force_tracking_epsilon_) {
        state_ = ForceState::FORCE_TRACKING;
        RCLCPP_INFO(get_logger(), "State transition: CONTACT_TRANSITION -> FORCE_TRACKING");
      }
      return;
    }

    force_ref_ = force_target_;
    if (abs_normal_force < release_force_threshold_) {
      state_ = ForceState::FREE_SPACE;
      force_ref_ = 0.0;
      RCLCPP_INFO(
        get_logger(),
        "State transition: FORCE_TRACKING -> FREE_SPACE (|F_n|=%.3f)",
        abs_normal_force);
    }
  }

  void publish_wrench()
  {
    const auto current_time = now();
    double dt = (current_time - last_publish_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      dt = 0.01;
    }
    last_publish_time_ = current_time;

    update_state_machine(dt);

    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = frame_id_;

    msg.wrench.force.x = get_parameter("force_x").as_double();
    msg.wrench.force.y = get_parameter("force_y").as_double();
    msg.wrench.force.z = get_parameter("force_z").as_double();
    msg.wrench.torque.x = get_parameter("torque_x").as_double();
    msg.wrench.torque.y = get_parameter("torque_y").as_double();
    msg.wrench.torque.z = get_parameter("torque_z").as_double();

    const double signed_force_ref = normal_axis_sign_ * force_ref_;
    set_normal_control_force(msg.wrench, signed_force_ref);

    pub_->publish(msg);
  }

private:
  std::string topic_control_wrench_;
  std::string topic_compensated_wrench_;
  std::string frame_id_;
  double tf_lookup_timeout_sec_{0.05};

  std::string normal_axis_{"z"};
  double normal_axis_sign_{1.0};
  double contact_force_threshold_{2.0};
  double release_force_threshold_{1.0};
  double force_target_{10.0};
  double force_rate_limit_{20.0};
  double force_tracking_epsilon_{0.5};

  ForceState state_{ForceState::FREE_SPACE};
  double force_ref_{0.0};
  rclcpp::Time last_publish_time_;

  bool has_transformed_sensor_wrench_{false};
  geometry_msgs::msg::Wrench transformed_sensor_wrench_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr compensated_wrench_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlWrenchPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
