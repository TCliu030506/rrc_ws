//用于接受话题"robotstate"(在节点ur5_msg_demo中创建)的信息并且打印出来

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ur5_msg/msg/robot_state.hpp"


void topic_callback(const ur5_msg::msg::RobotState::SharedPtr msg, rclcpp::Logger logger) {
    RCLCPP_INFO(logger, "Received message:");
    RCLCPP_INFO(logger, "Header:");
    RCLCPP_INFO(logger, "  Stamp: %d.%09d", msg->header.stamp.sec, msg->header.stamp.nanosec);
    RCLCPP_INFO(logger, "  Frame ID: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(logger, "Joint Positions:");
    for (size_t i = 0; i < 6; ++i) {
        RCLCPP_INFO(logger, "  Position of joint %zu: %f", i, msg->joint_pos[i]);
    }
    RCLCPP_INFO(logger, "Joint Velocities:");
    for (size_t i = 0; i < 6; ++i) {
        RCLCPP_INFO(logger, "  Velocity of joint %zu: %f", i, msg->joint_vel[i]);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("listener_node");

   // auto subscription = node->create_subscription<ur5_msg::msg::RobotState>(
   //     "robotstate", 10, std::bind(topic_callback, std::placeholders::_1, node->get_logger()));

    auto subscription = node->create_subscription<ur5_msg::msg::RobotState>(
        "robotstate", 10, [=](const ur5_msg::msg::RobotState::SharedPtr msg) {
            topic_callback(msg, node->get_logger());
        });

    RCLCPP_INFO(node->get_logger(), "Listening to 'chatter' topic...");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
