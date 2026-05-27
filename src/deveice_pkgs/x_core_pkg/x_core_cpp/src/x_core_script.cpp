#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "../include/x_core_cpp/robot.h"  // 假设路径正确
#include "print_helper.hpp"

using namespace rokae;

class RobotSubscriberNode : public rclcpp::Node
{
public:
    RobotSubscriberNode() : Node("robot_subscriber_node")
    {
        // 创建订阅者，订阅名为 "robot_command" 的话题
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "robot_command", 10, std::bind(&RobotSubscriberNode::topic_callback, this, std::placeholders::_1));

        // 连接机器人
        std::string ip = "192.168.0.160";
        std::error_code ec;
        robot_ = std::make_unique<xMateRobot>(ip);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot: %s", ec.message().c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected to robot successfully");
        }
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: %s", msg->data.c_str());

        // 根据接收到的命令执行相应的操作
        if (msg->data == "basic_operation") {
            example_basicOperation(robot_.get());
        } else if (msg->data == "calibrate_frame") {
            example_calibrateFrame(robot_.get());
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", msg->data.c_str());
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::unique_ptr<xMateRobot> robot_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
