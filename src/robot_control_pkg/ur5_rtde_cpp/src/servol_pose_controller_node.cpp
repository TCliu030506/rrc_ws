// C++ ROS2节点：UR5 servoL位姿控制
// 依赖：rclcpp, geometry_msgs, ur_rtde C++接口（需提前准备好）
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <cmath>
#include <chrono>
// #include <ur_rtde/rtde_control_interface.h> // 假设有C++接口
// #include <ur_rtde/rtde_receive_interface.h>

using namespace std::chrono_literals;

class URServoLPoseControllerNode : public rclcpp::Node {
public:
    URServoLPoseControllerNode() : Node("ur_servol_pose_controller_node"), running_(true), first_frame_aligned_(false) {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.102");
        this->declare_parameter<std::string>("topic_cmd_pose", "/ur_cmd_pose");
        this->declare_parameter<double>("speed", 0.15);
        this->declare_parameter<double>("acceleration", 0.1);
        this->declare_parameter<double>("lookahead_time", 0.1);
        this->declare_parameter<double>("gain", 300.0);
        this->declare_parameter<std::vector<double>>("tcp_offset", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<double>("dt", 1.0/125.0);

        robot_ip_ = this->get_parameter("robot_ip").as_string();
        topic_cmd_pose_ = this->get_parameter("topic_cmd_pose").as_string();
        speed_ = this->get_parameter("speed").as_double();
        acceleration_ = this->get_parameter("acceleration").as_double();
        lookahead_time_ = this->get_parameter("lookahead_time").as_double();
        gain_ = this->get_parameter("gain").as_double();
        tcp_offset_ = this->get_parameter("tcp_offset").as_double_array();
        dt_ = this->get_parameter("dt").as_double();

        if (tcp_offset_.size() != 6) {
            RCLCPP_FATAL(this->get_logger(), "tcp_offset参数长度必须为6");
            throw std::runtime_error("tcp_offset参数长度必须为6");
        }

        RCLCPP_INFO(this->get_logger(), "连接UR机器人: %s", robot_ip_.c_str());
        // rtde_c_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
        // rtde_r_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
        // if (!rtde_c_->isConnected() || !rtde_r_->isConnected()) {
        //     RCLCPP_FATAL(this->get_logger(), "RTDE接口连接失败");
        //     throw std::runtime_error("RTDE接口连接失败");
        // }
        // rtde_c_->setTcp(tcp_offset_);
        // RCLCPP_INFO(this->get_logger(), "RTDE接口连接成功");

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            topic_cmd_pose_, 10,
            std::bind(&URServoLPoseControllerNode::pose_callback, this, std::placeholders::_1)
        );

        send_thread_ = std::thread(&URServoLPoseControllerNode::servo_send_loop, this);
    }

    ~URServoLPoseControllerNode() override {
        running_ = false;
        if (send_thread_.joinable()) send_thread_.join();
        // if (rtde_c_) rtde_c_->stopScript();
        // if (rtde_c_) rtde_c_->disconnect();
        // if (rtde_r_) rtde_r_->disconnect();
        RCLCPP_INFO(this->get_logger(), "已断开UR RTDE连接");
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        std::vector<double> rotvec = quaternion_to_rotvec(
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w
        );
        std::vector<double> target_pose = {
            msg->position.x, msg->position.y, msg->position.z,
            rotvec[0], rotvec[1], rotvec[2]
        };
        std::lock_guard<std::mutex> lock(mutex_);
        target_pose_ = target_pose;
    }

    std::vector<double> quaternion_to_rotvec(double qx, double qy, double qz, double qw) {
        double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if (norm < 1e-12) return {0.0, 0.0, 0.0};
        qx /= norm; qy /= norm; qz /= norm; qw /= norm;
        double angle = 2.0 * std::acos(std::max(-1.0, std::min(1.0, qw)));
        double s = std::sqrt(std::max(0.0, 1.0 - qw*qw));
        if (s < 1e-9) return {0.0, 0.0, 0.0};
        double ax = qx / s, ay = qy / s, az = qz / s;
        return {ax * angle, ay * angle, az * angle};
    }

    void servo_send_loop() {
        while (running_) {
            std::vector<double> target_pose;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                target_pose = target_pose_;
            }
            if (!target_pose.empty()) {
                // if (!first_frame_aligned_) {
                //     auto current_tcp_pose = rtde_r_->getActualTCPPose();
                //     if (current_tcp_pose.size() == 6) {
                //         target_pose = current_tcp_pose;
                //         first_frame_aligned_ = true;
                //         RCLCPP_INFO(this->get_logger(), "首帧已对齐当前TCP位姿");
                //     }
                // }
                // rtde_c_->servoL(target_pose, speed_, acceleration_, dt_, lookahead_time_, gain_);
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
        }
    }

    // RTDE接口成员变量（如有C++库请取消注释）
    // std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_c_;
    // std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_r_;
    std::string robot_ip_;
    std::string topic_cmd_pose_;
    double speed_;
    double acceleration_;
    double lookahead_time_;
    double gain_;
    std::vector<double> tcp_offset_;
    double dt_;
    std::atomic<bool> running_;
    std::atomic<bool> first_frame_aligned_;
    std::vector<double> target_pose_;
    std::mutex mutex_;
    std::thread send_thread_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<URServoLPoseControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
