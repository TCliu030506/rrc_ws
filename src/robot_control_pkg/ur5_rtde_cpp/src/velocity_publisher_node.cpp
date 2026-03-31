// C++ ROS2节点：UR5 speedL速度控制
// 依赖：rclcpp, geometry_msgs, ur_rtde C++接口（需提前准备好）
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <chrono>
#include <cmath>

class URVelocityControlNode : public rclcpp::Node {
public:
    URVelocityControlNode() : Node("ur_velocity_control_node"), running_(true) {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.102");
        this->declare_parameter<std::string>("topic_cmd_vel", "/ur_cmd_vel");
        this->declare_parameter<double>("adaptive_acc_min", 0.3);
        this->declare_parameter<double>("adaptive_acc_max", 3.0);
        this->declare_parameter<double>("adaptive_acc_scale", 1.0);
        this->declare_parameter<bool>("enable_debug_output", true);
        this->declare_parameter<std::string>("topic_debug_sent_velocity", "/UR5/debug/sent_velocity");
        this->declare_parameter<std::string>("topic_debug_sent_acceleration", "/UR5/debug/sent_acceleration");
        this->declare_parameter<double>("dt", 1.0/125.0);

        robot_ip_ = this->get_parameter("robot_ip").as_string();
        topic_cmd_vel_ = this->get_parameter("topic_cmd_vel").as_string();
        adaptive_acc_min_ = this->get_parameter("adaptive_acc_min").as_double();
        adaptive_acc_max_ = this->get_parameter("adaptive_acc_max").as_double();
        adaptive_acc_scale_ = this->get_parameter("adaptive_acc_scale").as_double();
        enable_debug_output_ = this->get_parameter("enable_debug_output").as_bool();
        topic_debug_sent_velocity_ = this->get_parameter("topic_debug_sent_velocity").as_string();
        topic_debug_sent_acceleration_ = this->get_parameter("topic_debug_sent_acceleration").as_string();
        dt_ = this->get_parameter("dt").as_double();

        RCLCPP_INFO(this->get_logger(), "连接UR机器人: %s", robot_ip_.c_str());
        // rtde_c_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
        // rtde_r_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
        // if (!rtde_c_->isConnected() || !rtde_r_->isConnected()) {
        //     RCLCPP_FATAL(this->get_logger(), "RTDE接口连接失败");
        //     throw std::runtime_error("RTDE接口连接失败");
        // }
        // RCLCPP_INFO(this->get_logger(), "RTDE接口连接成功");

        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            topic_cmd_vel_, 10,
            std::bind(&URVelocityControlNode::cmd_vel_callback, this, std::placeholders::_1)
        );

        if (enable_debug_output_) {
            debug_sent_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_debug_sent_velocity_, 10);
            debug_sent_acceleration_pub_ = this->create_publisher<std_msgs::msg::Float64>(topic_debug_sent_acceleration_, 10);
        }

        send_thread_ = std::thread(&URVelocityControlNode::speed_send_loop, this);
    }

    ~URVelocityControlNode() override {
        running_ = false;
        if (send_thread_.joinable()) send_thread_.join();
        // if (rtde_c_) rtde_c_->stopScript();
        // if (rtde_c_) rtde_c_->disconnect();
        // if (rtde_r_) rtde_r_->disconnect();
        RCLCPP_INFO(this->get_logger(), "已断开UR RTDE连接");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        velocity_ = {msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z};
    }

    void speed_send_loop() {
        std::vector<double> last_sent_velocity(6, 0.0);
        while (running_) {
            std::vector<double> velocity;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                velocity = velocity_;
            }
            std::vector<double> delta_velocity(6, 0.0);
            double max_delta = 0.0;
            for (size_t i = 0; i < 6; ++i) {
                delta_velocity[i] = velocity[i] - last_sent_velocity[i];
                if (std::abs(delta_velocity[i]) > max_delta) max_delta = std::abs(delta_velocity[i]);
            }
            double required_acc = (max_delta / dt_) * adaptive_acc_scale_;
            double adaptive_acc = std::max(adaptive_acc_min_, std::min(adaptive_acc_max_, required_acc));
            // if (rtde_c_) rtde_c_->speedL(velocity, adaptive_acc, dt_);
            last_sent_velocity = velocity;
            if (enable_debug_output_) {
                std_msgs::msg::Float64MultiArray vel_msg;
                vel_msg.data = velocity;
                debug_sent_velocity_pub_->publish(vel_msg);
                std_msgs::msg::Float64 acc_msg;
                acc_msg.data = adaptive_acc;
                debug_sent_acceleration_pub_->publish(acc_msg);
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
        }
    }

    // RTDE接口成员变量（如有C++库请取消注释）
    // std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_c_;
    // std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_r_;
    std::string robot_ip_;
    std::string topic_cmd_vel_;
    double adaptive_acc_min_;
    double adaptive_acc_max_;
    double adaptive_acc_scale_;
    bool enable_debug_output_;
    std::string topic_debug_sent_velocity_;
    std::string topic_debug_sent_acceleration_;
    double dt_;
    std::atomic<bool> running_;
    std::vector<double> velocity_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::mutex mutex_;
    std::thread send_thread_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_sent_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_sent_acceleration_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<URVelocityControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
