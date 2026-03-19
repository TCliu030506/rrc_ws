#include <rclcpp/rclcpp.hpp>
#include "xmate_cr7_msg/srv/cr7_script.hpp"
#include <iostream>
#include <memory>
#include "force_sensor_msg/msg/three_axis_force.hpp"
#include "force_sensor_msg/srv/serialport.hpp"
#include <atomic>
#include <chrono>
#include <sstream>

// 客户端类定义
class Cr7Client : public rclcpp::Node
{
public:
    Cr7Client() : Node("cr7_client")
    {
        // 创建服务客户端
        client_ = this->create_client<xmate_cr7_msg::srv::Cr7Script>("xmate_cr7_server");
        // 等待服务器启动
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        // 创建一个发布者，用于周期性发布力传感器数据
        forcesensor_pub = this->create_publisher<force_sensor_msg::msg::ThreeAxisForce>("forcesensor", 10);
        // 创建一个服务服务器，用于设置力传感器参数
        command_ser = this->create_service<force_sensor_msg::srv::Serialport>("forcesensor_srv",
            std::bind(&Cr7Client::handle_command, this, std::placeholders::_1, std::placeholders::_2));
        // 声明与获取轮询频率参数（Hz）
        declare_parameter<double>("force_poll_hz", 100.0);
        double hz = get_parameter("force_poll_hz").as_double();
        if (hz <= 0.0) hz = 50.0;
        auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));

        // 定时器：周期请求机械臂末端受力
        timer_ = this->create_wall_timer(period, std::bind(&Cr7Client::poll_force, this));

        // 初始化成功后打印出该节点的功能已启动
        RCLCPP_INFO(this->get_logger(), "cr7_force_sensor_client 已启动，轮询频率：%.2f Hz", hz);
        RCLCPP_INFO(this->get_logger(), "正在向话题/forcesensor 发布末端力数据");
    }

private:
    rclcpp::Client<xmate_cr7_msg::srv::Cr7Script>::SharedPtr client_;
    rclcpp::Publisher<force_sensor_msg::msg::ThreeAxisForce>::SharedPtr forcesensor_pub;
    rclcpp::Service<force_sensor_msg::srv::Serialport>::SharedPtr command_ser;

    void poll_force();
    bool handle_command(const std::shared_ptr<force_sensor_msg::srv::Serialport::Request> req, const std::shared_ptr<force_sensor_msg::srv::Serialport::Response> res);

    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> in_flight_{false};

};

bool Cr7Client::handle_command(
    const std::shared_ptr<force_sensor_msg::srv::Serialport::Request> req,
    const std::shared_ptr<force_sensor_msg::srv::Serialport::Response> res)
{
    // 仅实现能在本客户端完成的子集：GOD（一次性读取并返回外力）
    const std::string call = req->callname;
    const std::string cmd  = req->command;

    if (call == "GOD")
    {
        // 向机械臂服务请求一次末端外力
        auto arm_req = std::make_shared<xmate_cr7_msg::srv::Cr7Script::Request>();
        arm_req->command  = "getEndTorque";
        arm_req->ref_type = "world";

        // 同步等待（设置超时，避免阻塞执行器过久）
        auto fut = client_->async_send_request(arm_req);
        if (fut.wait_for(std::chrono::milliseconds(800)) != std::future_status::ready) {
            res->result = "Timeout: getEndTorque response not ready.";
            return false;
        }
        auto arm_res = fut.get();
        if (!arm_res) {
            res->result = "Empty response from xmate_cr7_server.";
            return false;
        }

        // 组织响应字符串，并发布一次
        std::ostringstream oss;
        if (arm_res->cart_force.size() >= 3) {
            const double fx = arm_res->cart_force[0];
            const double fy = arm_res->cart_force[1];
            const double fz = arm_res->cart_force[2];

            // 返回字符串
            oss << "GOD OK: Fx=" << fx << ", Fy=" << fy << ", Fz=" << fz;
            res->result = oss.str();

            // 可选：顺便发布一次消息
            force_sensor_msg::msg::ThreeAxisForce msg;
            msg.header.stamp = this->now();
            msg.x_axis = fx; msg.y_axis = fy; msg.z_axis = fz;
            forcesensor_pub->publish(msg);
            return true;
        } else {
            res->result = "getEndTorque has no cart_force in response.";
            return false;
        }
    }
    else if (call == "HELP")
    {
        res->result = "Supported commands in this node: GOD (one-shot read). "
                      "Not supported here: ADJZF/SMPR/SGDM.";
        return true;
    }
    else
    {
        res->result = "Unsupported in this node: " + call +
                      ". Only GOD is implemented; ADJZF/SMPR/SGDM require serial node.";
        return false;
    }
}

void Cr7Client::poll_force()
{
    if (in_flight_) return; // 上次请求未返回，跳过本周期
    auto req = std::make_shared<xmate_cr7_msg::srv::Cr7Script::Request>();
    req->command = "getEndTorque";   // 服务器侧已支持的命令，返回 cart_force/cart_torque
    req->ref_type = "world";
    in_flight_ = true;

    auto cb = [this](rclcpp::Client<xmate_cr7_msg::srv::Cr7Script>::SharedFuture fut) {
        auto res = fut.get();
        if (!res) {
            RCLCPP_WARN(this->get_logger(), "Empty response from xmate_cr7_server");
            in_flight_ = false;
            return;
        }
        // 从响应中提取笛卡尔外力并发布（单位与坐标系以服务端定义为准，通常是工具坐标系）
        if (res->cart_force.size() >= 3) {
            force_sensor_msg::msg::ThreeAxisForce msg;
            msg.header.stamp = this->now();
            msg.x_axis = res->cart_force[0];
            msg.y_axis = res->cart_force[1];
            msg.z_axis = res->cart_force[2];
            forcesensor_pub->publish(msg);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "cart_force not available in response");
        }
        in_flight_ = false;
    };

    // 异步调用，回调内发布，避免阻塞执行器
    (void)client_->async_send_request(req, cb);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<Cr7Client>();
    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}
