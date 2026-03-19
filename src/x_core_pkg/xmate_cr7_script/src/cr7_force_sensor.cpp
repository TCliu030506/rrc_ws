#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>
#include <atomic>
#include <chrono>
#include <sstream>
#include "xmate_cr7_script/cr7_script.h"
#include "force_sensor_msg/msg/three_axis_force.hpp"
#include "force_sensor_msg/srv/serialport.hpp"

// 客户端类定义
class Cr7ForceSensor : public rclcpp::Node
{
public:
    Cr7ForceSensor() : Node("cr7_force_sensor")
    {
        // 创建客户端实例
        cr7_force_sensor_client = std::make_shared<cr7_script::Cr7ScriptClient>();
        // 创建一个发布者，用于周期性发布力传感器数据
        forcesensor_pub = this->create_publisher<force_sensor_msg::msg::ThreeAxisForce>("forcesensor", 10);
        // 创建一个服务服务器，用于设置力传感器参数
        command_ser = this->create_service<force_sensor_msg::srv::Serialport>("forcesensor_srv",
            std::bind(&Cr7ForceSensor::handle_command, this, std::placeholders::_1, std::placeholders::_2));
        // 声明与获取轮询频率参数（Hz）
        declare_parameter<double>("force_poll_hz", 100.0);
        double hz = get_parameter("force_poll_hz").as_double();
        if (hz <= 0.0) hz = 50.0;
        auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));

        // 定时器：周期请求机械臂末端受力
        timer_ = this->create_wall_timer(period, std::bind(&Cr7ForceSensor::poll_force, this));

        // 初始化成功后打印出该节点的功能已启动
        RCLCPP_INFO(this->get_logger(), "cr7_force_sensor_client 已启动，轮询频率：%.2f Hz", hz);
        RCLCPP_INFO(this->get_logger(), "正在向话题/forcesensor 发布末端力数据");
    }

private:
    std::shared_ptr<cr7_script::Cr7ScriptClient> cr7_force_sensor_client;
    std::string cr7_response;

    rclcpp::Publisher<force_sensor_msg::msg::ThreeAxisForce>::SharedPtr forcesensor_pub;
    rclcpp::Service<force_sensor_msg::srv::Serialport>::SharedPtr command_ser;

    void poll_force();
    bool handle_command(const std::shared_ptr<force_sensor_msg::srv::Serialport::Request> req, const std::shared_ptr<force_sensor_msg::srv::Serialport::Response> res);

    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> in_flight_{false};

};

bool Cr7ForceSensor::handle_command(
    const std::shared_ptr<force_sensor_msg::srv::Serialport::Request> req,
    const std::shared_ptr<force_sensor_msg::srv::Serialport::Response> res)
{
    // 仅实现能在本客户端完成的子集：GOD（一次性读取并返回外力）
    const std::string call = req->callname;
    const std::string cmd  = req->command;

    if (call == "GOD")
    {
        // 向机械臂服务请求一次末端外力状态，解析末端外力并发布
        cr7_force_sensor_client->getEndTorque("world", cr7_response);
        // 解析响应字符串，提取力传感器数据，解析 cart_force 数据
        std::size_t pos = cr7_response.find("cart_force:");
        if (pos != std::string::npos) {
            std::string cart_force_str = cr7_response.substr(pos + std::string("cart_force:").length());
            auto end_pos = cart_force_str.find(';');
            if (end_pos != std::string::npos) {
                cart_force_str = cart_force_str.substr(0, end_pos);
            }
            std::istringstream iss(cart_force_str);
            std::string val;
            std::vector<double> cart_force_vals;
            while (std::getline(iss, val, ',')) {
                try {
                    cart_force_vals.push_back(std::stod(val));
                } catch (...) {}
            }
            if (cart_force_vals.size() >= 3) {
                // 组织响应字符串，并发布一次
                std::ostringstream oss;
                oss << "GOD OK: Fx=" << cart_force_vals[0] << ", Fy=" << cart_force_vals[1] << ", Fz=" << cart_force_vals[2];
                res->result = oss.str();

                // 发布一次力传感器数据
                force_sensor_msg::msg::ThreeAxisForce msg;
                msg.header.stamp = this->now();
                msg.x_axis = cart_force_vals[0]; 
                msg.y_axis = cart_force_vals[1]; 
                msg.z_axis = cart_force_vals[2];
                forcesensor_pub->publish(msg);
                return true;
            } else {
                res->result = "getEndTorque has no cart_force in response.";
                return false;
            }
        } else {
            res->result = "cart_force not available in response.";
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

void Cr7ForceSensor::poll_force()
{
    if (in_flight_) return; // 上次请求未返回，跳过本周期
    in_flight_ = true;
    // 发送请求得到机械臂的受力状态
    cr7_force_sensor_client->getEndTorque("world", cr7_response);
    // 解析响应字符串，提取力传感器数据，解析 cart_force 数据
    force_sensor_msg::msg::ThreeAxisForce msg;
    std::size_t pos = cr7_response.find("cart_force:");
    if (pos != std::string::npos) {
        std::string cart_force_str = cr7_response.substr(pos + std::string("cart_force:").length());
        auto end_pos = cart_force_str.find(';');
        if (end_pos != std::string::npos) {
            cart_force_str = cart_force_str.substr(0, end_pos);
        }
        std::istringstream iss(cart_force_str);
        std::string val;
        std::vector<double> cart_force_vals;
        while (std::getline(iss, val, ',')) {
            try {
                cart_force_vals.push_back(std::stod(val));
            } catch (...) {}
        }
        if (cart_force_vals.size() >= 3) {
            msg.x_axis = cart_force_vals[0];
            msg.y_axis = cart_force_vals[1];
            msg.z_axis = cart_force_vals[2];
            msg.header.stamp = this->now();
            forcesensor_pub->publish(msg);
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "cart_force not available in response");
    }
    in_flight_ = false;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<Cr7ForceSensor>();
    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}

