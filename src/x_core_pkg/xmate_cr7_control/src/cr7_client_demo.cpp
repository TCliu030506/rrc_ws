#include <rclcpp/rclcpp.hpp>
#include "xmate_cr7_msg/srv/cr7_script.hpp"
#include <iostream>
#include <memory>

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
    }

    // 发送请求的函数
    std::shared_ptr<xmate_cr7_msg::srv::Cr7Script_Response> send_request(const std::string& command, const std::array<double, 6> &pose = {}, int speed = 0, int zone = 0, bool all_axes = true, int axis_index = 0, const std::string& ref_type = "world", const std::string& rt_type = "jointPosition")
    {
        // 创建请求
        auto request = std::make_shared<xmate_cr7_msg::srv::Cr7Script::Request>();
        request->command = command;
        std::copy(pose.begin(), pose.end(), std::back_inserter(request->pose));
        request->speed = speed;
        request->zone = zone;
        request->all_axes = all_axes;
        request->axis_index = axis_index;
        request->ref_type = ref_type;
        request->rt_type = rt_type;

        // 异步发送请求
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // 处理响应
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service response: %s", response->result.c_str());
            if (!response->pose.empty()) {
                RCLCPP_INFO(this->get_logger(), "Pose response:");
                for (double val : response->pose) {
                    RCLCPP_INFO(this->get_logger(), "  %f", val);
                }
            }
            if (!response->joint.empty()) {
                RCLCPP_INFO(this->get_logger(), "Joint response:");
                for (double val : response->joint) {
                    RCLCPP_INFO(this->get_logger(), "  %f", val);
                }
            }
            if (response->vel.size() > 0) {
                RCLCPP_INFO(this->get_logger(), "Velocity response:");
                for (double val : response->vel) {
                    RCLCPP_INFO(this->get_logger(), "  %f", val);
                }
            }
            if (response->acc > 0) {
                RCLCPP_INFO(this->get_logger(), "Acceleration response: %f", response->acc);
            }
            if (response->jerk > 0) {
                RCLCPP_INFO(this->get_logger(), "Jerk response: %f", response->jerk);
            }
            if (!response->torque.empty()) {
                RCLCPP_INFO(this->get_logger(), "Joint Torque response:");
                for (double val : response->torque) {
                    RCLCPP_INFO(this->get_logger(), "  %f", val);
                }
            }
            if (!response->external_torque.empty()) {
                RCLCPP_INFO(this->get_logger(), "External Joint Torque response:");
                for (double val : response->external_torque) {
                    RCLCPP_INFO(this->get_logger(), "  %f", val);
                }
            }
            if (!response->cart_torque.empty()) {
                RCLCPP_INFO(this->get_logger(), "Cartesian Torque response:");
                for (double val : response->cart_torque) {
                    RCLCPP_INFO(this->get_logger(), "  %f", val);
                }
            }
            if (!response->cart_force.empty()) {
                RCLCPP_INFO(this->get_logger(), "Cartesian Force response:");
                for (double val : response->cart_force) {
                    RCLCPP_INFO(this->get_logger(), "  %f", val);
                }
            }
            return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service xmate_cr7_server");
            return nullptr;
        }
    }

private:
    rclcpp::Client<xmate_cr7_msg::srv::Cr7Script>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<Cr7Client>();

    // 示例请求1：movej
    std::array<double, 6> joint_angles = {M_PI/2, 0, M_PI/2, 0, M_PI/2, 0};
    // std::array<double, 6> joint_angles = {M_PI/2, M_PI/6, M_PI/3, 0, M_PI/2, 0};
    client->send_request("movej", joint_angles, 100, 30);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::array<double, 6> joint_angles2 = {M_PI/2, 0, M_PI/2, M_PI/6, M_PI/2, 0};
    client->send_request("movej", joint_angles2, 100, 30);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    client->send_request("movej", joint_angles, 100, 30);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 示例请求1：movep
    std::array<double, 6> pose = {0.200, -0.356, 0.724, -150.0*M_PI/180.0, 0.0, -85.0*M_PI/180.0};
    client->send_request("movep", pose, 100, 30);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::array<double, 6> pose2 = {0.150, -0.360, 0.660, -M_PI, 0.0, -M_PI/2};
    client->send_request("movep", pose2, 100, 30);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    client->send_request("stop");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 示例请求1：movel
    std::array<double, 6> posel = {0.200, -0.356, 0.724, -150.0*M_PI/180.0, 0.0, -85.0*M_PI/180.0};
    client->send_request("movel", posel, 100, 30);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::array<double, 6> posel2 = {0.150, -0.360, 0.660, -M_PI, 0.0, -M_PI/2};
    client->send_request("movel", posel2, 100, 30);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));


    // // 示例请求2：follow_pos_start
    // client->send_request("follow_pos_start");
    // // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // // 每隔 10ms 更新点位
    // for (int i = 0; i < 1000; ++i) { 
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     joint_angles[4] += M_PI*6/10000; // 更新第 5 个关节角度
    //     client->send_request("follow_pos_update", joint_angles, 400);
    // }
    // for (int i = 0; i < 1000; ++i) { 
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     joint_angles[4] -= M_PI*6/10000; 
    //     client->send_request("follow_pos_update", joint_angles, 400);
    // }
    // client->send_request("follow_pos_stop");

    // // 示例请求3：open_rtControl_loop
    // client->send_request("open_rtControl_loop");
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // // 每隔 500ms 更新点位
    // for (int i = 0; i < 10000; ++i) { // 这里假设更新 10 次，你可以根据需要修改
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     joint_angles[4] += M_PI*6/100000; // 假设更新第 5 个关节角度，你可以根据需要修改
    //     client->send_request("rtControl_jointpos_update", joint_angles);
    // }
    // for (int i = 0; i < 10000; ++i) { // 这里假设更新 10 次，你可以根据需要修改
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     joint_angles[4] -= M_PI*6/100000; // 假设更新第 5 个关节角度，你可以根据需要修改
    //     client->send_request("rtControl_jointpos_update", joint_angles);
    // }

    // // 示例请求3：open_rtControl_loop
    // client->send_request("open_rtControl_loop", {}, 0, 0, true, 0, "world", "cartesianPosition");
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // // 每隔 500ms 更新点位
    // for (int i = 0; i < 10000; ++i) { // 这里假设更新 10 次，你可以根据需要修改
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     posel2[1] += 0.2/10000; // 沿y方向运动
    //     client->send_request("rtControl_cartesianpos_update", posel2);
    // }
    // for (int i = 0; i < 10000; ++i) { // 这里假设更新 10 次，你可以根据需要修改
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     posel2[1] -= 0.2/10000;
    //     client->send_request("rtControl_cartesianpos_update", posel2);
    // }

    // // 示例请求4：readp
    // client->send_request("readp");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求5：readj
    // client->send_request("readj");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求6：get_vel
    // client->send_request("get_vel");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求7：get_joint_vel
    // client->send_request("get_joint_vel");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求8：calibrateForceSensor
    // client->send_request("calibrateForceSensor");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求9：getEndTorque
    // client->send_request("getEndTorque");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求10：get_joint_torque
    // client->send_request("get_joint_torque");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求11：get_acc
    // client->send_request("get_acc");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求12：get_jerk
    // client->send_request("get_jerk");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求13：inv_kinematics
    // client->send_request("inv_kinematics", {0.15, -0.35, 0.66, M_PI, 0.0, -M_PI/2});
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例请求14：forward_kinematics
    // client->send_request("forward_kinematics", {M_PI/2, 0, M_PI/2, 0, M_PI/2, 0});
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    rclcpp::shutdown();
    return 0;
}
