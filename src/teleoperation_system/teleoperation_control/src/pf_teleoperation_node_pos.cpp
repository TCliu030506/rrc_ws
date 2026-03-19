#define XMATEMODEL_LIB_SUPPORTED
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include <thread>
#include <atomic>
#include "sigema7_msg/msg/sigema7_position.hpp"
#include"pf_control/pf_control.h"

using namespace std;

class TeleControlClient : public rclcpp::Node
{
public:
    TeleControlClient() : Node("pf_teleoreration_node"),controller("/dev/ttyUSB0")
    {
        // // 创建订阅者，订阅名为 "sigema7_getpos" 的话题
        tele_control_subscriber = this->create_subscription<sigema7_msg::msg::Sigema7Position>(
            "sigema7_getpos", 10, std::bind(&TeleControlClient::tele_control_callback, this, std::placeholders::_1));
    }
    ~TeleControlClient() {};

private:
    pf_control controller;
    rclcpp::Subscription<sigema7_msg::msg::Sigema7Position>::SharedPtr tele_control_subscriber;

    double loop_time = 0.001; //单位为秒
    double mapping_ratio = 0.08;
    double sigema7_pos_default[6] = {-0.00544945, -0.000884445, -0.00275382, 0.00194075, 0.000535195, -0.00308807};
    double pf_pos_default[3] = {0.020696, 0.0152364, -56.43};  //单位为mm

    // 函数用于计算绕原点旋转后的新位姿
    std::array<double, 6> rotatePose(const std::array<double, 6>& pose) {
        // 提取原始位姿
        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double original_rx = pose[3];
        double original_ry = pose[4];
        double original_rz = pose[5];
        
        //手动调整xyz
        double rotated_x = -y;
        double rotated_y = x;
        double rotated_z = z;

        // 单独调整合适的RPY
        double rotated_rx = -1*original_rx;
        double rotated_ry = -1*original_ry;
        double rotated_rz = original_rz;

        // 返回新的位姿
        return {rotated_x, rotated_y, rotated_z, rotated_rx, rotated_ry, rotated_rz};
    }

    // 函数用于计算绕原点旋转后的新位姿
    std::array<double, 3> rotateVel(const std::array<double, 3>& vel) {
        // 提取原始位姿
        double vx = vel[0];
        double vy = vel[1];
        double vz = vel[2];
        
        //手动调整xyz
        double rotated_x = -vy;
        double rotated_y = vx;
        double rotated_z = vz;

        // 返回新的位姿
        return {rotated_x, rotated_y, rotated_z};
    }

    /**
     * @brief 回调函数，处理手柄输入并更新目标点位
     */
    void tele_control_callback(const sigema7_msg::msg::Sigema7Position::SharedPtr msg)
    {
       std::cout << "进入回调函数: " << std::endl;

        // 获取手柄的输入位置、速度和力
        std::array<double, 6> sigema7_pos = {0, 0, 0, 0, 0, 0};
        std::array<double, 3> sigema7_vel_linear= {0, 0, 0};
        std::array<double, 7> sigema7_force= {0, 0, 0, 0, 0, 0, 0};

        for (size_t i = 0; i < msg->sigema7_pos.size(); ++i)
        {
            sigema7_pos[i] = msg->sigema7_pos[i];
        }
        for (size_t i = 0; i < msg->sigema7_vel_linear.size(); ++i)
        {
            sigema7_vel_linear[i] = msg->sigema7_vel_linear[i];
        }
        for (size_t i = 0; i < msg->sigema7_force.size(); ++i)
        {
            sigema7_force[i] = msg->sigema7_force[i];
        }

        // 手柄位置映射到机械臂位置
        // 先计算手柄运动的映射
        std::array<double, 6> sigema7_pos_mapping = {0, 0, 0, 0, 0, 0};
        sigema7_pos_mapping[0] = sigema7_pos[0] - sigema7_pos_default[0];
        sigema7_pos_mapping[1] = sigema7_pos[1] - sigema7_pos_default[1];
        sigema7_pos_mapping[2] = sigema7_pos[2] - sigema7_pos_default[2];
        sigema7_pos_mapping[3] = sigema7_pos[3] - sigema7_pos_default[3];
        sigema7_pos_mapping[4] = sigema7_pos[4] - sigema7_pos_default[4];
        sigema7_pos_mapping[5] = sigema7_pos[5] - sigema7_pos_default[5];
        // // 根据机械臂实际情况进行旋转
        // sigema7_pos_mapping = rotatePose(sigema7_pos_mapping);
        // 计算PF的目标位置
        std::array<double, 3> pos_mapping = {0, 0, 0};
        pos_mapping[0] = pf_pos_default[0] + sigema7_pos_mapping[0] * 1000 * mapping_ratio;
        pos_mapping[1] = pf_pos_default[1] + sigema7_pos_mapping[1] * 1000 * mapping_ratio;
        pos_mapping[2] = pf_pos_default[2] + sigema7_pos_mapping[2] * 1000 * mapping_ratio;

        // 手柄速度映射到机械臂速度
        std::array<double, 3> vel_mapping = {0, 0, 0};
        vel_mapping[0] = sigema7_vel_linear[0] * mapping_ratio *1000 ;
        vel_mapping[1] = sigema7_vel_linear[1] * mapping_ratio *1000;
        vel_mapping[2] = sigema7_vel_linear[2] * mapping_ratio *1000;
        // vel_mapping = rotateVel(vel_mapping);

        std::cout << "—————————————TEST——————————— " << std::endl;
        // 将位置发送至内窥镜运动平台
        controller.pf_movep(pos_mapping[0], pos_mapping[1], pos_mapping[2], loop_time);

        // 打印出目标位置
        std::cout << "映射位置： " << std::endl;
        std::cout << pos_mapping[0] << ',' << pos_mapping[1] << ',' << pos_mapping[2] << std::endl;
        std::cout << "———————————————————————— " << std::endl;
        std::cout << "完成发送 " << std::endl;
        std::cout << "———————————————————————— " << std::endl;
    }

    
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleControlClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
