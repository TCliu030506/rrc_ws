#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include "sigema7_pkg/dhdc.h"
#include "sigema7_pkg/drdc.h"
#include "xmate_cr7_msg/msg/cr7_state.hpp"


class Sigema7Control : public rclcpp::Node
{
public:
    Sigema7Control() : Node("sigema7_control")
    {
        // 初始化手柄
        deviceId = dhdOpen();
        if (deviceId < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open haptic device: %s", dhdErrorGetLastStr());
            return;
        }
        dhdEnableForce(DHD_ON, deviceId);

        // // 创建订阅者，订阅名为 "xmate_state" 的话题
        cr7_state_subscriber = this->create_subscription<xmate_cr7_msg::msg::Cr7State>(
            "xmate_state", 10, std::bind(&Sigema7Control::sigema7_control_callback, this, std::placeholders::_1));
        
    }
    ~Sigema7Control() 
    {
        dhdEnableForce(DHD_OFF, deviceId);
        dhdClose(deviceId);
    };

private:

    rclcpp::Subscription<xmate_cr7_msg::msg::Cr7State>::SharedPtr cr7_state_subscriber;
    int deviceId;

    std::array<double, 7> sigema7_force_default = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 7> sigema7_force_target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::array<double, 6> cr7_pos;
    std::array<double, 6> cr7_force;

    void sigema7_control_callback(const xmate_cr7_msg::msg::Cr7State::SharedPtr msg)
    {
        // 读取机械臂的位置和力
        for (size_t i = 0; i < msg->cr7_pos.size(); ++i)
        {
            cr7_pos[i] = msg->cr7_pos[i];
        }
        for (size_t i = 0; i < msg->cr7_force.size(); ++i)
        {
            cr7_force[i] = msg->cr7_force[i];
        }
        // 机械臂的位置和力旋转
        cr7_pos = rotatePose(cr7_pos);
        cr7_force = rotatePose(cr7_force);

        // 机械臂的力映射到手柄当中
        sigema7_force_target[0] = sigema7_force_default[0]+cr7_force[0] * 2;
        sigema7_force_target[1] = sigema7_force_default[0]+cr7_force[1] * 2;
        sigema7_force_target[2] = sigema7_force_default[0]+cr7_force[2] * 2;
        sigema7_force_target[3] = sigema7_force_default[0]+cr7_force[3] * 0.04;
        sigema7_force_target[4] = sigema7_force_default[0]+cr7_force[4] * 0.04;
        sigema7_force_target[5] = sigema7_force_default[0]+cr7_force[5] * 0.04;

        // 将力发送至手柄
        double frequency = dhdGetComFreq();
        std::cout << "交互频率：" << frequency*1000.0 << "Hz" <<std::endl;
        dhdSetForceAndTorque(sigema7_force_target[0], sigema7_force_target[1], sigema7_force_target[2],sigema7_force_target[3], sigema7_force_target[4], sigema7_force_target[5]);
        // dhdSetForceAndWristJointTorquesAndGripperForce(sigema7_force_target[0], sigema7_force_target[1], sigema7_force_target[2],sigema7_force_target[3], sigema7_force_target[4], sigema7_force_target[5],0.0);
        // dhdSetForceAndWristJointTorques(sigema7_force_target[0], sigema7_force_target[1], sigema7_force_target[2],sigema7_force_target[3], sigema7_force_target[4], sigema7_force_target[5]);
        // 打印手柄的力
        std::cout << "手柄的力：" << sigema7_force_target[0] << ", " << sigema7_force_target[1] << ", " << sigema7_force_target[2] << ", " << sigema7_force_target[3] << ", " << sigema7_force_target[4] << ", " << sigema7_force_target[5] << std::endl;
    }

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

};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sigema7Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
