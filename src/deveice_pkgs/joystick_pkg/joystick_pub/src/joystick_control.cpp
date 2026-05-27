#include "../include/joystick_control.hpp"

#ifndef JY_PARAMETER
#define JY_PARAMETER 360            // 260
#endif

#ifndef MT_GEAR_RATIO
#define MT_GEAR_RATIO (30*1.0/26)   // 42-m1 & 26-m1
#endif

#ifndef PI
#define PI 3.14159265359
#endif

bool is_locked = false;

long joystick_jy_position;
long joystick_mt_position;

JoystickControlClient::JoystickControlClient()
    : Node("joystick_control_client")
{
    // 创建订阅者
    joystick_subscriber_ = this->create_subscription<joystick_msg::msg::JoystickParameter>(
        "joystick_input",
        80,
        std::bind(&JoystickControlClient::joystickInputCallback, this, std::placeholders::_1));

}

void JoystickControlClient::joystickInputCallback(const joystick_msg::msg::JoystickParameter::SharedPtr msg)
{

    int XX = msg->xx;
    int YY = msg->yy;
    long lx = msg->lx;
    long ly = msg->ly;
    int a = msg->a;
    int b = msg->b;
    int x = msg->x;

    joystick_jy_position = sqrt(lx * lx + ly * ly) / JY_PARAMETER;
    joystick_mt_position = atan2(ly, lx) / PI * 180 / MT_GEAR_RATIO * 100;

    // bool is_locked = false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickControlClient>());
    rclcpp::shutdown();
    return 0;
}