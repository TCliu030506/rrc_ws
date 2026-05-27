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

    // 创建服务客户端
    motor_service_client_jy = this->create_client<jyactuator_msg::srv::JyAction>("jyactuator_srv");
    motor_service_client_mt = this->create_client<mtactuator_msg::srv::MtAction>("mtactuator_srv");

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

    if (a == 1) // 复位
    {
        controlMotorPosition_jy(300);
        controlMotorPosition_mt_absolute(100, 0);
    }
    else
    {
        if (b == 1) is_locked = true; // 锁死
        else if (x == 1) is_locked = false; // 解锁
 
        if (lx != 0 || ly != 0)
        {
            if (is_locked == false)
            {
                if (lx > 2000 || ly > 2000) controlMotorPosition_mt_absolute(100, joystick_mt_position);
                controlMotorPosition_jy(300 + joystick_jy_position);
                joystick_jy_position = 0;
                joystick_mt_position = 0;
            }
        }
        else
        {
            if (YY == 32767)
            {
                position_jy += 10;
                controlMotorPosition_jy(position_jy);
                RCLCPP_INFO(this->get_logger(), "YY_UP PRESSED!");
            }
            if (YY == -32767)
            {
                position_jy -= 10; 
                controlMotorPosition_jy(position_jy);
                RCLCPP_INFO(this->get_logger(), "YY_DOWN PRESSED!");
            }
            if (XX == 32767)
            {
                position_mt = 1000; 
                controlMotorPosition_mt_add(100, position_mt);
                RCLCPP_INFO(this->get_logger(), "XX_RIGHT PRESSED!");
            }
            if (XX == -32767)
            {
                position_mt = -1000; // reverse rotation
                controlMotorPosition_mt_add(100, position_mt);
                RCLCPP_INFO(this->get_logger(), "XX_LEFT PRESSED!");
            }
        }

    }
    
}

void JoystickControlClient::controlMotorPosition_jy(int position)
{
    auto request = std::make_shared<jyactuator_msg::srv::JyAction::Request>();
    request->pos = position;
    request->tim = 2.0;

    auto result_future = motor_service_client_jy->async_send_request(request);
}

void JoystickControlClient::controlMotorPosition_mt_add(int maxSpeed, int position)
{
    auto request = std::make_shared<mtactuator_msg::srv::MtAction::Request>();
    request->mode = 1;
    request->maxspeed = maxSpeed;
    request->pos = position;

    auto result_future = motor_service_client_mt->async_send_request(request);
}

void JoystickControlClient::controlMotorPosition_mt_absolute(int maxSpeed, int position)
{
    auto request = std::make_shared<mtactuator_msg::srv::MtAction::Request>();
    request->mode = 3;
    request->maxspeed = maxSpeed;
    request->pos = position;

    auto result_future = motor_service_client_mt->async_send_request(request);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickControlClient>());
    rclcpp::shutdown();
    return 0;
}