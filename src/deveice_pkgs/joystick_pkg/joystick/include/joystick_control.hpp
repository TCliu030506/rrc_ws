#ifndef JOYSTICK_CONTROL_HPP_
#define JOYSTICK_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "jyactuator_msg/srv/jy_action.hpp"
#include "mtactuator_msg/srv/mt_action.hpp"
#include "joystick_msg/msg/joystick_parameter.hpp"

class JoystickControlClient : public rclcpp::Node
{
public:
    JoystickControlClient();

private:
    void joystickInputCallback(const joystick_msg::msg::JoystickParameter::SharedPtr msg);
    void controlMotorPosition_jy(int position);
    void controlMotorPosition_mt_add(int maxSpeed, int position);
    void controlMotorPosition_mt_absolute(int maxSpeed, int position);

    int position_jy = 300;
    int position_mt = 1000;

    rclcpp::Subscription<joystick_msg::msg::JoystickParameter>::SharedPtr joystick_subscriber_;
    rclcpp::Client<jyactuator_msg::srv::JyAction>::SharedPtr motor_service_client_jy;
    rclcpp::Client<mtactuator_msg::srv::MtAction>::SharedPtr motor_service_client_mt;
};

#endif  // JOYSTICK_CONTROL_HPP_

