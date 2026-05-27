#ifndef JOYSTICK_CONTROL_HPP_
#define JOYSTICK_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "joystick_msg/msg/joystick_parameter.hpp"

class JoystickControlClient : public rclcpp::Node
{
public:
    JoystickControlClient();

private:
    void joystickInputCallback(const joystick_msg::msg::JoystickParameter::SharedPtr msg);

    int position_jy = 300;
    int position_mt = 1000;

    rclcpp::Subscription<joystick_msg::msg::JoystickParameter>::SharedPtr joystick_subscriber_;
};

#endif  // JOYSTICK_CONTROL_HPP_

