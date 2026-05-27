// 脉塔电机RMD-L-4015-CAN通讯

#ifndef MTACTUATOR_H
#define MTACTUATOR_H

#include <unistd.h>
#include <math.h> 
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
 
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "mtactuator_msg/srv/mt_action.hpp"
#include "mtactuator_msg/msg/mt_position.hpp"

namespace mtactuator {

// class to communicate with ftmotors
class MtActuator : public rclcpp::Node
{

public:

  MtActuator(int loop_rate_in, std::string topic_pub_in, std::string serve_pub_in);
  ~MtActuator();
  
  int ReadAngle();

  void Move(uint8_t mode, uint16_t maxSpeed, int64_t angleControl, int32_t speed);

  void AngleControl_add(uint16_t maxSpeed, int64_t angleControl);

  void SpeedControl(int32_t speed);

  void AngleControl_absolute(uint16_t maxSpeed, int64_t angleControl);

  void AngleControl_track(uint16_t maxSpeed, int64_t angleControl);

  void Close();

  void Stop();

private:
  
  int loop_rate;
  std::string topic_pub;
  std::string serve_pub;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Publisher<mtactuator_msg::msg::MtPosition>::SharedPtr msg_pub;
  rclcpp::Service<mtactuator_msg::srv::MtAction>::SharedPtr srv_pub;

  serial::Serial ser;                       //声明串口对象 
  uint8_t transmit_buffer[80];
  unsigned long receive_length;
  
  // 零位置
  long pos_0; 
  

  std::string msg_print(std::string head, uint8_t * buffer, unsigned long len);

  // receive serialport data
  void receive_pos();

  bool serialport_init();

  //srv callback
  bool serCallback(
    const std::shared_ptr<mtactuator_msg::srv::MtAction::Request> req, 
    const std::shared_ptr<mtactuator_msg::srv::MtAction::Response> res);

  //pub motor_msg thread
  void pub_thread();
  void upload_msg(int32_t pos);
  
};




} //mtactuator

#endif // MTACTUATOR_H