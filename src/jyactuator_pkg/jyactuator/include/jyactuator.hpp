#ifndef JYACTUATOR_HPP
#define JYACTUATOR_HPP

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
#include "jyactuator_msg/srv/jy_action.hpp"
#include "jyactuator_msg/msg/jy_position.hpp"

namespace jyactuator {

void InvertUint16(unsigned short *dBuf, unsigned short *srcBuf);

std::string msg_print(std::string head, uint8_t *buffer, unsigned long len);

class JyActuator : public rclcpp::Node {
private:
  
  uint8_t id;
  int loop_rate;
  
  std::string topic_pub;
  std::string serve_pub;

  serial::Serial ser;
  uint8_t transmit_buffer[40];
  unsigned long receive_length;
  
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<jyactuator_msg::msg::JyPosition>::SharedPtr jyactuator_pub;
  rclcpp::Service<jyactuator_msg::srv::JyAction>::SharedPtr jyactuator_act;

  bool pub_flag = false;
  bool is_service_on = true;

  uint16_t jy_pos;
  uint16_t jy_pos_goal;
  double jy_pos_plan;
  double jy_step;

  jyactuator_msg::msg::JyPosition pos_data;

  void pub_thread();

  bool serialport_init();

  bool serCallback(const std::shared_ptr<jyactuator_msg::srv::JyAction::Request> req, std::shared_ptr<jyactuator_msg::srv::JyAction::Response> res);

  void upload_msg();

  unsigned short CRC16_MODBUS(uint8_t *pdata, uint8_t datalen);

  void read_data();

  void receive_data();

  void report_data();

  void mov_cmd(uint16_t pos_in);

  void motion();

public:
  JyActuator(uint8_t id_in, int loop_rate_in, std::string topic_pub_in, std::string serve_pub_in);
  ~JyActuator();

/*
  bool serialport_init();

  bool serCallback(const std::shared_ptr<jyactuator_msg::srv::JyAction::Request> req, std::shared_ptr<jyactuator_msg::srv::JyAction::Response> res);

  void upload_msg();

  unsigned short CRC16_MODBUS(uint8_t *pdata, uint8_t datalen);

  void read_data();

  void receive_data();

  void report_data();

  void mov_cmd(uint16_t pos_in);

  void motion();

  void pub_thread();
  */
};

} // namespace jyactuator

#endif // JYACTUATOR_HPP
