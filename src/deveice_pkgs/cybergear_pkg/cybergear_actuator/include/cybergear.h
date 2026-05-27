#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include <unistd.h>
#include <math.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdexcept>  // for std::runtime_error
#include <cstring>    // for memcpy
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "coordinate/msg/array_int16.hpp"
#include "cybergear_msg/srv/cybergear_script.hpp"
#include "cybergear_msg/msg/cybergear_state.hpp"

namespace cybergear {

class cybergear : public rclcpp::Node
{
public:
  cybergear();
  ~cybergear();

  // 功能命令函数
  void cybergear_send_read_req(uint8_t can_id_req);
  void cybergear_poll_and_parse_once();
  void send_wake();
  void send_enable(uint8_t id);
  void send_set_current_zero(uint8_t id);
  void send_speed_position_control(uint8_t id, float speed, float pos);
  void send_return_to_zero(uint8_t id);
  void send_disable(uint8_t id);
  void send_torque_control(uint8_t id,float torque,float position,float speed,float kp,float kd);
  static uint32_t float_to_uint(float value, float min, float max, uint8_t bits);

private:
  int loop_rate;
  std::string topic_pub;
  std::string serve_pub;
  std::string serial_port;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<cybergear_msg::msg::CybergearState>::SharedPtr state_pub;
  rclcpp::Service<cybergear_msg::srv::CybergearScript>::SharedPtr srv_pub;

  serial::Serial ser;
  uint8_t transmit_buffer[40];
  unsigned long receive_length;

  bool is_srv_on;       //阻塞服务之外的通道
  uint8_t msg_index;
  uint8_t aquire_index;

  std::vector<uint8_t> id;
  std::vector<double>  angle_now, vel_now, torque_now, temp_now;

  cybergear_msg::msg::CybergearState motor_data;

  // 辅助函数
  std::string msg_print(const std::string &head, uint8_t *buffer, unsigned long len);
  bool serialport_init();
  bool serCallback(const std::shared_ptr<cybergear_msg::srv::CybergearScript::Request> req,
                   const std::shared_ptr<cybergear_msg::srv::CybergearScript::Response> res);

  // 定时回调，读取并发布状态
  void publish_state();
  void upload_msg(uint8_t index);
};

} // namespace cybergear

#endif // CYBERGEAR_H
