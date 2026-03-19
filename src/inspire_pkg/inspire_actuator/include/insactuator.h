#ifndef INSACTUATOR_H
#define INSACTUATOR_H

#include <unistd.h>
#include <math.h> 
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "coordinate/msg/array_int16.hpp"
#include "inspire_msg/srv/inspire_script.hpp"

//更新了电机指令以匹配最新的手册

namespace inspire {

// class to communicate with ftmotors
class insactuator : public rclcpp::Node
{

public:

  insactuator();
  ~insactuator();
  
  //read register: pos
  void RD_POS(uint8_t id_);

  //read register: len_read: 1:pos, 2:current, 3:force, 4:raw force, 5:temperature, 6:wrong data
  void RD_MSG(uint8_t id_, uint8_t len_read);

  //read register: 新的读取方式，读取所有寄存器
  void RD_MSG_NEW(uint8_t id_);

  //write register: 设置模式 0：定位，1：伺服，2：速度， 3：力控, 4：速度力控
  void WR_MOD(uint8_t id_, uint16_t mod);

  //write register: 位置模式
  void WR_POS(uint8_t id_, uint16_t pos_in);

  //write register: 伺服模式
  void WR_SRV(uint8_t id_, uint16_t pos_in);

  //write register: 速度模式
  void WR_SPE(uint8_t id_, uint16_t spe_in, uint16_t pos_in);

  //write register: 力控模式
  void WR_FOR(uint8_t id_, uint16_t force_in);

  //write register: 速度力控模式
  void WR_SPE_FOR(uint8_t id_, uint16_t force_in, uint16_t spe_in, uint16_t pos_in);

  //write register: 急停
  void WR_STOP(uint8_t id_);

private:
  
  int loop_rate;
  std::string topic_pub;
  std::string serve_pub;
  std::string serial_port;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Publisher<coordinate::msg::ArrayInt16>::SharedPtr msg_pub;
  rclcpp::Service<inspire_msg::srv::InspireScript>::SharedPtr srv_pub;

  serial::Serial ser;                       //声明串口对象 
  uint8_t transmit_buffer[40];
  unsigned long receive_length;
  
  bool is_srv_on;                           //阻塞服务之外的通道
  uint8_t msg_index;
  uint8_t aquire_index;

  std::vector<uint8_t> id;
  std::vector<int16_t> pos_now;                       //the motor pos
  std::vector<int16_t> force_now;                     //the motor force

  coordinate::msg::ArrayInt16 motor_data;

  std::string msg_print(std::string head, uint8_t * buffer, unsigned long len);

  //open the serial port
  bool serialport_init();

  //srv callback
  bool serCallback(
    const std::shared_ptr<inspire_msg::srv::InspireScript::Request> req, 
    const std::shared_ptr<inspire_msg::srv::InspireScript::Response> res);

  //pub motor_msg thread
  void pub_thread();

  //update motor_msg
  void upload_msg(uint8_t index);

  //calculate crc (jump the datahead and calculate the sum of the rest, output the low 8 bits)
  uint8_t checksum(uint8_t* pdata, uint8_t datalen);

  //receive serialport data, start from 0X2A
  void receive_pos();

  //receive serialport data
  void report_data();
};




} //zigzag_robot

#endif // INSACTUATOR_H