#ifndef FTMOTOR_H
#define FTMOTOR_H

#include "rclcpp/rclcpp.hpp"

#include "ft_motor_msg/msg/ft_motor.hpp"
#include "ft_motor_msg/srv/ft_command.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <serial/serial.h>

#include <unistd.h>
//#include <math.h> 
#include <string>
//#include <iostream>
//#include <iomanip>
//#include <fstream>
//#include <sstream>


namespace ft_motor {

// class to communicate with ftmotors
class ftmotor: public rclcpp::Node
{
private:
  

  int loop_rate;

  serial::Serial ser;                                   //声明串口对象 
  uint8_t transmit_buffer[40];
  unsigned long receive_length;

  std::thread thread_obj;

  rclcpp::Publisher<ft_motor_msg::msg::FTMotor>::SharedPtr ftmotor_pub;
  rclcpp::Service<ft_motor_msg::srv::FTCommand>::SharedPtr ftmotor_srv;
  //ros::Publisher ftmotor_pub;
  //ros::ServiceServer ftmotor_action;

  bool ftmotor_write_flag;

  ft_motor_msg::msg::FTMotor ftmotor_msg;    

  double ftmotor_zeropos[2];                          //the motor pos when in 0 deg
  double ftmotor_pos[2];                                //Current pos data / deg

  double reduction_ratio;


public:
  ftmotor();
  ~ftmotor();

  //open the serial port
  bool serialport_init();

  //srv callback
  bool serCallback(const ft_motor_msg::srv::FTCommand::Request::SharedPtr req, const ft_motor_msg::srv::FTCommand::Response::SharedPtr res);

  //pub motor_msg thread
  void pub_thread();

  //update motor_msg
  void upload_msg();

  //calculate crc
  uint8_t ftmotor_calculate_crc(uint8_t* pdata, uint8_t start_, uint8_t len_);
  
  //ftmotor cmd: read data
  void ftmotor_read_data(uint8_t id_);

  //ftmotor cmd: sync write
  void ftmotor_syncwrite_data(uint16_t pos_[2], uint16_t tim_[2], uint16_t vel_[2]);

  //receive serialport data
  void receive_data();

  //check ftmotor data head
  bool check_ftmotor_datahead(uint8_t* pstart_);

  //update motor pos
  bool update_motormsg(uint8_t* pstart_);
};

} //robotic_vertebra

#endif // FTMOTOR_1_H