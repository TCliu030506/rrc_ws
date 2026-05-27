#include "../include/mtactuator.hpp"

namespace mtactuator {

#ifndef PI
#define PI 3.14159265359
#endif


MtActuator::MtActuator(int loop_rate_in, std::string topic_pub_in, std::string serve_pub_in) 
  : Node("mt_actuator"), loop_rate(loop_rate_in), topic_pub(topic_pub_in),  serve_pub(serve_pub_in)
{

  msg_pub = create_publisher<mtactuator_msg::msg::MtPosition>("/mt/position", 10);
  srv_pub = create_service<mtactuator_msg::srv::MtAction>(serve_pub, 
    std::bind(&MtActuator::serCallback, this, std::placeholders::_1, std::placeholders::_2));

  serialport_init();

  int64_t sleep_time = static_cast<int64_t>(1000/loop_rate);
  timer = create_wall_timer(std::chrono::milliseconds(sleep_time),std::bind(&MtActuator::pub_thread, this));
  
  pos_0 = ReadAngle();
  RCLCPP_INFO(get_logger(), "POS_0 = %ld", pos_0);

  rclcpp::on_shutdown([this]() {
    RCLCPP_INFO(this->get_logger(), "ROS2 shutdown detected, flushing and closing serial port...");
    if (ser.isOpen()) {
      ser.flush();
      ser.close();
    }
  });

}

MtActuator::~MtActuator(){
  RCLCPP_INFO(get_logger(), "Node shutting down, cleaning up serial port...");
  try {
    if (ser.isOpen()) {
      ser.flush();
      ser.close();
      RCLCPP_INFO(get_logger(), "Serial port closed successfully.");
    }
  } catch (serial::IOException &e) {
    RCLCPP_WARN(get_logger(), "Error while closing serial port: %s", e.what());
  }
}

// 读取多圈角度命令
int MtActuator::ReadAngle()
{
  ser.flush();
  
  uint8_t TxData[17] = {0};
  
  TxData[0] = 0x41;
  TxData[1] = 0x54;

  TxData[2] = 0x28;
  TxData[3] = 0x20;
  TxData[4] = 0x00;
  TxData[5] = 0x00;
  TxData[6] = 0x08;

  TxData[7] = 0x92;
  TxData[8] = 0x00;
  TxData[9] = 0x00;
  TxData[10] = 0x00;
  TxData[11] = 0x00; 
  TxData[12] = 0x00;
  TxData[13] = 0x00;
  TxData[14] = 0x00;

  TxData[15] = 0x0D;
  TxData[16] = 0x0A;
        
  RCLCPP_INFO(get_logger(), "%s", msg_print("TxData", TxData, 17).c_str());
  ser.write(TxData, 17);
  rclcpp::sleep_for(std::chrono::milliseconds(50));

  uint8_t RxData[17] = {0};
  size_t read_len = ser.read(RxData, 17);

  if (read_len != 17) {
    RCLCPP_WARN(get_logger(), "ReadAngle: receive length error, len = %zu (expect 17)", read_len);
    return 0;
  }
  if (RxData[0] != 0x41 || RxData[1] != 0x54 || RxData[15] != 0x0D || RxData[16] != 0x0A) {
    RCLCPP_WARN(get_logger(),
                "ReadAngle: frame header/tail error: %s",
                msg_print("BACK DATA", RxData, 17).c_str());
    return 0;
  }

  RCLCPP_INFO(get_logger(), "%s", msg_print("BACK DATA", RxData, 17).c_str());

  int32_t angle =
      (uint32_t)RxData[14] << 24 |
      (uint32_t)RxData[13] << 16 |
      (uint32_t)RxData[12] << 8  |
      (uint32_t)RxData[11];

  return angle;
}

// Move actuator
void MtActuator::Move(uint8_t mode, uint16_t maxSpeed, int64_t angleControl, int32_t speed)
{

  switch (mode) {
    case 1:
        AngleControl_add(maxSpeed, angleControl);
        break;
    case 2:
        SpeedControl(speed);
        break; 
    case 3:
        AngleControl_absolute(maxSpeed, angleControl);
        break;
    case 4:
        AngleControl_track(maxSpeed, angleControl);
        break;
    case 5:
        Close();
        break;
    default: 
        // 
        break;
  }
}

// 增量模式-MODE1
void MtActuator::AngleControl_add(uint16_t maxSpeed, int64_t angleControl)
{
  
  uint8_t TxData[17] = {0};
  TxData[0] = 0x41;
  TxData[1] = 0x54;

  TxData[2] = 0x28;
  TxData[3] = 0x20;
  TxData[4] = 0x00;
  TxData[5] = 0x00;
  TxData[6] = 0x08;

  TxData[7] = 0xA8;
  TxData[8] = 0x00;
  TxData[9] = *(uint8_t *)(&maxSpeed);
  TxData[10] = *((uint8_t *)(&maxSpeed)+1);
  TxData[11] = *(uint8_t *)(&angleControl); 
  TxData[12] = *((uint8_t *)(&angleControl)+1);
  TxData[13] = *((uint8_t *)(&angleControl)+2);
  TxData[14] = *((uint8_t *)(&angleControl)+3);

  TxData[15] = 0x0D;
  TxData[16] = 0x0A;
        
  RCLCPP_INFO(get_logger(), "%s", msg_print("TxData", TxData, 17).c_str());
  ser.write(TxData, 17);
  rclcpp::sleep_for(std::chrono::milliseconds(50));
}

// 速度控制-MODE2
void MtActuator::SpeedControl(int32_t maxSpeed)
{
  
  uint8_t TxData[17] = {0};
  TxData[0] = 0x41;
  TxData[1] = 0x54;

  TxData[2] = 0x28;
  TxData[3] = 0x20;
  TxData[4] = 0x00;
  TxData[5] = 0x00;
  TxData[6] = 0x08;

  TxData[7] = 0xA2;
  TxData[8] = 0x00;
  TxData[9] = 0x00;
  TxData[10] = 0x00;
  TxData[11] = *(uint8_t *)(&maxSpeed); 
  TxData[12] = *((uint8_t *)(&maxSpeed)+1);
  TxData[13] = *((uint8_t *)(&maxSpeed)+2);
  TxData[14] = *((uint8_t *)(&maxSpeed)+3);

  TxData[15] = 0x0D;
  TxData[16] = 0x0A;
        
  //RCLCPP_INFO(get_logger(), "%s", TxData);
  RCLCPP_INFO(get_logger(), "%s", msg_print("TxData", TxData, 17).c_str());
  ser.write(TxData, 17);
  rclcpp::sleep_for(std::chrono::milliseconds(50));
}

// 多圈模式-MODE3
void MtActuator::AngleControl_absolute(uint16_t maxSpeed, int64_t angleControl)
{
  
  uint8_t TxData[17] = {0};
  angleControl += pos_0;
 
  TxData[0] = 0x41;
  TxData[1] = 0x54;

  TxData[2] = 0x28;
  TxData[3] = 0x20;
  TxData[4] = 0x00;
  TxData[5] = 0x00;
  TxData[6] = 0x08;

  TxData[7] = 0xA4;
  TxData[8] = 0x00;
  TxData[9] = *(uint8_t *)(&maxSpeed);
  TxData[10] = *((uint8_t *)(&maxSpeed)+1);
  TxData[11] = *(uint8_t *)(&angleControl); 
  TxData[12] = *((uint8_t *)(&angleControl)+1);
  TxData[13] = *((uint8_t *)(&angleControl)+2);
  TxData[14] = *((uint8_t *)(&angleControl)+3);

  TxData[15] = 0x0D;
  TxData[16] = 0x0A;
        
  RCLCPP_INFO(get_logger(), "%s", msg_print("TxData", TxData, 17).c_str());
  ser.write(TxData, 17);
  rclcpp::sleep_for(std::chrono::milliseconds(50));
}

// 跟踪模式-MODE4
void MtActuator::AngleControl_track(uint16_t maxSpeed, int64_t angleControl)
{
  
  uint8_t TxData[17] = {0};

  angleControl += pos_0; 
  TxData[0] = 0x41;
  TxData[1] = 0x54;

  TxData[2] = 0x28;
  TxData[3] = 0x20;
  TxData[4] = 0x00;
  TxData[5] = 0x00;
  TxData[6] = 0x08;

  TxData[7] = 0xA5;
  TxData[8] = 0x00;
  TxData[9] = *(uint8_t *)(&maxSpeed);
  TxData[10] = *((uint8_t *)(&maxSpeed)+1);
  TxData[11] = *(uint8_t *)(&angleControl); 
  TxData[12] = *((uint8_t *)(&angleControl)+1);
  TxData[13] = *((uint8_t *)(&angleControl)+2);
  TxData[14] = *((uint8_t *)(&angleControl)+3);
        
  TxData[15] = 0x0D;
  TxData[16] = 0x0A;
        
  RCLCPP_INFO(get_logger(), "%s", msg_print("TxData", TxData, 17).c_str());
  ser.write(TxData, 17);
  rclcpp::sleep_for(std::chrono::milliseconds(50));
}

// 关闭电机
void MtActuator::Close()
{
  
  uint8_t TxData[17] = {0};

  TxData[0] = 0x41;
  TxData[1] = 0x54;

  TxData[2] = 0x28;
  TxData[3] = 0x20;
  TxData[4] = 0x00;
  TxData[5] = 0x00;
  TxData[6] = 0x08;

  TxData[7] = 0x80;
  TxData[8] = 0x00;
  TxData[9] = 0x00;
  TxData[10] = 0x00;
  TxData[11] = 0x00; 
  TxData[12] = 0x00;
  TxData[13] = 0x00;
  TxData[14] = 0x00;
        
  TxData[15] = 0x0D;
  TxData[16] = 0x0A;
        
  RCLCPP_INFO(get_logger(), "%s", msg_print("TxData", TxData, 17).c_str());
  ser.write(TxData, 17);
  rclcpp::sleep_for(std::chrono::milliseconds(50));
}

bool MtActuator::serialport_init()
{
  try{ 
		//设置串口属性，并打开串口 
		ser.setPort("/dev/ttyUSB0"); 
		ser.setBaudrate(115200); 
		serial::Timeout to = serial::Timeout::simpleTimeout(100); 
		ser.setTimeout(to);
		ser.open(); 
	} 
	catch (serial::IOException& e){ 
    RCLCPP_ERROR(get_logger(), "Unable to open port!");
		return false; 
	} 

  //检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()){ 
    RCLCPP_INFO(get_logger(), "Serial Port is open!");
	} 
	else { 
		return false; 
	} 

  //清空串口数据
  ser.flush();
  

  RCLCPP_INFO(get_logger(), "ROS_SRV: ros2 service call /mtactuator_srv mtactuator_msg/srv/MtAction  \"{mode: 1, maxspeed: 100 , pos: 1000}\"");
  RCLCPP_INFO(get_logger(), "\nMODE1-增量模式输入: 速度限制maxspeed(单位dps)、角度值pos(单位0.01deg)\nMODE2-旋转速度输入: 速度限制maxspeed(单位0.01dps)\nMODE3-绝对位置输入: 速度限制maxspeed(单位dps)、角度值pos(单位0.01deg)\nMODE4-绝对位置跟踪: 速度限制maxspeed(单位dps)、角度值pos(单位0.01deg)");

  return true;
}

std::string MtActuator::msg_print(std::string head, uint8_t * buffer, unsigned long len)
{
  std::ostringstream msg;
  msg << head;
  msg << "-[";
  for(unsigned long i=0;i<len;i++) msg << std::hex << std::setw(2) << std::setfill('0') << + *(buffer+i) << " ";
  std::string msg_out = msg.str();
  msg_out.pop_back();
  msg_out.append("]");
  return msg_out;
};

void MtActuator::receive_pos()
{
  // 若当前缓冲区中至少有一帧数据，可以尝试读取一整帧
  if (ser.available() >= 17) {

    uint8_t RxData[17] = {0};
    size_t read_len = ser.read(RxData, 17);

    if (read_len != 17) {
      RCLCPP_WARN(get_logger(), "receive_pos: receive length error, len = %zu (expect 17)", read_len);
      return;
    }

    // 帧头帧尾校验
    if (RxData[0] != 0x41 || RxData[1] != 0x54 || RxData[15] != 0x0D || RxData[16] != 0x0A) {
      RCLCPP_WARN(get_logger(),
                  "receive_pos: frame header/tail error: %s",
                  msg_print("BACK DATA", RxData, 17).c_str());
      return;
    }

    // 打印整帧
    RCLCPP_INFO(get_logger(), "%s", msg_print("BACK DATA", RxData, 17).c_str());
  }
};

void MtActuator::pub_thread()
{
  // 读取当前位置数据
  int32_t current_pos = ReadAngle();

  // 发布消息
  upload_msg(current_pos);
}

void MtActuator::upload_msg(int32_t pos)
{
  mtactuator_msg::msg::MtPosition msg;

  // 填充消息头
  msg.header.stamp = now();
  msg.header.frame_id = "mt_actuator";

  // 填充电机位置信息 int能 .clear() 或 .push_back() float 只能赋值
  // msg.position.clear();
  // msg.position.push_back(pos * 0.01);
  msg.position = pos * 0.01;

  // 发布话题
  msg_pub->publish(msg);

  RCLCPP_INFO(get_logger(), "Published position: %.2f (raw pos = %d)", msg.position, pos);
}

bool MtActuator::serCallback(
  const std::shared_ptr<mtactuator_msg::srv::MtAction::Request> req, 
  const std::shared_ptr<mtactuator_msg::srv::MtAction::Response> res){
  
  ser.flush();
  res->result = "OK";

  Move(req->mode, req->maxspeed, req->pos, req->speed);
  
  return true;
}
  
}; //mtactuator