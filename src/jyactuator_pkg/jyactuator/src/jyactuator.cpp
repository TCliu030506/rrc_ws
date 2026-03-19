#include "../include/jyactuator.hpp"

#define FTMOTOR_ID1 0X01

namespace jyactuator {

#ifndef PI
#define PI 3.1415926
#endif

#define REPORT_INFO 0

void InvertUint16(unsigned short *dBuf, unsigned short *srcBuf) {
  int i;
  unsigned short tmp[4] = {0};

  for(i=0;i<16;i++)
  {
    if(srcBuf[0] & (1<<i))
    tmp[0] |= 1<<(15-i);
  }
  dBuf[0] = tmp[0];
}

std::string msg_print(std::string head, uint8_t *buffer, unsigned long len) {
  std::ostringstream msg;
  msg << head;
  msg << "-[";
  for(unsigned long i=0;i<len;i++) msg << std::hex << std::setw(2) << std::setfill('0') << + *(buffer+i) << " ";
  std::string msg_out = msg.str();
  msg_out.pop_back();
  msg_out.append("]");
  return msg_out;
}

JyActuator::JyActuator(uint8_t id_in, int loop_rate_in, std::string topic_pub_in, std::string serve_pub_in)
    : Node("jyactuator_node"), id(id_in), loop_rate(loop_rate_in), topic_pub(topic_pub_in), serve_pub(serve_pub_in) {
  jyactuator_act = create_service<jyactuator_msg::srv::JyAction>(serve_pub, std::bind(&JyActuator::serCallback, this, std::placeholders::_1, std::placeholders::_2));
  jyactuator_pub = create_publisher<jyactuator_msg::msg::JyPosition>(topic_pub, 10);

  pub_flag = false;
  is_service_on = false;

  jy_pos = 300;
  jy_pos_goal = jy_pos;
  jy_pos_plan = jy_pos_goal;
  jy_step = 10.0 / loop_rate;

  pos_data.header.stamp = now();
  // pos_data.header.seq = 0;
  pos_data.header.frame_id = "jy_motor";

  serialport_init();

  int64_t sleep_time = static_cast<int64_t>(1000/loop_rate);
  timer = create_wall_timer(std::chrono::milliseconds(sleep_time),std::bind(&JyActuator::pub_thread, this));
}

JyActuator::~JyActuator() {}

// 初始化串口连接，配置串口路径、波特率等参数，并打开串口连接
bool JyActuator::serialport_init() {
  try {
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port of jy_motor");
    return false;
  }

  if (ser.isOpen()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Serial Port of JY_Actuator initialized");
  } else {
    return false;
  }

  return true;
}

// 校验
unsigned short JyActuator::CRC16_MODBUS(uint8_t *pdata, uint8_t datalen) {
  unsigned short wCRCin = 0xFFFF;
  unsigned short wCPoly = 0x8005;

  InvertUint16(&wCPoly, &wCPoly);
  while (datalen--)
  {
    wCRCin ^= *(pdata++);
    for(int i=0;i<8;i++)
    {
      if(wCRCin & 0x01)
        wCRCin = (wCRCin >> 1) ^ wCPoly;
      else
        wCRCin = wCRCin >> 1;
    }
  }
  return (wCRCin);
}

// 读取电机数据的命令到串口，并将获取的数据保存
void JyActuator::read_data() {
  pub_flag = true;

  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = id;
  transmit_buffer[1] = 0X04;
  transmit_buffer[2] = 0X00;
  transmit_buffer[3] = 0X02;
  transmit_buffer[4] = 0X00;
  transmit_buffer[5] = 0X01;
  *reinterpret_cast<unsigned short *>(&transmit_buffer[6]) = CRC16_MODBUS(transmit_buffer, 6);

#if REPORT_INFO
  RCLCPP_INFO(this->get_logger(), msg_print("READ DATA", transmit_buffer, 8).c_str());
#endif
  ser.write(transmit_buffer, 8);

  pub_flag = false;
}

// 读取串口接收缓冲区中的数据，并解析出电机位置信息
void JyActuator::receive_data() {
  if (ser.available()) {
    std::vector<uint8_t> receive_buffer;
    receive_length = ser.available();
    ser.read(receive_buffer, receive_length);
    receive_buffer.resize(receive_length);

#if REPORT_INFO
    RCLCPP_INFO(this->get_logger(), msg_print("BACK DATA", &receive_buffer.data[0], receive_length).c_str());
#endif

    uint8_t pos_[2];
    pos_[0] = receive_buffer[4];
    pos_[1] = receive_buffer[3];
    jy_pos = *reinterpret_cast<uint16_t *>(&pos_);
  }
}

// 读取串口接收缓冲区中的数据     // std::vector<uint8_t> receive_buffer;
void JyActuator::report_data() {
  if (ser.available()) {
    std::vector<uint8_t> receive_buffer;
    receive_length = ser.available();
    ser.read(receive_buffer, receive_length);
    receive_buffer.resize(receive_length);

#if REPORT_INFO
    RCLCPP_INFO(this->get_logger(), msg_print("REPORT DATA", &receive_buffer.data[0], receive_length).c_str());
#endif
  }
}

// 将电机位置信息打包成 ROS2 消息并发布
void JyActuator::upload_msg() {
  // pos_data.header.seq++;
  pos_data.header.stamp = now();

  pos_data.id = static_cast<int8_t>(id);
  pos_data.pos = jy_pos;

  jyactuator_pub->publish(pos_data);
}

// 发送电机移动命令到串口，控制电机运动到指定位置
void JyActuator::mov_cmd(uint16_t pos_in) {
  pub_flag = true;

  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  transmit_buffer[0] = id;
  transmit_buffer[1] = 0X06;
  transmit_buffer[2] = 0X00;
  transmit_buffer[3] = 0X02;
  transmit_buffer[4] = *(reinterpret_cast<uint8_t *>(&pos_in) + 1);
  transmit_buffer[5] = *reinterpret_cast<uint8_t *>(&pos_in);
  *reinterpret_cast<unsigned short *>(&transmit_buffer[6]) = CRC16_MODBUS(transmit_buffer, 6);

#if REPORT_INFO
  RCLCPP_INFO(this->get_logger(), "%s", msg_print("Motion", transmit_buffer, 8).c_str());
#endif
  ser.write(transmit_buffer, 8);

  pub_flag = false;
}

// 实现电机的运动规划，根据设定的目标位置和步长逐渐调整电机位置
void JyActuator::motion() {
  double bias_ = jy_pos_goal - jy_pos;
  if (fabs(bias_) <= jy_step) {
    jy_pos_plan = jy_pos_goal;
    jy_step = 2.0 / loop_rate;
  } else {
    if (jy_pos < jy_pos_goal)
      jy_pos_plan += jy_step;
    else
      jy_pos_plan -= jy_step;
  }
  mov_cmd(static_cast<uint16_t>(round(jy_pos_plan)));
}

// 回调函数
bool JyActuator::serCallback(const std::shared_ptr<jyactuator_msg::srv::JyAction::Request> req,
                             const std::shared_ptr<jyactuator_msg::srv::JyAction::Response> res) {
  // 收到了服务请求，设置标志为true
  // is_service_on = false;
  if (req->pos < 120 || req->pos > 480 || req->tim < 0.01) {
    jy_pos_goal = jy_pos;
    res->result = "Stop";
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal set to: %d.", jy_pos_goal);
    jy_pos_goal = req->pos;
    jy_step = static_cast<double>(abs(jy_pos_goal - jy_pos)) / req->tim / loop_rate;
    
    res->result = "Move";
  }
  rclcpp::Rate naptime(loop_rate * 2);
  read_data();
  naptime.sleep();
  receive_data();
  // jy_pos_goal = jy_pos;
  
  mov_cmd(static_cast<uint16_t>(round(jy_pos_goal)));
  naptime.sleep();
  read_data();
  naptime.sleep();
  receive_data();
  upload_msg();
  /*
  if (jy_pos_goal - jy_pos >= 1){
    mov_cmd(static_cast<uint16_t>(round(jy_pos_goal+1)));
  }
  if (jy_pos_goal - jy_pos <= -1){
    mov_cmd(static_cast<uint16_t>(round(jy_pos_goal-1)));
  }
  */
  naptime.sleep();
  report_data();

  if (!pub_flag)
    read_data();
  naptime.sleep();

  receive_data();
  upload_msg();
  // is_service_on = false;

  RCLCPP_INFO(this->get_logger(), "Current: %d, Goal:%d", jy_pos, jy_pos_goal);
  
  return true;
}

// 周期性地执行电机控制、数据读取和消息发布等操作
void JyActuator::pub_thread() {
  
  rclcpp::Rate naptime(loop_rate * 2);

  read_data();
  naptime.sleep();
  receive_data();
  jy_pos_goal = jy_pos;
  // RCLCPP_INFO(this->get_logger(), "Current: %d, Goal:%d", jy_pos, jy_pos_goal);
  
}

} // namespace jyactuator

