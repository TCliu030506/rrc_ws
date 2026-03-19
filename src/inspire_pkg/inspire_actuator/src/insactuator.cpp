#include "../include/insactuator.h"

//更新了电机指令以匹配最新的手册

namespace inspire {

#ifndef PI
#define PI 3.1415926
#endif

#define INSMOTOR_HEAD_CLIENT_1 0x55
#define INSMOTOR_HEAD_CLIENT_2 0xAA

#define REPORT_INFO 1             //print srv cmd when sended
#define REPORT_MSG  0             //print received msg
#define CHECK_SERIALPORT_DATA   1 //check data
#define SHOW_ERROR              0 //show error when received wrong data
#define SERIAL_WAIT rclcpp::sleep_for(std::chrono::milliseconds(5))
#define MAX_ID 32

insactuator::insactuator() : Node("inspire_actuator")
{
  this->declare_parameter<int>("pub_rate", 50);
  loop_rate = this->get_parameter("pub_rate").as_int();
  RCLCPP_INFO(get_logger(), "Loop_rate: %d", loop_rate);

  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  serial_port = this->get_parameter("port").as_string();
  RCLCPP_INFO(get_logger(), "Serial port: %s", serial_port.c_str());

  this->declare_parameter<std::vector<int64_t>>("id", std::vector<int64_t>{0x01});
  std::vector<int64_t> id__ = this->get_parameter("id").as_integer_array();
  id.clear();
  std::stringstream ss;
  ss << "id: ";
  for(size_t i = 0; i < id__.size(); i++){
    id.push_back(static_cast<uint8_t>(id__[i]));
    ss << "0x" << std::setw(2) << std::setfill('0') << std::hex << id__[i];
    if(i != id__.size()-1) ss << ", ";
  }
  ss << ".";
  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

  topic_pub = "insactuator_msg";
  serve_pub = "insactuator_srv";

  msg_pub = create_publisher<coordinate::msg::ArrayInt16>(topic_pub, 10);
  srv_pub = create_service<inspire_msg::srv::InspireScript>(serve_pub, 
    std::bind(&insactuator::serCallback, this, std::placeholders::_1, std::placeholders::_2));

  is_srv_on = false;
  msg_index = MAX_ID;
  aquire_index = 0;

  pos_now.clear();
  force_now.clear();
  for(unsigned long i=0; i<id.size(); i++){
    pos_now.push_back(1000);
    force_now.push_back(0);
  }

  motor_data.header.stamp = now();
  motor_data.header.frame_id = "inspire_actuator";
  motor_data.name.clear();
  motor_data.data.clear();

  serialport_init();
  int64_t sleep_time = static_cast<int64_t>(1000/(3*loop_rate));
  timer = create_wall_timer(std::chrono::milliseconds(sleep_time),std::bind(&insactuator::pub_thread, this));
}

insactuator::~insactuator(){}

bool insactuator::serialport_init()
{
  try{ 
		//设置串口属性，并打开串口 
		ser.setPort(serial_port); 
		ser.setBaudrate(921600); 
		serial::Timeout to = serial::Timeout::simpleTimeout(100); 
		ser.setTimeout(to);
		ser.open(); 
	} 
	catch (serial::IOException& e){ 
    RCLCPP_ERROR(get_logger(), "Unable to open port of %s.", motor_data.header.frame_id.c_str());
		return false; 
	} 

  //检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()){ 
    RCLCPP_INFO(get_logger(), "Serial Port: %s is open!", motor_data.header.frame_id.c_str());
	} 
	else { 
		return false; 
	} 

  //清空串口数据
  ser.flush();
  for(unsigned long i=0;i<id.size();i++){
    RD_POS(id.at(i));
    receive_pos();
    RCLCPP_INFO(get_logger(), "Initial position for motor id%c: %d.", char('0' + (id.at(i) % 10)), pos_now[i]);
  }

  RCLCPP_INFO(get_logger(), "ROS_SRV: ros2 service call /%s inspire_msg/srv/InspireScript \"{command: ' ', id: , data: [ ]}\"", serve_pub.c_str());
  RCLCPP_INFO(get_logger(), "command: M set mode(m) / P move_pos(p) / X move_srv(p) / V move_vel(p,v) / F move-force(f) / W move_speed&force(p,v,f) / S stop(x)");

  return true;
}

void insactuator::pub_thread()
{
  //读取当前位置数据
  if(!is_srv_on){
    receive_pos();

    //upload msg data
    if(msg_index!=MAX_ID) upload_msg(msg_index);

    if(++aquire_index>=id.size()) aquire_index = 0;

    // RD_POS(id[i]);
    // RD_MSG(id.at(aquire_index),3);
    RD_MSG_NEW(id.at(aquire_index));
  }
};

bool insactuator::serCallback(
  const std::shared_ptr<inspire_msg::srv::InspireScript::Request> req,
  const std::shared_ptr<inspire_msg::srv::InspireScript::Response> res){
  is_srv_on = true;
  ser.flush();
  res->result = "OK";

/*req.command:
  # M: set mode   # P: move_pos   # X: move_srv  # V: move_vel
  # F: move_force # W: move_speed and force      # S: stop 
  # int16[]: m/p, v, f
*/

  switch (req->command.c_str()[0])
  {
  case 'S':
    WR_STOP(req->id);
    break;

  case 'M':
    if(req->data.size()>=1) WR_MOD(req->id,req->data[0]);
    break;

  case 'P':
    if(req->data.size()>=1) WR_POS(req->id,req->data[0]);
    break;

  case 'X':
    if(req->data.size()>=1) WR_SRV(req->id,req->data[0]);
    break;

  case 'V':
    if(req->data.size()>=2) WR_SPE(req->id,req->data[1],req->data[0]);
    break;

  case 'F':
    if(req->data.size()>=1) WR_FOR(req->id,req->data[0]);
    break;

  case 'W':
    if(req->data.size()>=3) WR_SPE_FOR(req->id,req->data[2],req->data[1],req->data[0]);
    break;

  default:
    res->result = std::string("Wrong command to srv: ").append(serve_pub);
    break;
  }
  
  is_srv_on = false;
  return true;
};

std::string insactuator::msg_print(std::string head, uint8_t * buffer, unsigned long len)
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

uint8_t insactuator::checksum(uint8_t* pdata, uint8_t datalen)
{
  uint8_t sum_ = 0x00;
  
  datalen -= 2;
  pdata += 2;

  while (datalen--)
  {
    sum_ += *(pdata++);
  }
  return sum_;
};

void insactuator::RD_POS(uint8_t id_)
{
  //READ DATA: 31
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X04;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X31;  //Read REG
  transmit_buffer[5] = 0X2A;  //dir
  transmit_buffer[6] = 0X00;  
  transmit_buffer[7] = 0X01;
  transmit_buffer[8] = checksum(transmit_buffer,8);

#if REPORT_MSG
  RCLCPP_INFO(get_logger(), "%s", msg_print("READ POS",transmit_buffer,9).c_str());
#endif
  ser.write(transmit_buffer,10);
  SERIAL_WAIT;
};

void insactuator::RD_MSG(uint8_t id_, uint8_t len_read)
{
  //READ DATA: 31
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X04;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X31;  //Read REG
  transmit_buffer[5] = 0X2A;  //dir
  transmit_buffer[6] = 0X00;  
  transmit_buffer[7] = len_read;
  transmit_buffer[8] = checksum(transmit_buffer,8);

#if REPORT_MSG
  RCLCPP_INFO(get_logger(), "%s", msg_print("READ MSG",transmit_buffer,9).c_str());
#endif
  ser.write(transmit_buffer,10);
  SERIAL_WAIT;
};

void insactuator::RD_MSG_NEW(uint8_t id_)
{
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X01;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X30;
  transmit_buffer[5] = checksum(transmit_buffer,8);

#if REPORT_MSG
  RCLCPP_INFO(get_logger(), "%s", msg_print("READ MSG",transmit_buffer,9).c_str());
#endif
  ser.write(transmit_buffer,10);
  SERIAL_WAIT;
};

void insactuator::WR_MOD(uint8_t id_, uint16_t mod)
{
  //WRITE DATA: 32
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X05;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X32;
  transmit_buffer[5] = 0X25;    //dir
  transmit_buffer[6] = 0X00; 
  transmit_buffer[7] = *reinterpret_cast<uint8_t *>(&mod);   //0X25:模式 
  transmit_buffer[8] = *(reinterpret_cast<uint8_t *>(&mod)+1);
  transmit_buffer[9] = checksum(transmit_buffer,9);

#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s",msg_print("MODE SET",transmit_buffer,10).c_str());
#endif
  ser.write(transmit_buffer,11);
  SERIAL_WAIT;
};

void insactuator::WR_POS(uint8_t id_, uint16_t pos_in)
{
  //WRITE DATA: 32
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X0D;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X32;
  transmit_buffer[5] = 0X25;    //dir
  transmit_buffer[6] = 0X00; 
  transmit_buffer[7] = 0X00;    //pos mode
  transmit_buffer[8] = 0X00;
  transmit_buffer[9] = 0X00;    //vol
  transmit_buffer[10] = 0X00;
  transmit_buffer[11] = 0X00;    //force
  transmit_buffer[12] = 0X00;
  transmit_buffer[13] = 0X00;    //speed
  transmit_buffer[14] = 0X00;
  transmit_buffer[15] = *reinterpret_cast<uint8_t *>(&pos_in);   //0X29:位置 
  transmit_buffer[16] = *(reinterpret_cast<uint8_t *>(&pos_in)+1);
  transmit_buffer[17] = checksum(transmit_buffer,17);

#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s",msg_print("MOV_POS",transmit_buffer,18).c_str());
#endif
  ser.write(transmit_buffer,18);
  SERIAL_WAIT;
};

void insactuator::WR_SRV(uint8_t id_, uint16_t pos_in)
{
  //WRITE DATA: 32
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X0D;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X32;
  transmit_buffer[5] = 0X25;    //dir
  transmit_buffer[6] = 0X00; 
  transmit_buffer[7] = 0X01;    //pos mode
  transmit_buffer[8] = 0X00;
  transmit_buffer[9] = 0X00;    //vol
  transmit_buffer[10] = 0X00;
  transmit_buffer[11] = 0X00;    //force
  transmit_buffer[12] = 0X00;
  transmit_buffer[13] = 0X00;    //speed
  transmit_buffer[14] = 0X00;
  transmit_buffer[15] = *reinterpret_cast<uint8_t *>(&pos_in);   //0X29:位置 
  transmit_buffer[16] = *(reinterpret_cast<uint8_t *>(&pos_in)+1);
  transmit_buffer[17] = checksum(transmit_buffer,17);

#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s",msg_print("MOV_POS",transmit_buffer,18).c_str());
#endif
  ser.write(transmit_buffer,18);
  SERIAL_WAIT;
};

void insactuator::WR_SPE(uint8_t id_, uint16_t spe_in, uint16_t pos_in)
{
  //WRITE DATA: 32
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X0D;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X32;
  transmit_buffer[5] = 0X25;    //dir
  transmit_buffer[6] = 0X00; 
  transmit_buffer[7] = 0X02;    //speed mode
  transmit_buffer[8] = 0X00;
  transmit_buffer[9] = 0X00;    //vol
  transmit_buffer[10] = 0X00;
  transmit_buffer[11] = 0X00;    //force
  transmit_buffer[12] = 0X00;
  transmit_buffer[13] = *reinterpret_cast<uint8_t *>(&spe_in);   //0X28:速度 
  transmit_buffer[14] = *(reinterpret_cast<uint8_t *>(&spe_in)+1);
  transmit_buffer[15] = *reinterpret_cast<uint8_t *>(&pos_in);   //0X29:位置 
  transmit_buffer[16] = *(reinterpret_cast<uint8_t *>(&pos_in)+1);
  transmit_buffer[17] = checksum(transmit_buffer,17);

#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s",msg_print("MOV_SPEED",transmit_buffer,18).c_str());
#endif
  ser.write(transmit_buffer,18);
  SERIAL_WAIT;
};

void insactuator::WR_FOR(uint8_t id_, uint16_t force_in)
{
  //WRITE DATA: 32
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X09;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X32;
  transmit_buffer[5] = 0X25;    //dir
  transmit_buffer[6] = 0X00; 
  transmit_buffer[7] = 0X03;    //force mode
  transmit_buffer[8] = 0X00;
  transmit_buffer[9] = 0X00;    //vol
  transmit_buffer[10] = 0X00;
  transmit_buffer[11] = *reinterpret_cast<uint8_t *>(&force_in);   //0X29:位置 
  transmit_buffer[12] = *(reinterpret_cast<uint8_t *>(&force_in)+1);
  transmit_buffer[13] = checksum(transmit_buffer,13);

#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s",msg_print("MOV_FORCE",transmit_buffer,14).c_str());
#endif
  ser.write(transmit_buffer,14);
  SERIAL_WAIT;
};

void insactuator::WR_SPE_FOR(uint8_t id_, uint16_t force_in, uint16_t spe_in, uint16_t pos_in)
{
  //WRITE DATA: 32
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X0D;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X32;
  transmit_buffer[5] = 0X25;    //dir
  transmit_buffer[6] = 0X00; 
  transmit_buffer[7] = 0X04;    //speed & force mode
  transmit_buffer[8] = 0X00; 
  transmit_buffer[9] = 0X00;    //vol
  transmit_buffer[10] = 0X00;
  transmit_buffer[11] = *reinterpret_cast<uint8_t *>(&force_in);    //0X27:力控 
  transmit_buffer[12] = *(reinterpret_cast<uint8_t *>(&force_in)+1);
  transmit_buffer[13] = *reinterpret_cast<uint8_t *>(&spe_in);      //0X28:速度 
  transmit_buffer[14] = *(reinterpret_cast<uint8_t *>(&spe_in)+1);
  transmit_buffer[15] = *reinterpret_cast<uint8_t *>(&pos_in);      //0X29:位置 
  transmit_buffer[16] = *(reinterpret_cast<uint8_t *>(&pos_in)+1);
  transmit_buffer[17] = checksum(transmit_buffer,17);

#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s",msg_print("MOV_SPEED&FORCE",transmit_buffer,18).c_str());
#endif
  ser.write(transmit_buffer,18);
  SERIAL_WAIT;
};

void insactuator::WR_STOP(uint8_t id_)
{
  //WRITE DATA: 32
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = INSMOTOR_HEAD_CLIENT_1;
  transmit_buffer[1] = INSMOTOR_HEAD_CLIENT_2;
  transmit_buffer[2] = 0X05;
  transmit_buffer[3] = id_;
  transmit_buffer[4] = 0X32;
  transmit_buffer[5] = 0X1A;  //dir
  transmit_buffer[6] = 0X00;  
  transmit_buffer[7] = 0X01;
  transmit_buffer[8] = 0X00; 
  transmit_buffer[9] = checksum(transmit_buffer,9);

#if REPORT_INFO
  RCLCPP_INFO(get_logger(), "%s", msg_print("STOP",transmit_buffer,10).c_str());
#endif
  ser.write(transmit_buffer,11);
  SERIAL_WAIT;
};

void insactuator::receive_pos()
{
  if(ser.available()){
    //get data
    std::vector<uint8_t> receive_buffer;
    receive_length = ser.available();
    ser.read (receive_buffer, receive_length);
    receive_buffer.resize(receive_length);
    
    uint8_t len_ = receive_buffer[2] + 4;

    //output received data
#if REPORT_MSG
    RCLCPP_INFO(get_logger(), "%s", msg_print("BACK DATA",&receive_buffer[0],receive_length).c_str());
#endif

#if CHECK_SERIALPORT_DATA
    if(receive_buffer[0] != 0xAA || receive_buffer[1] != 0x55)
    {
#if SHOW_ERROR
      RCLCPP_INFO(get_logger(), "%s", msg_print("WRONG DATA HEAD:",&receive_buffer[0],2).c_str());
#endif
      return;
    }
    if(receive_buffer[len_]!=checksum(&receive_buffer[0],len_))
    {
#if SHOW_ERROR
      RCLCPP_INFO(get_logger(), "%s", msg_print("WRONG DATA CHECK_SUM:",&receive_buffer[len_],1).c_str());
#endif
      return;
    }
#endif
    
    //get id
    uint8_t id_ = receive_buffer[3];

    //get pos
    unsigned long i;
    for(i=0;i<id.size();i++) if(id_ == id.at(i)) break;
    if(i<id.size()) {
      //pos_now[i] = *reinterpret_cast<int16_t *>(&receive_buffer[7]);
      int16_t actual_pos = static_cast<int16_t>((receive_buffer[10] << 8) | receive_buffer[9]);
      pos_now[i] = actual_pos;
      if(len_>=15) {
      //force_now[i] = *reinterpret_cast<int16_t *>(&receive_buffer[11]);
        int16_t force_value = static_cast<int16_t>((receive_buffer[14] << 8) | receive_buffer[13]);
        force_now[i] = force_value;
      }
      msg_index = i;
    }
    else msg_index = MAX_ID;
  }
};

void insactuator::report_data()
{
  if(ser.available()){
    //get data
    std::vector<uint8_t> receive_buffer;
    receive_length = ser.available();
    ser.read (receive_buffer, receive_length);
    receive_buffer.resize(receive_length);

    //output received data
    RCLCPP_INFO(get_logger(), "%s", msg_print("REPORT DATA",&receive_buffer[0],receive_length).c_str());
  }
};

void insactuator::upload_msg(uint8_t index)
{
  motor_data.header.stamp = now();
  std::string frame_id = "inspire_actuator_id";
  frame_id.push_back(char('0' + (id.at(index) % 10)));
  motor_data.header.frame_id = frame_id.c_str();
  motor_data.name.clear();
  motor_data.name.push_back("postion");
  motor_data.name.push_back("force");
  motor_data.data.clear();
  motor_data.data.push_back(pos_now[index]);
  motor_data.data.push_back(force_now[index]);

  msg_pub->publish(motor_data);
  msg_index = MAX_ID;
};



} //zigzag_robot