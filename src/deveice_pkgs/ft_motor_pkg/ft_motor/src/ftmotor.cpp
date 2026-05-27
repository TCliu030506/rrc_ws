#include "../include/ft_motor/ftmotor.h"

#define FTMOTOR_ID1 0X01
#define FTMOTOR_ID2 0X02
#define ZEROPOS_0 500
#define ZEROPOS_1 500


namespace ft_motor {

#ifndef PI
#define PI 3.1415926
#endif


ftmotor::ftmotor() :  Node("ft_motor")
{
  //ftmotor_action = n.advertiseService(serve_pub, &ftmotor::serCallback,this);
  //ftmotor_pub = n.advertise<punc_path_adjust::FTmotorPuncPosition>(topic_pub, 10);
  ftmotor_pub = create_publisher<ft_motor_msg::msg::FTMotor>("ftmotor_msg", 10);
  ftmotor_srv = create_service<ft_motor_msg::srv::FTCommand>("ftmotor_srv", std::bind(&ftmotor::serCallback, this, std::placeholders::_1, std::placeholders::_2));
  declare_parameter<std::string>("ftmotorport","/dev/ttyUSB0");

  ftmotor_write_flag = false;

  reduction_ratio = 300.0/1024; //(24/25) * (8/18) * (300/1024) 

  ftmotor_zeropos[0] = ZEROPOS_0;
  ftmotor_zeropos[1] = ZEROPOS_1;

  for(unsigned long i=0;i<2;i++) ftmotor_pos[i] = 0;

  ftmotor_msg.header.stamp = now();
  ftmotor_msg.header.frame_id = "ft_motor";
  for(int i=0;i<3;i++) ftmotor_msg.motor[i] = 0.0;

  serialport_init();
  thread_obj = std::thread(&ftmotor::pub_thread,this);

}

ftmotor::~ftmotor(){
  thread_obj.join();
}

bool ftmotor::serialport_init()
{
  std::string port;
  port = get_parameter("ftmotorport").as_string();
  try
  { 
	  //设置串口属性，并打开串口 
	  ser.setPort(port); 
	  ser.setBaudrate(1000000); 
	  serial::Timeout to = serial::Timeout::simpleTimeout(25); 
	  ser.setTimeout(to);
	  ser.open(); 
  } 
  catch (serial::IOException& e){ 
    RCLCPP_ERROR(get_logger(), "Unable to open port of FT_motor");
		//ROS_ERROR_STREAM("Unable to open port of FT_motor"); 
		return false; 
	} 

  //检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()){ 
    RCLCPP_INFO(get_logger(), "Serial Port: %s is open!", ftmotor_msg.header.frame_id.c_str());
		//ROS_INFO_STREAM("Serial Port of FT_motor initialized"); 
	} 
	else { 
    RCLCPP_ERROR(get_logger(), "Serial Port: %s is not open!", ftmotor_msg.header.frame_id.c_str());
		return false; 
	} 
  return true;
}

void ftmotor::pub_thread()
{

  //指定循环的频率 
  rclcpp::WallRate naptime(300.0);

  while(rclcpp::ok())
	{ 
    //ask for new data
    if(!ftmotor_write_flag) ftmotor_read_data(FTMOTOR_ID1);
    naptime.sleep();

    //receive data
    receive_data();
    
    //ask for new data
    if(!ftmotor_write_flag) ftmotor_read_data(FTMOTOR_ID2);

    naptime.sleep();

    //receive data
    receive_data();
    

    upload_msg();
  }


};

bool ftmotor::serCallback(const ft_motor_msg::srv::FTCommand::Request::SharedPtr req, const ft_motor_msg::srv::FTCommand::Response::SharedPtr res)
{
  if(strcmp(req->command.c_str(),"MOV") == 0){
    uint16_t pos[2], tim[2], vel[2];
    for(unsigned long i=0;i<2;i++){
      pos[i] = static_cast<uint16_t>(ftmotor_zeropos[i] + req->motor_pos[i] / reduction_ratio);
      if(req->motor_tim[i] < 0.001) tim[i] = 0;
      else tim[i] = static_cast<uint16_t>(req->motor_tim[i]*1000);
      if(req->motor_vel[i] < 0.001) vel[i] = 0;
      else{
        vel[i] = static_cast<uint16_t>((req->motor_vel[i]) / reduction_ratio / 50);
        if(vel[i] == 0) vel[i] = 1;
      }
    }

    //Wait until the serialport is idle
    while(ftmotor_write_flag) usleep(1);

    //ROS_INFO("SYNC WRITE: %04d %04d %04d.",pos[0],pos[1],pos[2]);
    ftmotor_syncwrite_data(pos,tim,vel);
    res->result = "Action OK";
  }
  else if(strcmp(req->command.c_str(),"SET") == 0){
    ftmotor_zeropos[0] = ftmotor_pos[0]/reduction_ratio + ftmotor_zeropos[0];
    ftmotor_zeropos[1] = ftmotor_pos[1]/reduction_ratio + ftmotor_zeropos[1];
    RCLCPP_INFO(get_logger(),"Current Zero-pos: %04d %04d",static_cast<uint16_t>(ftmotor_zeropos[0]),static_cast<uint16_t>(ftmotor_zeropos[1]));
    res->result = "OK";
  }
  else if(strcmp(req->command.c_str(),"FIX") == 0){
    uint16_t pos[2], tim[2] = {2,2}, vel[2] = {0,0};
    pos[0] = static_cast<uint16_t>(ftmotor_zeropos[0]);
    pos[1] = static_cast<uint16_t>(ftmotor_zeropos[1]);
    //Wait until the serialport is idle
    while(ftmotor_write_flag) usleep(1);
    //ROS_INFO("SYNC WRITE: %04d %04d %04d.",pos[0],pos[1],pos[2]);
    ftmotor_syncwrite_data(pos,tim,vel);
    res->result = "Action OK";
  }
  else {return false;}

  return true;
};


uint8_t ftmotor::ftmotor_calculate_crc(uint8_t* pdata, uint8_t start_, uint8_t len_)
{
  uint8_t crc = 0;
  for(uint8_t i=0;i<len_;i++) crc += *(pdata+start_+i);
  return ~crc;
};

void ftmotor::ftmotor_read_data(uint8_t id_)
{
  //block other write serialport thread
  ftmotor_write_flag = true;

  //READ DATA: FF FF ID LEN 02 DIR LEN CRC
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0] = 0XFF;
  transmit_buffer[1] = 0XFF;
  transmit_buffer[2] = id_;
  transmit_buffer[3] = 0X04;
  transmit_buffer[4] = 0X02;  //read
  transmit_buffer[5] = 0X38;  //Current position
  transmit_buffer[6] = 0X02;
  transmit_buffer[7] = ftmotor_calculate_crc(transmit_buffer,2,5);

  //ROS_INFO(msg_print("READ DATA",transmit_buffer,8).c_str());
  ser.write(transmit_buffer,8);

  ftmotor_write_flag = false;
}

void ftmotor::ftmotor_syncwrite_data(uint16_t pos_[2], uint16_t tim_[2], uint16_t vel_[2])
{
  //block other write serialport thread
  ftmotor_write_flag = true;

  //SYNC WRITE DATA: FF FF FE 19 83 2A 06 
  //    ID1 P1H P1L T1H T1L V1H V1L 
  //    ID2 P2H P2L T2H T2L V2H V2L
  //    ID3 P3H P3L T3H T3L V3H V3L CRC
  memset(transmit_buffer,0,sizeof(transmit_buffer));
  transmit_buffer[0]  = 0XFF;
  transmit_buffer[1]  = 0XFF;
  transmit_buffer[2]  = 0XFE;
  transmit_buffer[3]  = 0X12;
  transmit_buffer[4]  = 0X83; //sync write
  transmit_buffer[5]  = 0X2A;
  transmit_buffer[6]  = 0X06;
  transmit_buffer[7]  = FTMOTOR_ID1;
  transmit_buffer[8]  = *(reinterpret_cast<uint8_t*>(&pos_[0])+1);
  transmit_buffer[9]  = *(reinterpret_cast<uint8_t*>(&pos_[0]));
  transmit_buffer[10] = *(reinterpret_cast<uint8_t*>(&tim_[0])+1);
  transmit_buffer[11] = *(reinterpret_cast<uint8_t*>(&tim_[0]));
  transmit_buffer[12] = *(reinterpret_cast<uint8_t*>(&vel_[0])+1);
  transmit_buffer[13] = *(reinterpret_cast<uint8_t*>(&vel_[0]));
  transmit_buffer[14] = FTMOTOR_ID2;
  transmit_buffer[15] = *(reinterpret_cast<uint8_t*>(&pos_[1])+1);
  transmit_buffer[16] = *(reinterpret_cast<uint8_t*>(&pos_[1]));
  transmit_buffer[17] = *(reinterpret_cast<uint8_t*>(&tim_[1])+1);
  transmit_buffer[18] = *(reinterpret_cast<uint8_t*>(&tim_[1]));
  transmit_buffer[19] = *(reinterpret_cast<uint8_t*>(&vel_[1])+1);
  transmit_buffer[20] = *(reinterpret_cast<uint8_t*>(&vel_[1]));
  transmit_buffer[21] = ftmotor_calculate_crc(transmit_buffer,2,19);

  //ROS_INFO("%s",msg_print("SYNC WRITE",transmit_buffer,29).c_str());
  ser.write(transmit_buffer,22);

  ftmotor_write_flag = false;
};

void ftmotor::receive_data()
{
  if(ser.available()){
    //get data
    std_msgs::msg::UInt8MultiArray receive_buffer;
    receive_length = ser.available();
    ser.read (receive_buffer.data, receive_length);
    receive_buffer.data.resize(receive_length);

    //output received data
    //ROS_INFO(msg_print("BACK DATA",&receive_buffer.data[0],receive_length).c_str());

    //split packages
    unsigned long index = 0;
    while((index+8) <= receive_length){
      if(check_ftmotor_datahead(&receive_buffer.data[index])){
        if(update_motormsg(&receive_buffer.data[index+2])) index += 8;
        else index ++;
      }
      else index ++;
    }
  }
};

bool ftmotor::update_motormsg(uint8_t* pstart_)
{
  uint8_t data_[6];
  for(int i=0;i<6;i++) data_[i] = *(pstart_+i);
  if(data_[2] != 0X00) return false;
  if(ftmotor_calculate_crc(data_,0,5) != data_[5]) return false;

  unsigned long motor_id;
  if(data_[0] == FTMOTOR_ID1){
    motor_id = 0;
  }
  else if(data_[0] == FTMOTOR_ID2){
    motor_id = 1;
  }
  else return false;

  uint8_t pos[2];
  pos[0] = data_[4];
  pos[1] = data_[3];
  ftmotor_pos[motor_id] = (static_cast<double>(*(reinterpret_cast<uint16_t*>(pos))) - ftmotor_zeropos[motor_id])*reduction_ratio;
  
  //ROS_INFO("Motor ID %02X: %04d", + static_cast<uint8_t>(motor_id+1),*(reinterpret_cast<uint16_t*>(pos)));

  return true;
};

void ftmotor::upload_msg()
{
  ftmotor_msg.header.stamp = now();

  ftmotor_msg.motor[0] = ftmotor_pos[0];
  ftmotor_msg.motor[1] = ftmotor_pos[1];

  ftmotor_pub->publish(ftmotor_msg);
};

bool ftmotor::check_ftmotor_datahead(uint8_t* pstart_)
{
  if(*pstart_ != 0XFF) return false;
  if(*(pstart_+1) != 0XFF) return false;
  if(*(pstart_+2) == 0XFF) return false;
  return true;
};

} //robotic_vertebra