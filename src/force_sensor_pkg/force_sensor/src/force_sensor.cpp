#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <unistd.h>
#include <std_msgs/msg/string.hpp> 
#include <std_msgs/msg/empty.hpp> 
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "force_sensor_msg/msg/three_axis_force.hpp"
#include "force_sensor_msg/srv/serialport.hpp"
#include <math.h> 
#include <string>

#define PI 3.1415926

namespace force_sen {

class serialport: public rclcpp::Node
{
private:

  std::string topic_pub;
  std::string serve_pub;

  bool pub = false;

  serial::Serial ser; //声明串口对象 
  uint8_t s_buffer[50];
  unsigned long p;
	int loop_rate;

  rclcpp::Publisher<force_sensor_msg::msg::ThreeAxisForce>::SharedPtr forcesensor_pub;
  rclcpp::Service<force_sensor_msg::srv::Serialport>::SharedPtr command_ser;
  std::thread thread_obj;

  force_sensor_msg::msg::ThreeAxisForce force_data;

  std_msgs::msg::UInt8MultiArray msg;

  bool get_pubrate();

  void get_instructions();

  void set_GOD();

  void set_ADJZF(char adjzf);

  void ask_ADJZF();

  void set_SMPR(std::string smpr);

  void ask_SMPR();

  void set_SGDM(std::string sgdm);

  void ask_SGDM();

  void recieve();

  bool check_data(std_msgs::msg::UInt8MultiArray &data);

  bool wash_data(std_msgs::msg::UInt8MultiArray &data, unsigned long num);

public:

  serialport(std::string topic_pub_in, std::string serve_pub_in);
  ~serialport();

  bool init();

  void pub_thread();

  bool serCallback(const std::shared_ptr<force_sensor_msg::srv::Serialport::Request> req, const std::shared_ptr<force_sensor_msg::srv::Serialport::Response> res);

};

serialport::serialport(std::string topic_pub_in, std::string serve_pub_in) :
  Node("force_sensor_node"), topic_pub(topic_pub_in),  serve_pub(serve_pub_in)
{
  command_ser = create_service<force_sensor_msg::srv::Serialport>(serve_pub, std::bind(&serialport::serCallback, this, std::placeholders::_1, std::placeholders::_2));
  forcesensor_pub = create_publisher<force_sensor_msg::msg::ThreeAxisForce>(topic_pub, 10);

  thread_obj=std::thread(&serialport::pub_thread, this);

  declare_parameter<int>("forcesensor_rate",100);
  declare_parameter<std::string>("forcesensorport","/dev/ttyUSB0");

  force_data.header.stamp = now();
  force_data.header.frame_id = "base_frame";

  force_data.x_axis = 0.0;
  force_data.y_axis = 0.0;
  force_data.z_axis = 0.0;
}

serialport::~serialport()
{
  thread_obj.join();
}

bool serialport::init()
{
  get_pubrate();
  std::string sensor_port;
  get_parameter("forcesensorport",sensor_port);
  try{ 
		//设置串口属性，并打开串口 
		ser.setPort(sensor_port); 
		ser.setBaudrate(115200); 
		serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
		ser.setTimeout(to);
		ser.open(); 
	} 
	catch (serial::IOException& e){ 
		RCLCPP_ERROR(get_logger(),"Unable to open port "); 
		return false; 
	} 

	//检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()){ 
		RCLCPP_INFO(get_logger(),"Serial Port initialized"); 
	} 
	else { 
		return false; 
	} 

  //set zero
  set_ADJZF('1');
  sleep(1);
  recieve();

  get_instructions();

  pub = true;

  return true;
}

bool serialport::get_pubrate()
{
  get_parameter("forcesensor_rate",loop_rate);
  RCLCPP_INFO(get_logger(),"Got looprate: %d",loop_rate);
  
  return true;
}

void serialport::get_instructions()
{
  std::cout << "Waiting command: \n";
  std::cout << "  GOD(return data once)\n";
  std::cout << "  ADJZF(set zero)\n   -[send: 0 / 1 / 2]\n";
  std::cout << "  SMPR(return sample rate)\n   -[send: 10 ~ 200]\n";
  std::cout << "  SGDM(return data upload format)\n   -[send: (A01,A02,A03);E;1;(WMA:1)]\n";

  std::cout << "Format: rosservice call /" << serve_pub << " \"callname: '_'\ncommand: '_'\"\n";
}

bool serialport::check_data(std_msgs::msg::UInt8MultiArray &data)
{
  bool pass = true;

  //0xAA
  if(static_cast<int>(data.data[0])!=0xaa) pass = false;
  //0x55
  if(static_cast<int>(data.data[1])!=0x55) pass = false;
  //len
  unsigned long pack_len = data.data[2] * 256 + data.data[3];
  if(p != (pack_len+4)) pass = false;

  if(!pass) RCLCPP_ERROR(get_logger(),"Invalid data!");
  return pass;
}

bool serialport::wash_data(std_msgs::msg::UInt8MultiArray &data, unsigned long num)
{
  unsigned long i = 0;
  bool data_ = false, msg_ = false;
  while((!msg_ || !data_ ) && i<num){
    if(static_cast<int>(data.data[i])==0xaa) data_ = true;
    if(static_cast<int>(data.data[i])==0x41) msg_ = true;
    i++;
  }

  if(i>=num){
    i = 1;
    printf("Data miss\n");
  }

  if(data_ || msg_){
    msg.data.clear();
    for(unsigned long j=i-1; j<num; j++){
      msg.data.push_back(data.data[j]);
    }
  }

  return true;

}

void serialport::pub_thread()
{
	
	//指定循环的频率 
	rclcpp::WallRate naptime(loop_rate);

	while(rclcpp::ok())
	{ 
		while(!pub){
      sleep(1);
    }

    set_GOD();

		//save data para
    unsigned long channel = 3;
    std::vector<float> f_data;
    for(unsigned long i=0; i<channel; i++) f_data.push_back(0);
		uint8_t data_get[4]; 
    unsigned long data_locate = 6;

		//read
		if(ser.available()){ 
			//get data
			std_msgs::msg::UInt8MultiArray r_buffer;
			std_msgs::msg::UInt8MultiArray serial_data;
			p = ser.available();
			ser.read (serial_data.data, p);

			//save data
			r_buffer.data.resize(p);
			//ROS_INFO_STREAM("Reading:");
      for(unsigned long i=0; i<p; i++){
        r_buffer.data[i]=serial_data.data[i];
				//printf("%02x ",r_buffer.data[i]);
      }
			//cout << endl;

			if(check_data(r_buffer)){
				//translate data
				force_data.header.stamp = now();

        for(unsigned long i=0; i<channel; i++){
          for(unsigned long j=0;j<4;j++){
						data_get[j] = r_buffer.data[data_locate+4*i+j];
					}
          //f_data[i] = *(float*)(&data_get[0]);
          f_data[i] = *reinterpret_cast<float*>(data_get);
				}
		
        force_data.x_axis = static_cast<double>(f_data[0]);
        force_data.y_axis = static_cast<double>(f_data[1]);
        force_data.z_axis = static_cast<double>(f_data[2]);
				
				forcesensor_pub->publish(force_data); 
			}
			
		} 

		naptime.sleep(); 	
	}
}

void serialport::recieve()
{
  if(ser.available()){
    std_msgs::msg::UInt8MultiArray r_buffer;
    std_msgs::msg::UInt8MultiArray serial_data;
    p = ser.available();
    ser.read (serial_data.data, p);

    //save data
    r_buffer.data.resize(p);
    //ROS_INFO_STREAM("Reading:");
    for(unsigned long i=0; i<p; i++){
      r_buffer.data[i]=serial_data.data[i];
    }

    if(wash_data(r_buffer, p)){
      for(unsigned long i=0; i<p; i++) std::cout << r_buffer.data[i];
      std::cout << std::endl;		
    }
    		
  }
}

bool serialport::serCallback(const std::shared_ptr<force_sensor_msg::srv::Serialport::Request> req, const std::shared_ptr<force_sensor_msg::srv::Serialport::Response> res)
{
  pub = false;

  if(ser.available()){
    std_msgs::msg::UInt8MultiArray serial_data;
    p = ser.available();
    ser.read (serial_data.data, p);
  }

  bool easy = false;
  if(req->command.length()<1){
    printf("Wrong request of force_sensor::Serialport\n");
    return false;
  }
  if(req->command[0] == '.') easy = true;
  
  if (strcmp(req->callname.c_str(),"GOD")==0)
  {
    set_GOD();
  }

  else if (strcmp(req->callname.c_str(),"ADJZF")==0)
  {
    if(easy){
      ask_ADJZF();
    }
    else{
      set_ADJZF(req->command[0]);
    }
  }
  
  else if (strcmp(req->callname.c_str(),"SMPR")==0)
  {
    if(easy){
      ask_SMPR();
    }
    else{
      set_SMPR(req->command.c_str());
    }
  }
  
  else if (strcmp(req->callname.c_str(),"SGDM")==0)
  {
    if(easy){
      ask_SGDM();
    }
    else{
      set_SGDM(req->command.c_str());
    }
  }

  else{
    printf("Wrong request of force_sensor::Serialport\n");
    return false;
  }

  sleep(1);
  recieve();
  
  unsigned long len_ = msg.data.size();
  for(unsigned long i=0; i<len_; i++) res->result.push_back(static_cast<char>(msg.data[i]));
  
  get_instructions();

  pub = true;
  return true;
}

void serialport::set_GOD()
{
  //send: A  T  +  G  O  D  \r \n
  //ASCII:41 54 2b 47 4f 44 0d 0a
  memset(s_buffer,0,sizeof(s_buffer));
  s_buffer[0]  = 0x41;
  s_buffer[1]  = 0x54;
  s_buffer[2]  = 0x2b;
  s_buffer[3]  = 0X47;
  s_buffer[4]  = 0X4f;
  s_buffer[5]  = 0X44;
  s_buffer[6]  = 0X0d;
  s_buffer[7]  = 0X0a;

  ser.write(s_buffer,8);
}

void serialport::set_ADJZF(char adjzf)
{
  //send: A  T  +  A  D  J  Z  F  =  1  ;  1  ;  1  ;  1  ;  1  ;  1  ;  \r \n
  //ASCII:41 54 2b 41 44 4a 5a 46 3d 31 3b 31 3b 31 3b 31 3b 31 3b 31 3b 0d 0a
  memset(s_buffer,0,sizeof(s_buffer));
  s_buffer[0]  = 0x41;
  s_buffer[1]  = 0x54;
  s_buffer[2]  = 0x2b;
  s_buffer[3]  = 0X41;
  s_buffer[4]  = 0X44;
  s_buffer[5]  = 0X4a;
  s_buffer[6]  = 0X5a;
  s_buffer[7]  = 0X46;
  s_buffer[8]  = 0x3d;

  s_buffer[9]  = static_cast<unsigned char>(adjzf);
  s_buffer[10] = 0x3b;
  s_buffer[11] = static_cast<unsigned char>(adjzf);
  s_buffer[12] = 0X3b;
  s_buffer[13] = static_cast<unsigned char>(adjzf);
  s_buffer[14] = 0X3b;

  s_buffer[15] = static_cast<unsigned char>(adjzf);
  s_buffer[16] = 0X3b;
  s_buffer[17] = static_cast<unsigned char>(adjzf);
  s_buffer[18] = 0X3b;
  s_buffer[19] = static_cast<unsigned char>(adjzf);
  s_buffer[20] = 0X3b;

  s_buffer[21] = 0X0d;
  s_buffer[22] = 0X0a;

  ser.write(s_buffer,23);

  printf("Wait ");
  for(int i=0; i<5; i++){
    printf(". ");
    sleep(1);
  }
  printf("done.\n");
}

void serialport::ask_ADJZF()
{
  //send: A  T  +  A  D  J  Z  F  =  ?  \r \n
  //ASCII:41 54 2b 41 44 4a 5a 46 3d 3f 0d 0a
  memset(s_buffer,0,sizeof(s_buffer));
  s_buffer[0]  = 0x41;
  s_buffer[1]  = 0x54;
  s_buffer[2]  = 0x2b;
  s_buffer[3]  = 0X41;
  s_buffer[4]  = 0X44;
  s_buffer[5]  = 0X4a;
  s_buffer[6]  = 0X5a;
  s_buffer[7]  = 0X46;
  s_buffer[8]  = 0x3d;
  s_buffer[9]  = 0x3f;

  s_buffer[10] = 0X0d;
  s_buffer[11] = 0X0a;

  ser.write(s_buffer,12);
}

void serialport::set_SMPR(std::string smpr)
{
  unsigned long count = smpr.length();
  //send: A  T  +  S  M  P  R  =  _  _  _  _  \r \n
  //ASCII:41 54 2b 53 4d 50 52 3d _  _  _  _  0d 0a
  memset(s_buffer,0,sizeof(s_buffer));
  s_buffer[0]  = 0x41;
  s_buffer[1]  = 0x54;
  s_buffer[2]  = 0x2b;
  s_buffer[3]  = 0X53;
  s_buffer[4]  = 0X4d;
  s_buffer[5]  = 0X50;
  s_buffer[6]  = 0X52;
  s_buffer[7]  = 0X3d;

  for(unsigned long i=0;i<count;i++) s_buffer[8+i] = static_cast<unsigned char>(smpr[i]);

  s_buffer[8+count] = 0X0d;
  s_buffer[9+count] = 0X0a;

  ser.write(s_buffer,14);
}

void serialport::ask_SMPR()
{
  //send: A  T  +  S  M  P  R  =  ?  \r \n
  //ASCII:41 54 2b 53 4d 50 52 3d 3f 0d 0a
  memset(s_buffer,0,sizeof(s_buffer));
  s_buffer[0]  = 0x41;
  s_buffer[1]  = 0x54;
  s_buffer[2]  = 0x2b;
  s_buffer[3]  = 0X53;
  s_buffer[4]  = 0X4d;
  s_buffer[5]  = 0X50;
  s_buffer[6]  = 0X52;
  s_buffer[7]  = 0X3d;
  s_buffer[8]  = 0X3f;
  s_buffer[9]  = 0X0d;
  s_buffer[10] = 0X0a;

  ser.write(s_buffer,11);
}

void serialport::set_SGDM(std::string sgdm)
{
  unsigned long count = sgdm.length();
  //send: A  T  +  S  G  D  M  =  _  _  _  _  \r \n
  //ASCII:41 54 2b 53 47 44 4d 3d _  _  _  _  0d 0a
  memset(s_buffer,0,sizeof(s_buffer));
  s_buffer[0]  = 0x41;
  s_buffer[1]  = 0x54;
  s_buffer[2]  = 0x2b;
  s_buffer[3]  = 0X53;
  s_buffer[4]  = 0X47;
  s_buffer[5]  = 0X44;
  s_buffer[6]  = 0X4d;
  s_buffer[7]  = 0X3d;
  
  for(unsigned long i=0;i<count;i++) s_buffer[8+i] = static_cast<unsigned char>(sgdm[i]);

  s_buffer[8+count] = 0X0d;
  s_buffer[9+count] = 0X0a;

  ser.write(s_buffer,10+count);
}

void serialport::ask_SGDM()
{
  //send: A  T  +  S  G  D  M  =  ?  \r \n
  //ASCII:41 54 2b 53 47 44 4d 3d 3f 0d 0a
  memset(s_buffer,0,sizeof(s_buffer));
  s_buffer[0]  = 0x41;
  s_buffer[1]  = 0x54;
  s_buffer[2]  = 0x2b;
  s_buffer[3]  = 0X53;
  s_buffer[4]  = 0X47;
  s_buffer[5]  = 0X44;
  s_buffer[6]  = 0X4d;
  s_buffer[7]  = 0X3d;
  s_buffer[8]  = 0X3f;
  s_buffer[9]  = 0X0d;
  s_buffer[10] = 0X0a;

  ser.write(s_buffer,11);
}

}


int main (int argc, char** argv) 
{ 
	//初始化节点 
	rclcpp::init(argc, argv); 

  auto port = std::make_shared<force_sen::serialport>("forcesensor","forcesensor_srv");

  port->init();

  port->pub_thread();
  
  rclcpp::spin(port);

  return 0;
} 
