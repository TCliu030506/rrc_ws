#include "../include/mtactuator.hpp"

int main (int argc, char* argv[]) 
{ 
	rclcpp::init(argc, argv); 
  
  rclcpp::spin(std::make_shared<mtactuator::MtActuator>(100,"mtactuator_msg","mtactuator_srv"));
	
  rclcpp::shutdown();
  
  return 0; 
} 