#include "../include/cybergear.h"

int main (int argc, char* argv[]) 
{ 
	rclcpp::init(argc, argv); 
  rclcpp::spin(std::make_shared<cybergear::cybergear>());
	rclcpp::shutdown();
  return 0;
} 