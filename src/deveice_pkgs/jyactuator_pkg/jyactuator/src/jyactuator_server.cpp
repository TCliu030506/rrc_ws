#include "../include/jyactuator.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<jyactuator::JyActuator>(1, 25, "jyactuator_msg", "jyactuator_srv"));
	
  rclcpp::shutdown();

  return 0;
} 
