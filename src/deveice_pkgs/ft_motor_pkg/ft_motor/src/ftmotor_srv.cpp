#include "../include/ft_motor/ftmotor.h"

int main (int argc, char** argv) 
{ 
	//初始化节点 
	rclcpp::init(argc, argv);

	//声明节点句柄 

  auto node = std::make_shared<ft_motor::ftmotor>();

  rclcpp::spin(node);

  return 0;
} 
