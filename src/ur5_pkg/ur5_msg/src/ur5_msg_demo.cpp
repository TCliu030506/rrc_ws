//接受机械臂的关节位置和速度信息并且发布在话题"robotstate"中

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include "ur5_msg/msg/robot_state.hpp"


class sensor_pipeline : public rclcpp::Node
{
private:
  std_msgs::msg::String jointstate_topic ;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate;
  rclcpp::Publisher<ur5_msg::msg::RobotState>::SharedPtr robotstate;

  ur5_msg::msg::RobotState robotstate_msg;
  bool callflag_jointstate;
  

  float joint_pos[6], joint_vel[6];    //save joint msg


  //fill robotstate msg
  void fillRobotstateMsg()
  {
    robotstate_msg.header.frame_id  = "this->now()";

    for(int i=0; i<6; i++){
      robotstate_msg.joint_pos[i] = joint_pos[i];
      robotstate_msg.joint_vel[i] = joint_vel[i];
    }
    return;

  }

public:
  sensor_pipeline();
  ~sensor_pipeline();

  void jointstateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    //get msg in
    //RCLCPP_INFO(this->get_logger(), "Received JointState message:");

    for(int i=0; i<6; i++){
      if(strcmp(msg->name[i].c_str(),"shoulder_pan_joint")==0)
      {
        joint_pos[0] = msg->position[i];
        joint_vel[0] = msg->velocity[i];
        //RCLCPP_INFO(this->get_logger(), "shoulder_pan_joint");
      }else if (strcmp(msg->name[i].c_str(),"shoulder_lift_joint")==0)
      {
        joint_pos[1] = msg->position[i];
        joint_vel[1] = msg->velocity[i];
        //RCLCPP_INFO(this->get_logger(), "shoulder_lift_joint");
      }else if (strcmp(msg->name[i].c_str(),"elbow_joint")==0)
      {
        joint_pos[2] = msg->position[i];
        joint_vel[2] = msg->velocity[i];
        //RCLCPP_INFO(this->get_logger(), "elbow_joint");
      }else if (strcmp(msg->name[i].c_str(),"wrist_1_joint")==0)
      {
        joint_pos[3] = msg->position[i];
        joint_vel[3] = msg->velocity[i];
        //RCLCPP_INFO(this->get_logger(), "wrist_1_joint");
      }else if (strcmp(msg->name[i].c_str(),"wrist_2_joint")==0)
      {
        joint_pos[4] = msg->position[i];
        joint_vel[4] = msg->velocity[i];
        //RCLCPP_INFO(this->get_logger(), "wrist_2_joint");
      }else if (strcmp(msg->name[i].c_str(),"wrist_3_joint")==0)
      {
        joint_pos[5] = msg->position[i];
        joint_vel[5] = msg->velocity[i];
        //RCLCPP_INFO(this->get_logger(), "wrist_3_joint");
      }else
      {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Could not match joint name");
      }

    }
    
    fillRobotstateMsg();
    robotstate->publish(robotstate_msg);
  

    //test
    //std::cout << "I calculated Rs_b: " << std::endl;
    //std::cout << "Compensation: " << std::endl;
    //std::cout << (Rs_b.transpose()*gravity + force_base) << std::endl;
    
    callflag_jointstate = true;
  }

};



sensor_pipeline::sensor_pipeline(/* args */):Node("ur5_msg_demo")
{
  jointstate_topic.data = "joint_states";
  
  jointstate = create_subscription<sensor_msgs::msg::JointState>(jointstate_topic.data, 10,
    std::bind(&sensor_pipeline::jointstateCallback, this, std::placeholders::_1));

  
  robotstate = this->create_publisher<ur5_msg::msg::RobotState>("robotstate", 10);//创建一个发布者对象用于发布 ur5_msg::msg::RobotState类型的消息到名为 "robotstate" 的话题上。


  robotstate_msg.header.frame_id = "0";
  callflag_jointstate = false;
  
  RCLCPP_INFO(this->get_logger(), "Initialization finished");

  //std::cout << "Initialized: " << Rt_6 << Rs_t << Rs_p << std::endl;
}

sensor_pipeline::~sensor_pipeline(){}

    

  int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sensor_pipeline>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}

