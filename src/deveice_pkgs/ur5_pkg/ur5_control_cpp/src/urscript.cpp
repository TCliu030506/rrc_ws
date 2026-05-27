#include "../include/ur5_control_cpp/urscript.h"

namespace ur_script {

// class script_cmd
script_cmd::script_cmd(std::string cmd_in,bool pos_in,bool tim_in): 
cmd(cmd_in),is_pose(pos_in),is_tim(tim_in){}

script_cmd::~script_cmd(){}

void script_cmd::initialization(double acc_in, double vel_in, double tim_in, double rad_in)
{
  acceleration = acc_in;
  velocity = vel_in;
  time = tim_in;
  radius = rad_in;
  return;
}

void script_cmd::build_cmd(double (&pose)[6])
{
  std::stringstream ss;

  ss << cmd << "(";
  if(is_pose) ss << "p";
  ss << "[";
  for(int i=0;i<6;i++){
    ss << pose[i];
    if(i<5) ss << ",";
  }
  ss << "]";
  ss << ",a=" << std::setprecision(3) << acceleration;
  ss << ",v=" << std::setprecision(3) << velocity;
  if(is_tim) ss << ",t=" << std::setprecision(3) << time;
  ss << ",r=" << std::setprecision(3) << radius;
  ss << ")\n";

  ss >> msg.data;

  return;
}

// class script_movej
script_movej::script_movej(bool pos_):
script_cmd("movej",pos_,true){}

script_movej::~script_movej(){}

// class script_movel
script_movel::script_movel(bool pos_):
script_cmd("movel",pos_,true){}

script_movel::~script_movel(){}

// class script_movep
script_movep::script_movep(bool pos_):
script_cmd("movep",pos_,false){}

script_movep::~script_movep(){}

// class script_speedj
script_speed::script_speed(){}
script_speed::script_speed(double acc_in,double tim_in)
{
  acceleration = acc_in;
  time = tim_in;
}

script_speed::~script_speed(){}

void script_speed::build_cmd(std::string cmd,double (&vel)[6])
{
  std::stringstream ss;

  if(strcmp(cmd.c_str(),"speedj")==0) ss << "speedj([";
  else if(strcmp(cmd.c_str(),"speedl")==0) ss << "speedl([";
  else {
    //ROS_ERROR_STREAM("Wrong command to UR robot:"<<cmd);
    std::cout << "Wrong command to UR robot: "<< cmd << std::endl;
    return;
  }
  
  for(int i=0;i<6;i++){
    ss << vel[i];
    if(i<5) ss << ",";
  }
  ss << "]";
  ss << "," << std::setprecision(3) << acceleration;
  ss << "," << std::setprecision(3) << time;
  ss << ")\n";

  ss >> msg.data;

  return;
}

// class script_speedl
script_stop::script_stop(){}

script_stop::~script_stop(){}

void script_stop::build_cmd(std::string cmd,double acc)
{
  std::stringstream ss;
  
  if(strcmp(cmd.c_str(),"stopl")==0) ss << "stopl(";
  else {
    ss << "stopj(";
    if(strcmp(cmd.c_str(),"stopj")!=0)  std::cout << "Wrong command to UR robot: "<< cmd << std::endl;
     //ROS_ERROR_STREAM("Wrong command to UR robot:"<<cmd);
  }

  ss << std::setprecision(3) << acc << ")\n";

  ss >> msg.data;

  return;
}

// class script_pub
script_pub::script_pub(): Node("script_pub")
{
  topic_pub = create_publisher<std_msgs::msg::String>(URSCRIPT_TOPIC, 3);
  //zjurus = new moveit::planning_interface::MoveGroupInterface("arm");
  initialize_moveit = false;
  
  usleep(100000);
};

script_pub::~script_pub(){}

bool script_pub::urscript_publish(std_msgs::msg::String &urscript_msg)
{
  topic_pub->publish(urscript_msg);
  return true;
};

bool script_pub::urscript_publish_stopj(double acc)
{
  script_stop cmd;
  cmd.build_cmd("stopj",acc);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_stopl(double acc)
{
  script_stop cmd;
  cmd.build_cmd("stopl",acc);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_speedj(double (&vel)[6], double acc, double tim)
{
  script_speed cmd;
  cmd.acceleration = acc;
  cmd.time = tim;
  cmd.build_cmd("speedj",vel);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_speedl(double (&vel)[6], double acc, double tim)
{
  script_speed cmd;
  cmd.acceleration = acc;
  cmd.time = tim;
  cmd.build_cmd("speedl",vel);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_movej_pose(double (&pose)[6], double acc, double vel, double tim, double rad)
{
  script_movej cmd(true);
  cmd.acceleration = (acc);
  cmd.velocity = (vel);
  cmd.time = (tim);
  cmd.radius = (rad);

  cmd.build_cmd(pose);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_movej_joint(double (&joint)[6], double acc, double vel, double tim, double rad)
{
  script_movej cmd(false);
  cmd.acceleration = (acc);
  cmd.velocity = (vel);
  cmd.time = (tim);
  cmd.radius = (rad);

  cmd.build_cmd(joint);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_movel_pose(double (&pose)[6], double acc, double vel, double tim, double rad)
{
  script_movel cmd(true);
  cmd.acceleration = (acc);
  cmd.velocity = (vel);
  cmd.time = (tim);
  cmd.radius = (rad);

  cmd.build_cmd(pose);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_movel_joint(double (&joint)[6], double acc, double vel, double tim, double rad)
{
  script_movel cmd(false);
  cmd.acceleration = (acc);
  cmd.velocity = (vel);
  cmd.time = (tim);
  cmd.radius = (rad);

  cmd.build_cmd(joint);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_movep_pose(double (&pose)[6], double acc, double vel, double rad)
{
  script_movep cmd(true);
  cmd.acceleration = (acc);
  cmd.velocity = (vel);
  cmd.radius = (rad);

  cmd.build_cmd(pose);
  topic_pub->publish(cmd.msg);
  return true;
};

bool script_pub::urscript_publish_movep_joint(double (&joint)[6], double acc, double vel, double rad)
{
  script_movep cmd(false);
  cmd.acceleration = (acc);
  cmd.velocity = (vel);
  cmd.radius = (rad);

  cmd.build_cmd(joint);
  topic_pub->publish(cmd.msg);
  return true;
};

/*
void script_pub::load_moveit()
{
  zjurus = new moveit::planning_interface::MoveGroupInterface("arm");
  std::string reference_frame = "base_link";
  zjurus->setPoseReferenceFrame(reference_frame);
  zjurus->allowReplanning(true);
  // normal state
  zjurus->setGoalPositionTolerance(0.001);
  zjurus->setGoalOrientationTolerance(0.002);
  zjurus->setMaxAccelerationScalingFactor(1.0);
  zjurus->setMaxVelocityScalingFactor(1.0);
  initialize_moveit = true;
}
bool script_pub::moveit_joint_go(double (&joint)[6])
{
  if(!initialize_moveit) load_moveit();
  std::vector<double> joint_moveit;
  for (unsigned long i=0; i<6; i++) joint_moveit.push_back(joint[i]);
  printf("Moveit go - Joint:[%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f]\n",
    joint_moveit[0],joint_moveit[1],joint_moveit[2],joint_moveit[3],joint_moveit[4],joint_moveit[5]);

  zjurus->setJointValueTarget(joint_moveit);
  zjurus->move();
  //std::cout << "Pub moveit motion with joint target" << std::endl;

  return true;
};

bool script_pub::moveit_pose_go(double (&pose_angleaxis)[6])
{
  if(!initialize_moveit) load_moveit();
  geometry_msgs::Pose pose_moveit;

  //angleaxis to quaternion
  pose_moveit.position.x = pose_angleaxis[0];
  pose_moveit.position.y = pose_angleaxis[1];
  pose_moveit.position.z = pose_angleaxis[2];
  
  double theta = sqrt(pow(pose_angleaxis[3],2)+pow(pose_angleaxis[4],2)+pow(pose_angleaxis[5],2));
  if(theta<0.0001){
    pose_moveit.orientation.w = cos(theta/2.0);
    pose_moveit.orientation.x = sin(theta/2.0);
    pose_moveit.orientation.y = 0;
    pose_moveit.orientation.z = 0;
  }
  else{
    pose_moveit.orientation.w = cos(theta/2.0);
    pose_moveit.orientation.x = sin(theta/2.0)*pose_angleaxis[3]/theta;
    pose_moveit.orientation.y = sin(theta/2.0)*pose_angleaxis[4]/theta;
    pose_moveit.orientation.z = sin(theta/2.0)*pose_angleaxis[5]/theta;
  }
  printf("Moveit go - Pose:[%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f]\n",
    pose_moveit.position.x,pose_moveit.position.y,pose_moveit.position.z,
    pose_moveit.orientation.w,
    pose_moveit.orientation.x,
    pose_moveit.orientation.y,
    pose_moveit.orientation.z);

  zjurus->setPoseTarget(pose_moveit);
  zjurus->move();
  //std::cout << "Pub moveit motion with pose target" << std::endl;

  return true;
};

*/




}
