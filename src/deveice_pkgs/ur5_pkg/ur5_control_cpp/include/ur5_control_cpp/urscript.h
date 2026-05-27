// this is for sending motion msg to moveit or realrobot(UR script)

//----------------------------------------------------------------------
/* \file
 *
 * \author  Cloudriver
 * \date    2022-03-01
 *
 */
//----------------------------------------------------------------------

#ifndef URSCRIPT_H
#define URSCRIPT_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <sstream>

//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/robot_trajectory/robot_trajectory.h>
//#include <moveit_msgs/msg/robot_trajectory.hpp>
//#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace ur_script {

#define URSCRIPT_TOPIC "/urscript_interface/script_command"

//command type
enum urscript_command {movej, movel, movep, speedj, speedl, stopl, stopj};

//Creat several class to write URscript cmd in a string array

//URscript cmd  initialize: (cmd_name,pose/joint,tim:y/n)
class script_cmd
{
private:
  std::string cmd;
  const bool  is_pose;    //if it's pose, adding 'p' to use inverse kinematics
  const bool  is_tim;

public:
  double acceleration;
  double velocity;
  double time;
  double radius;

  std_msgs::msg::String msg;

  //script_cmd (cmd_name,pose/joint,tim:y/n)
  script_cmd(std::string cmd_in,bool pos_in,bool tim_in);
  ~script_cmd();

  //initialize the paras
  void initialization(double acc_in, double vel_in, double tim_in, double rad_in);

  //build cmd in msg
  void build_cmd(double (&pose)[6]);

};

//URscript movej cmd [movej(q/p,a,v,t,r)] initialize: bool (true: use pose data / false: use joint data)
class script_movej:public script_cmd
{
private:

public:
  //script_movej(pose/joint)
  script_movej(bool pos_);
  ~script_movej();

};

//URscript movel cmd [movel(q/p,a,v,t,r)] initialize: bool (true: use pose data / false: use joint data)
class script_movel:public script_cmd
{
private:

public:
  //script_movel(pose/joint)
  script_movel(bool pos_);
  ~script_movel();

};

//URscript movep cmd [movep(q/p,a,v,r)] initialize: bool (true: use pose data / false: use joint data)
class script_movep:public script_cmd
{
private:

public:
  //script_movep(pose/joint)
  script_movep(bool pos_);
  ~script_movep();

};

//URscript speed cmd [speedj/speedl(v6,a,t)]
class script_speed
{
private:

public:
  double acceleration;
  double time;
  std_msgs::msg::String msg;

  script_speed();
  script_speed(double acc_in,double tim_in);
  ~script_speed();

  //cmd: "speedl"/"speedj"
  void build_cmd(std::string cmd,double (&vel)[6]);

};

//URscript stop cmd [stopj/stopl(a)]
class script_stop
{
private:

public:
  std_msgs::msg::String msg;

  script_stop();
  ~script_stop();

  //cmd: "stopl"/"stopj"
  void build_cmd(std::string cmd,double acc);

};

class script_pub: public rclcpp::Node
{
private: 

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topic_pub;
  //moveit::planning_interface::MoveGroupInterface *zjurus;
  bool initialize_moveit;

  //void load_moveit();

public:
  script_pub(/* args */);
  ~script_pub();

  //pub command using moveit
  //bool moveit_joint_go(double (&joint)[6]);    

  //pub command using moveit
  //bool moveit_pose_go(double (&pose_angleaxis)[6]); 

  //pub command in move.data    
  bool urscript_publish(std_msgs::msg::String &urscript_msg);    

  //pub stop cmd (cmd: "stopj(a)")
  bool urscript_publish_stopj(double acc);

  //pub stop cmd (cmd: "stopl(a)")
  bool urscript_publish_stopl(double acc);

  //pub speedj cmd (cmd: "speedj(v6,a,t)")
  bool urscript_publish_speedj(double (&vel)[6], double acc, double tim);

  //pub speedl cmd (cmd: "speedl(v6,a,t)")
  bool urscript_publish_speedl(double (&vel)[6], double acc, double tim);

  //pub movej cmd (cmd: "movej(q/p,a,v,t,r)")
  bool urscript_publish_movej_pose(double (&pose)[6], double acc, double vel, double tim, double rad);

  //pub movej cmd (cmd: "movej(q/p,a,v,t,r)")
  bool urscript_publish_movej_joint(double (&joint)[6], double acc, double vel, double tim, double rad);

  //pub movel cmd (cmd: "movel(q/p,a,v,t,r)")
  bool urscript_publish_movel_pose(double (&pose)[6], double acc, double vel, double tim, double rad);

  //pub movel cmd (cmd: "movel(q/p,a,v,t,r)")
  bool urscript_publish_movel_joint(double (&joint)[6], double acc, double vel, double tim, double rad);

  //pub movep cmd (cmd: "movep(q/p,a,v,t,r)")
  bool urscript_publish_movep_pose(double (&pose)[6], double acc, double vel, double rad);

  //pub movep cmd (cmd: "movep(q/p,a,v,t,r)")
  bool urscript_publish_movep_joint(double (&joint)[6], double acc, double vel, double rad);

};

}

#endif // URSCRIPT_HANDLE_H