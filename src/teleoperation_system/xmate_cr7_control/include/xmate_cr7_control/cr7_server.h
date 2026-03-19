#define XMATEMODEL_LIB_SUPPORTED
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include <thread>
#include <atomic>
#include "x_core_sdk/robot.h"
#include "x_core_sdk/utility.h"
#include "x_core_sdk/model.h"
#include "xmate_cr7_msg/action/cr7_script.hpp"
#include "xmate_cr7_msg/srv/cr7_script.hpp"

using namespace std;
using namespace rokae;
using namespace rokae::RtSupportedFields;


class Cr7Server : public rclcpp::Node
{
public:

    Cr7Server();
    ~Cr7Server();


private:

    rclcpp::Service<xmate_cr7_msg::srv::Cr7Script>::SharedPtr cr7_service;
    xMateRobot robot;
    FollowPosition<6> follow_pose;
    shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> rtCon;
    thread followThread;
    thread rtConThread;
    std::mutex mtx; // 互斥锁
    bool running = true; // 用于控制线程的运行状态
    error_code ec;
    string id;
    CoordinateType ct = CoordinateType::flangeInBase;
    array<double, 6> target_joint; // 目标关节位置
    double default_speed = 400;
    double speed_ratio;


    void init();

    bool execute_cmd_callback(const std::shared_ptr<xmate_cr7_msg::srv::Cr7Script::Request> req,
                                const std::shared_ptr<xmate_cr7_msg::srv::Cr7Script::Response> res);

    void movej(std::vector<double> joint_angles, int speed, int zone);
    void movep(std::array<double, 6> target_pos, int speed, int zone);

    void rtControl_loop();
    void open_rtControl_loop();
    void rtControl_pos_update(std::array<double, 6> joint_angles);

    void followPosition(xMateRobot &robot, std::array<double, 6> &target_jnt, double speed_ratio); 
    void follow_pos_start();
    void follow_pos_stop();
    void follow_pos_update(std::array<double, 6> joint_angles, int speed);
    void set_follow_speed(int speed);

    std::array<double, 6>  readp();
    std::array<double, 6>  readj();

    std::array<double, 6>  get_vel();
    std::array<double, 6>  get_joint_vel();

    std::array<double, 6>  get_joint_torque();

    double  get_acc();
    double  get_jerk();

    void inv_kinematics(std::array<double, 6> &Pos, std::array<double, 6> &jntPos);

    void forward_kinematics(std::array<double, 6> &jntPos, std::array<double, 6> &Pos);

};
