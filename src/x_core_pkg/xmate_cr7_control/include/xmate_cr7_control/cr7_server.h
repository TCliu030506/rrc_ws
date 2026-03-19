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
    array<double, 6> target_pose; // 目标笛卡尔位置
    double default_speed = 400.0;
    double speed_ratio = 1.0;


    void init();

    bool execute_cmd_callback(const std::shared_ptr<xmate_cr7_msg::srv::Cr7Script::Request> req,
                                const std::shared_ptr<xmate_cr7_msg::srv::Cr7Script::Response> res);

    void stop();

    void movej(std::vector<double> joint_angles, int speed, int zone);
    void movep(std::array<double, 6> target_pos, int speed, int zone);
    void movel(std::array<double, 6> target_pos, int speed, int zone);

    void rtControl_loop(string rt_type);
    void open_rtControl_loop(string rt_type);
    void rtControl_jointpos_update(std::array<double, 6> joint_angles);
    void rtControl_cartesianpos_update(std::array<double, 6> cartesian_pos);

    void followPosition(xMateRobot &robot, std::array<double, 6> &target_jnt, double speed_ratio); 
    void follow_pos_start();
    void follow_pos_stop();
    void follow_pos_update(std::array<double, 6> joint_angles, int speed);
    void set_follow_speed(int speed);

    std::array<double, 6>  readp();
    std::array<double, 6>  readj();

    std::array<double, 6>  get_vel();
    std::array<double, 6>  get_joint_vel();

    double  get_acc();
    double  get_jerk();

    bool calibrateForceSensor(bool all_axes, int axis_index);

    std::array<double, 6>  get_joint_torque();
    void getEndTorque(string ref_type_str, std::array<double, 6> &joint_torque_measured, std::array<double, 6> &external_torque_measured,
        std::array<double, 3> &cart_torque, std::array<double, 3> &cart_force);

    void inv_kinematics(std::array<double, 6> &Pos, std::array<double, 6> &jntPos);

    void forward_kinematics(std::array<double, 6> &jntPos, std::array<double, 6> &Pos);

};

// 备注说明：
// 1. 所有运动命令（movej, movep, movel）在发送后立即返回，不等待运动完成。
// 2. 运动停止（stop）命令会立即停止机器人运动，不等待当前运动完成。
// 3. 运动速度（speed）参数为机器人末端最大线速度, 单位mm/s。
// 4. 运动区域（zone）参数为转弯区半径大小，单位mm。
