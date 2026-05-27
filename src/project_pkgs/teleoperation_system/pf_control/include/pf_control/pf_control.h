#pragma once

#include <iostream>
#include <cmath>
#include <stdexcept>
#include "pf_kinematics/pf_kinematics.h"
#include "ft_servo_motor_pkg/SCServo.h"
// #include "pf_kinematics/include/pf_kinematics/pf_kinematics.h"
// #include "ft_servo_motor_pkg/include/ft_servo_motor_pkg/SCServo.h"
// #include <SCServo.h>
// #include <pf_kinematics.h>


#define M_PI 3.14159265358979323846
#define FT_STEP 0.087890625       // 舵机每步的角度（角度）


typedef Eigen::Matrix3d           EM3d;
typedef Eigen::Vector3d           EV3d;

class pf_control
{
private:
    SMS_STS sm_st;
    pf_kinematics pf_kin;
    std::string serial_port;
    EV3d initial_pos = {0.0, 0.0, 0.0}; // 实际装配后的初始位置

    void pf_serial_open();
public:
    pf_control(const std::string& port);
    ~pf_control();

    void pf_movep(double x, double y, double z, double t);
    bool pf_movep_withflag(double x, double y, double z, double t);
    void pf_movep_corrected(double x, double y, double z, double t);
    void pf_movep(EV3d pos, EV3d vel);
    void pf_movej(double theta1, double theta2, double theta3, double t);
    void pf_movel(double start_x,double start_y,double start_z,double end_x,double end_y,double end_z,double step, double t);
    void pf_readj(double &theta1, double &theta2, double &theta3);
    void pf_readp(double &end_x,double &end_y,double &end_z);
    void pf_speedj(EV3d vel_theta);
    void pf_speedp(EV3d vel_end);
    void pf_speed_off();
};