#pragma once

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "eigen3/Eigen/Dense"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef Eigen::Matrix3d           EM3d;
typedef Eigen::Vector3d           EV3d;


class pf_kinematics
{
private:

public:
    pf_kinematics();
    ~pf_kinematics();

    // 判断给定的位置是否在安全域内
    bool is_within_SafetyDomain(double x,double y,double z);
    // 判断给定的关节位置是否在关节软限位内；输入为弧度
    bool is_within_SoftLimit(double theta1,double theta2,double theta3);

    // 后端连续体到前端连续体的运动学映射
    void forward_continuum(double x_continuum_back, double y_continuum_back, double z_continuum_back, 
                        double &x_continuum_front, double &y_continuum_front, double &z_continuum_front);

    // 对偶体正运动学计算
    void MD_forward_kinematics(double theta1, double theta2, double theta3, int type,
                                double &outputArg1, double &outputArg2, double &outputArg3);

    // 内窥镜运动平台运动学
    void PF_forward_kinematics(double theta1, double theta2, double theta3, double &x_continuum_front, double &y_continuum_front, double &z_continuum_front);


    // 连续体逆运动学：连续体前端至后端的映射
    void inverse_continuum(double x_continuum_front, double y_continuum_front, double z_continuum_front,
                            double &x_continuum_back, double &y_continuum_back, double &z_continuum_back);

    // 机械对偶体逆运动学
    std::vector<double> MD_inverse_kinematics(double theta1, double theta2, double theta3, 
                                                double x_target, double y_target, double z_target);

    // 内窥镜运动平台逆运动学
    void PF_inverse_kinematics(double theta1, double theta2, double theta3, double x_continuum_front, double y_continuum_front, double z_continuum_front,
                       double &theta_target1, double &theta_target2, double &theta_target3);

    // 计算给定位置的雅各比矩阵
    void jaccob_calculate(EV3d theta, EM3d &joccob_mat ,double angle_step);

    /**
    * @brief 内窥镜运动平台速度逆运动学
    * @param[in] theta 当前舵机关节角度。
    * @param[in] vel_end PF末端运动速度。
    * @param[in] angle_step 雅各比矩阵步长。
    * @param[out] vel_theta 舵机速度。
    */
    void PF_inverse_vel(EV3d theta, EV3d vel_end, double angle_step, EV3d &vel_theta);

};