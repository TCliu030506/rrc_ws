// pf_kinematics.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "../include/pf_kinematics/pf_kinematics.h"
#define M_PI 3.14159265358979323846

int main()
{
    pf_kinematics pf_kin;

    //// 测试连续体正向运动学
    //double x_continuum_back = 10.0, y_continuum_back = 20.0, z_continuum_back = 65.0;
    //double x_continuum_front, y_continuum_front, z_continuum_front;

    //forward_continuum(x_continuum_back, y_continuum_back, z_continuum_back,
    //                   x_continuum_front, y_continuum_front, z_continuum_front);
    //
    //std::cout << "Front Continuum Position (X, Y, Z): (" 
    //          << x_continuum_front << ", " 
    //          << y_continuum_front << ", " 
    //          << z_continuum_front << ")" << std::endl;

    //// 测试对偶体正向运动学
    //double theta1 = 30.0/180* M_PI, theta2 = 30.0 / 180 * M_PI, theta3 = 45.0 / 180 * M_PI;
    //int type = 1;  // Can be 1 for position (X,Y,Z) or 2 for (L, fi, theta)
    //double output1, output2, output3;

    //MD_forward_kinematics(theta1, theta2, theta3, type, output1, output2, output3);

    //std::cout << "Output1: " << output1 << ", Output2: " << output2 << ", Output3: " << output3 << std::endl;

 
    // //测试内窥镜运动平台运动学
    //double theta1 = 0.3, theta2 = 0.4, theta3 = 0.6;
    //double x_continuum_front, y_continuum_front, z_continuum_front;
    //PF_forward_kinematics(theta1, theta2, theta3, x_continuum_front, y_continuum_front, z_continuum_front);


    // //测试连续体逆运动学
    //double x_continuum_front =  8.0;
    //double y_continuum_front = -12.0;
    //double z_continuum_front = -65.0;

    //double x_continuum_back, y_continuum_back, z_continuum_back;

    //inverse_continuum(x_continuum_front, y_continuum_front, z_continuum_front,
    //                   x_continuum_back, y_continuum_back, z_continuum_back);

    //std::cout << "输入前端位置（X, Y, Z）: (" 
    //          << x_continuum_front << ", " 
    //          << y_continuum_front << ", " 
    //          << z_continuum_front << ")" << std::endl;

    //std::cout << "输出后端位置（X, Y, Z）: (" 
    //          << x_continuum_back << ", " 
    //          << y_continuum_back << ", " 
    //          << z_continuum_back << ")" << std::endl;


    //// 测试对偶体逆运动学
    //double theta1 = 30.0/180* M_PI, theta2 = 30.0 / 180 * M_PI, theta3 = 15.0 / 180 * M_PI;
    //double x_target = -6.0, y_target = -12.0, z_target = 63.0;

    //std::vector<double> result = MD_inverse_kinematics(theta1, theta2, theta3, x_target, y_target, z_target);
    //
    //std::cout << "当前摆杆角度（theta1, theta2, theta3）:" << std::endl;
    //std::cout << "theta1: " << theta1 << ", theta2: " << theta2 << ", theta3: " << theta3 << std::endl;

    //std::cout << "对偶体末端目标位置（X, Y, Z）:" << std::endl;
    //std::cout << "theta1: " << x_target << ", theta2: " << y_target << ", theta3: " << z_target << std::endl;

    //std::cout << "目标摆杆角度（theta1, theta2, theta3）:" << std::endl;
    //std::cout << "theta1: " << result[0] << ", theta2: " << result[1] << ", theta3: " << result[2] << std::endl;
    //


    // 测试 PF_inverse_kinematics 函数
    double theta1 = 30.0 / 180.0 * M_PI;
    double theta2 = 30.0 / 180.0 * M_PI;
    double theta3 = 15.0 / 180.0 * M_PI;
    double x_continuum_front =  8.0;
    double y_continuum_front = -12.0;
    double z_continuum_front = -65.0;
    double theta_target1, theta_target2, theta_target3;

    pf_kin.PF_inverse_kinematics(theta1, theta2, theta3, x_continuum_front, y_continuum_front, z_continuum_front,theta_target1, theta_target2, theta_target3);
 
    std::cout << "输入目标位置（X,Y,Z）:" << std::endl;
    std::cout << x_continuum_front << ", " << y_continuum_front << ", " << z_continuum_front << std::endl;
    std::cout << "输出摆杆角度（theta1, theta2, theta3）:" << std::endl;
    std::cout << theta_target1 / M_PI * 180 << ", " << theta_target2 / M_PI * 180 << ", " << theta_target3 / M_PI * 180 << std::endl;
    return 0;
}

