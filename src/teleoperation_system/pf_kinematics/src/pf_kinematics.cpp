#include "../include/pf_kinematics/pf_kinematics.h"

pf_kinematics::pf_kinematics(){};
pf_kinematics::~pf_kinematics(){};

// 判断给定的位置是否在安全域内；输入为mm
// bool pf_kinematics::is_within_SafetyDomain(double x,double y,double z){
//     // 计算判断值
//     double distance_to_upper1_origin = (x - 4.4848) * (x - 4.4848) / (41.1661 * 41.1661) + 
//                                        (y - 13.7067) * (y - 13.7067) / (45.1008 * 45.1008) + 
//                                        (z + 4.4222) * (z + 4.4222) / (42.5190 * 42.5190);
//     double distance_to_upper2_origin = (x + 14.8624) * (x + 14.8624) / (44.0828 * 44.0828) + 
//                                        (y + 2.9637) * (y + 2.9637) / (42.6996 * 42.6996) + 
//                                        (z - 1.9831) * (z - 1.9831) / (49.0050 * 49.0050);
//     double distance_to_upper3_origin = (x - 11.3548) * (x - 11.3548) / (40.1552 * 40.1552) + 
//                                        (y + 10.9556) * (y + 10.9556) / (43.7830 * 43.7830) + 
//                                        (z + 3.6268) * (z + 3.6268) / (43.2373 * 43.2373);
//     double distance_to_lower1_origin = (x + 18.4125) * (x + 18.4125) / (74.1662 * 74.1662) + 
//                                        (y + 31.6463) * (y + 31.6463) / (73.9882 * 73.9882) + 
//                                        (z + 34.9878) * (z + 34.9878) / (66.9111 * 66.9111);
//     double distance_to_lower2_origin = (x - 30.5363) * (x - 30.5363) / (68.0689 * 68.0689) + 
//                                        (y - 0.0076) * (y - 0.0076) / (71.5532 * 71.5532) + 
//                                        (z + 33.5938) * (z + 33.5938) / (66.2210 * 66.2210);
//     double distance_to_lower3_origin = (x + 18.3951) * (x + 18.3951) / (73.9608 * 73.9608) + 
//                                        (y - 31.3534) * (y - 31.3534) / (73.5610 * 73.5610) + 
//                                        (z + 35.1407) * (z + 35.1407) / (66.5571 * 66.5571);
//     // 逐个条件判断
//     bool flag_upper1 =  distance_to_upper1_origin > 1;
//     bool flag_upper2 =  distance_to_upper2_origin > 1;
//     bool flag_upper3 =  distance_to_upper3_origin > 1;
//     bool flag_lower1 =  distance_to_lower1_origin < 1;
//     bool flag_lower2 =  distance_to_lower2_origin < 1;
//     bool flag_lower3 =  distance_to_lower3_origin < 1;
//     bool flag_x = (x > -35.9319 && x < 45.0503);
//     bool flag_y = (y > -41.1745 && y < 42.0381);
//     bool flag_z = (z > -92.0666 && z < -38.1901);
//     // 综合判断
//     bool flag_in_zone = flag_upper1 && flag_upper2 && flag_upper3 && flag_lower1 && flag_lower2 && flag_lower3 && flag_x && flag_y && flag_z;
//     return flag_in_zone ;
// }

bool pf_kinematics::is_within_SafetyDomain(double x,double y,double z){
    return true ;
}

// 判断给定的关节位置是否在关节软限位内；输入为弧度
bool pf_kinematics::is_within_SoftLimit(double theta1,double theta2,double theta3)
{
    // 求最大关节角度
    double max_theta = std::max(theta1, std::max(theta2, theta3));
    // 求最小关节角度
    double min_theta = std::min(theta1, std::min(theta2, theta3));
    // 求关节角度差
    double delta_theta = max_theta - min_theta;
    // 定义软限位阈值
    double soft_limit_max = 75.0/180.0*M_PI + delta_theta/2.0 * 1.5;
    double soft_limit_min = 55.0/180.0*M_PI - delta_theta/2.0 * 1.5;

    // 测试用：打印角度差和软限位阈值
    std::cout << "delta_theta: " << delta_theta/M_PI*180 << std::endl;
    std::cout << "soft_limit_max: " << soft_limit_max/M_PI*180 << std::endl;
    std::cout << "soft_limit_min: " << soft_limit_min/M_PI*180 << std::endl;

    // 比较关节角度与软限位阈值
    bool flag_soft_limit = (theta1 < soft_limit_max) && (theta1 > soft_limit_min) && (theta2 < soft_limit_max) && (theta2 > soft_limit_min) && (theta3 < soft_limit_max) && (theta3 > soft_limit_min);
    return flag_soft_limit;
}


// 后端连续体到前端连续体的运动学映射
void pf_kinematics::forward_continuum(double x_continuum_back, double y_continuum_back, double z_continuum_back, 
                       double &x_continuum_front, double &y_continuum_front, double &z_continuum_front) {
    // Parameters
    double length_continuum_initial_back = 45.5;  //后端连续体长度由实际角度计算得到
    double length_continuum_initial_front = 38.5; //前段连续体长度由实际测量得到
    double radius_bottom = 5.0;
    double radius_bottom_front = 3.55;

    // Back-end position vector (x, y, z)
    double vec_Position[3] = {x_continuum_back, y_continuum_back, z_continuum_back};
    double dis_Position = sqrt(vec_Position[0] * vec_Position[0] + vec_Position[1] * vec_Position[1] + vec_Position[2] * vec_Position[2]);

    // Back-end angles (fi, theta)
    double fi_continuum_back, theta_continuum_back;
    if (vec_Position[0] == 0) {
        fi_continuum_back = 0;
    } else {
        fi_continuum_back = atan2(vec_Position[1], vec_Position[0]);
    }
    theta_continuum_back = 2 * atan2(sqrt(vec_Position[0] * vec_Position[0] + vec_Position[1] * vec_Position[1]), vec_Position[2]);
    
    double length_continuum_back;
    if (theta_continuum_back == 0) {
        length_continuum_back = dis_Position;
    } else {
        length_continuum_back = sqrt(dis_Position * dis_Position / 2 / (1 - cos(theta_continuum_back))) * theta_continuum_back;
    }

    // Front-end kinematic relations
    double delta_length_center = length_continuum_back - length_continuum_initial_back;
    double length_continuum_front = length_continuum_initial_front - delta_length_center;
    double fi_continuum_front = -fi_continuum_back;
    double theta_continuum_front = theta_continuum_back * radius_bottom / radius_bottom_front;

    // Convert to front-end (X, Y, Z)
    x_continuum_front = length_continuum_front / theta_continuum_front * cos(fi_continuum_front) * (1 - cos(theta_continuum_front));
    y_continuum_front = length_continuum_front / theta_continuum_front * sin(fi_continuum_front) * (1 - cos(theta_continuum_front));
    z_continuum_front = length_continuum_front / theta_continuum_front * sin(theta_continuum_front);

    // Rotate the coordinate system (invert Y and Z)
    y_continuum_front = -y_continuum_front;
    z_continuum_front = -z_continuum_front;
}


void pf_kinematics::MD_forward_kinematics(double theta1, double theta2, double theta3, int type,
    double& outputArg1, double& outputArg2, double& outputArg3) {
    // Initial geometric parameters
    double xo = 17;
    double h = 17 * std::sqrt(3);
    double m = 13.8;  // Modify size: m = 2.6; default size: m = 13.8;
    double l1 = 12.8;
    double l2 = 9;

    //std::cout << "theta1: " << theta1 << std::endl;

    // Fixed geometric relation computation
    EV3d vec_OO1(xo, 0, 0);
    EV3d cor_M1(xo - 0.5 * (m - l1), 0, h - std::sqrt(3) / 2 * (m - l1));

    // Rotation matrices using Eigen
    EM3d RX1;
    RX1 << 1, 0, 0,
        0, std::cos(theta1), std::sin(theta1),
        0, -std::sin(theta1), std::cos(theta1);

    //std::cout << "RX1: "  << std::endl;
    //std::cout << RX1 << std::endl;

    EM3d RX2;
    RX2 << 1, 0, 0,
        0, std::cos(theta2), std::sin(theta2),
        0, -std::sin(theta2), std::cos(theta2);

    EM3d RX3;
    RX3 << 1, 0, 0,
        0, std::cos(theta3), std::sin(theta3),
        0, -std::sin(theta3), std::cos(theta3);

    EM3d RZ;
    RZ << -1.0 / 2, -std::sqrt(3) / 2, 0,
        std::sqrt(3) / 2, -1.0 / 2, 0,
        0, 0, 1;

    // Perform rotations
    EV3d vec_M1_rot = RX1 * cor_M1;
    EV3d vec_M2_rot = RX2 * cor_M1;
    vec_M2_rot = RZ * vec_M2_rot;

    EV3d vec_M3_rot = RX3 * cor_M1;
    vec_M3_rot = RZ * vec_M3_rot;
    vec_M3_rot = RZ * vec_M3_rot;

    // Calculate plane coefficients (a, b, c, d)
    EV3d v1 = vec_M2_rot - vec_M1_rot;
    EV3d v2 = vec_M3_rot - vec_M1_rot;
    EV3d normal = v1.cross(v2);  // Normal vector of the plane
    normal.normalize();
    double a = normal(0), b = normal(1), c = normal(2);
    double d = -a * vec_M1_rot(0) - b * vec_M1_rot(1) - c * vec_M1_rot(2);

    // Projection of point O1 onto the plane
    EV3d O1(0, 0, 0);
    double dot_norm_O1 = a * O1(0) + b * O1(1) + c * O1(2);
    EV3d O1_projection = O1 - (dot_norm_O1 + d) * normal.normalized();

    // Symmetry point of O1
    EV3d O1_symmetry = 2 * O1_projection - O1;

    // Distance between O1 and O1_symmetry
    double Dis_O1O1sym = O1_symmetry.norm();

    // Distance between O1 and M1
    double Dis_O1M = cor_M1.norm();

    // Compute cos(fi)
    double cosfi = (2 * Dis_O1M * Dis_O1M - Dis_O1O1sym * Dis_O1O1sym) / (2 * Dis_O1M * Dis_O1M);

    
    // Calculate translation vector length (delt_dis)
    double delt_dis = std::sqrt(2 * l2 * l2 * (1 + cosfi));

    // Final vector position
    EV3d vec_Position = O1_symmetry.normalized() * (Dis_O1O1sym + delt_dis);
    double dis_Position = vec_Position.norm();

    // Angle calculations (fi_end, theta_end)
    double fi_end = (vec_Position(0) == 0) ? 0 : std::atan2(vec_Position(1), vec_Position(0));
    double theta_end = 2 * std::atan2(std::sqrt(vec_Position(0) * vec_Position(0) + vec_Position(1) * vec_Position(1)),
        vec_Position(2));

    double radius_end = 0;
    if (theta_end != 0) {
        radius_end = std::sqrt(dis_Position * dis_Position / 2 / (1 - std::cos(theta_end))) * theta_end;
    }
    else {
        radius_end = dis_Position;
    }
    //std::cout << "a: " << a << ", " << "b: " << b << ", " << "c: " << c << ", " << "d: " << d << ", " << std::endl;
    //std::cout << "Dis_O1M: " << Dis_O1M << std::endl;
    //std::cout << "Dis_O1O1sym: " << Dis_O1O1sym << std::endl;
    //std::cout << "cosfi: " << cosfi << std::endl;
    //std::cout << "delt_dis: " << delt_dis << std::endl;
    //std::cout << "theta_end: " << theta_end << std::endl;
  

    // Output
    if (type == 1) {
        outputArg1 = vec_Position(0);
        outputArg2 = vec_Position(1);
        outputArg3 = vec_Position(2);
    }
    else if (type == 2) {
        outputArg1 = radius_end;
        outputArg2 = fi_end;
        outputArg3 = theta_end;
    }
}

void pf_kinematics::PF_forward_kinematics(double theta1, double theta2, double theta3, double& x_continuum_front, double& y_continuum_front, double& z_continuum_front) {
    // 使用 MD_forward_kinematics 函数计算后端位置（X, Y, Z）
    double x_continuum_back, y_continuum_back, z_continuum_back;
    MD_forward_kinematics(theta1, theta2, theta3, 1, x_continuum_back, y_continuum_back, z_continuum_back);

    // 使用 forward_continuum 函数计算前端位置（X, Y, Z）
    forward_continuum(x_continuum_back, y_continuum_back, z_continuum_back,
                      x_continuum_front, y_continuum_front, z_continuum_front);

    // 展示输入角度和输出位置
    // std::cout << "输入摆杆角度（theta1, theta2, theta3）:" << std::endl;
    // std::cout << "theta1: " << theta1 * 180 / M_PI << "°, theta2: " << theta2 * 180 / M_PI << "°, theta3: " << theta3 * 180 / M_PI << "°" << std::endl;

    // std::cout << "输出前端连续体位置（X, Y, Z）:" << std::endl;
    // std::cout << "X: " << x_continuum_front << ", Y: " << y_continuum_front << ", Z: " << z_continuum_front << std::endl;

}

// 连续体逆运动学：连续体前端至后端的映射
void pf_kinematics::inverse_continuum(double x_continuum_front, double y_continuum_front, double z_continuum_front,
                       double &x_continuum_back, double &y_continuum_back, double &z_continuum_back) {
    // 连续体参数设置
    double length_continuum_initial_back = 45.5;
    double length_continuum_initial_front = 38.5;
    double radius_bottom = 5.0;
    double radius_bottom_front = 3.55;

    // 前端（X,Y,Z）转（L, fi, theta）
    // 计算对称平面中心位置向量
    double vec_Position[3] = {x_continuum_front, -y_continuum_front, -z_continuum_front};
    double dis_Position = std::sqrt(vec_Position[0] * vec_Position[0] +
                                    vec_Position[1] * vec_Position[1] +
                                    vec_Position[2] * vec_Position[2]);

    // 计算姿态角 fi 和 theta
    double fi_continuum_front;
    if (vec_Position[0] == 0) {
        fi_continuum_front = 0;
    } else {
        fi_continuum_front = std::atan2(vec_Position[1], vec_Position[0]);
    }

    double theta_continuum_front = 2 * std::atan2(std::sqrt(vec_Position[0] * vec_Position[0] +
                                                           vec_Position[1] * vec_Position[1]),
                                                 vec_Position[2]);

    double length_continuum_front;
    if (theta_continuum_front == 0) {
        length_continuum_front = dis_Position;
    } else {
        length_continuum_front = std::sqrt(dis_Position * dis_Position / 2 / (1 - std::cos(theta_continuum_front))) * theta_continuum_front;
    }

    // 前端（L, fi, theta）转后端（L, fi, theta）
    double delta_length_center = length_continuum_front - length_continuum_initial_front;

    // 计算后端的姿态
    double length_continuum_back = length_continuum_initial_back - delta_length_center;
    double fi_continuum_back = -fi_continuum_front;
    double theta_continuum_back = theta_continuum_front * radius_bottom_front / radius_bottom;

    // 后端（L, fi, theta）转（X, Y, Z）
    x_continuum_back = length_continuum_back / theta_continuum_back * std::cos(fi_continuum_back) * (1 - std::cos(theta_continuum_back));
    y_continuum_back = length_continuum_back / theta_continuum_back * std::sin(fi_continuum_back) * (1 - std::cos(theta_continuum_back));
    z_continuum_back = length_continuum_back / theta_continuum_back * std::sin(theta_continuum_back);
}


// 机械对偶体逆运动学
std::vector<double> pf_kinematics::MD_inverse_kinematics(double theta1, double theta2, double theta3,
    double x_target, double y_target, double z_target) {
    // 设置参数
    double angle_step = 0.2 / 180.0 * M_PI;
    double x_tolerance = 0.15;
    double y_tolerance = 0.15;
    double z_tolerance = 0.15;

    // 当前的位置
    double pos_current_x, pos_current_y, pos_current_z;
    MD_forward_kinematics(theta1, theta2, theta3, 1, pos_current_x, pos_current_y, pos_current_z);
    EV3d pos_current(pos_current_x, pos_current_y, pos_current_z);
    EV3d theta_current(theta1, theta2, theta3);

    // 输入目标位置
    EV3d pos_target(x_target, y_target, z_target);

    // 设置当前位置为迭代的初始位置
    EV3d pos_try = pos_current;
    EV3d theta_try = theta_current;
    EV3d pos_bias = pos_target - pos_try;

    // 自适应步长措施
    if (pos_bias.norm() / 180.0 * M_PI < angle_step * 20) {
        angle_step = pos_bias.norm() / 180.0 * M_PI;
    }
    else {
        angle_step = angle_step * 20 + pos_bias.norm() * 0.05 / 180.0 * M_PI;
    }

    // 开始逆运动学求解迭代
    while (pos_bias.norm() > std::max({ x_tolerance, y_tolerance, z_tolerance })) {

        // 添加自适应步长措施
        if (pos_bias.norm() < (angle_step * 180.0 / M_PI) / 10) {
            angle_step = angle_step / 2;
        }

        if (pos_bias.norm() < x_tolerance * 5) {
            angle_step = 0.3 / 180.0 * M_PI;
        }

        if (pos_bias.norm() < x_tolerance * 3) {
            angle_step = 0.2 / 180.0 * M_PI;
        }

        if (pos_bias.norm() < x_tolerance * 1.5) {
            angle_step = 0.05 / 180.0 * M_PI;
        }

        // 求出线性增量
        EV3d delt_theta;
        PF_inverse_vel(theta_try, pos_bias, angle_step, delt_theta);

        // 调整步进增量
        double theta_factor = delt_theta.norm() / angle_step;
        if (theta_factor > 3) {
            delt_theta /= (theta_factor / 3);
        }

        // 更新试探角度
        theta_try += delt_theta;

        // 更新当前位置
        MD_forward_kinematics(theta_try[0], theta_try[1], theta_try[2], 1, pos_try[0], pos_try[1], pos_try[2]);
        pos_bias = pos_target - pos_try;
    }

    return { theta_try[0], theta_try[1], theta_try[2] };
}



// 内窥镜运动平台逆运动学
void pf_kinematics::PF_inverse_kinematics(double theta1, double theta2, double theta3, double x_continuum_front, double y_continuum_front, double z_continuum_front,
                       double &theta_target1, double &theta_target2, double &theta_target3) {
    // 先判断目标点是否在安全域内
    if(is_within_SafetyDomain(x_continuum_front,y_continuum_front,z_continuum_front))
    {
        // 连续体逆运动学，得到后端位置
        std::vector<double> result_back;
        double x_continuum_back, y_continuum_back, z_continuum_back;
        inverse_continuum(x_continuum_front, y_continuum_front, z_continuum_front, x_continuum_back, y_continuum_back, z_continuum_back);

        // 对偶体逆运动学，得到目标摆杆角度
        std::vector<double> target_angles = MD_inverse_kinematics(theta1, theta2, theta3, 
                                                                x_continuum_back, y_continuum_back, z_continuum_back);
        theta_target1 = target_angles[0];
        theta_target2 = target_angles[1];
        theta_target3 = target_angles[2];
    }else{
        std::cout << "目标点超出安全域，无法求解" << std::endl;
    }

}


// 计算给定位置的雅各比矩阵
void pf_kinematics::jaccob_calculate(EV3d theta, EM3d &joccob_mat ,double angle_step){
    
    double theta1 = theta[0];
    double theta2 = theta[1];
    double theta3 = theta[2];

    // 当前位置
    double pos_try_x, pos_try_y, pos_try_z;
    MD_forward_kinematics(theta1, theta2, theta3, 1, pos_try_x, pos_try_y, pos_try_z);
    EV3d pos_x0(pos_try_x, pos_try_y, pos_try_z);

    // 计算当前位置的雅各比矩阵（三维斜率）
    EV3d pos_x1, pos_x2, pos_x3;
    MD_forward_kinematics(theta1 + angle_step, theta2, theta3, 1, pos_x1[0], pos_x1[1], pos_x1[2]);
    MD_forward_kinematics(theta1, theta2+ angle_step, theta3, 1, pos_x2[0], pos_x2[1], pos_x2[2]);
    MD_forward_kinematics(theta1, theta2, theta3 + angle_step, 1, pos_x3[0], pos_x3[1], pos_x3[2]);

    joccob_mat(0, 0) = (pos_x1[0] - pos_x0[0]) / angle_step;
    joccob_mat(0, 1) = (pos_x2[0] - pos_x0[0]) / angle_step;
    joccob_mat(0, 2) = (pos_x3[0] - pos_x0[0]) / angle_step;

    joccob_mat(1, 0) = (pos_x1[1] - pos_x0[1]) / angle_step;
    joccob_mat(1, 1) = (pos_x2[1] - pos_x0[1]) / angle_step;
    joccob_mat(1, 2) = (pos_x3[1] - pos_x0[1]) / angle_step;

    joccob_mat(2, 0) = (pos_x1[2] - pos_x0[2]) / angle_step;
    joccob_mat(2, 1) = (pos_x2[2] - pos_x0[2]) / angle_step;
    joccob_mat(2, 2) = (pos_x3[2] - pos_x0[2]) / angle_step;
}

/**
* @brief 内窥镜运动平台速度逆运动学
* @param[in] theta 当前舵机关节角度。
* @param[in] vel_end PF末端运动速度。
* @param[in] angle_step 雅各比矩阵步长。
* @param[out] vel_theta 舵机速度。
*/
void pf_kinematics::PF_inverse_vel(EV3d theta, EV3d vel_end, double angle_step,EV3d &vel_theta){
    
    EM3d joccob_mat;
    jaccob_calculate(theta, joccob_mat,angle_step);
    vel_theta = joccob_mat.inverse() * vel_end;
}
