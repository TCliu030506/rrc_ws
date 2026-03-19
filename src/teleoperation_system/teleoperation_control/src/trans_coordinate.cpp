#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "x_core_sdk/robot.h"
#include "x_core_sdk/utility.h"
#include "x_core_sdk/model.h"

// 定义构建齐次变换矩阵的函数
Eigen::Matrix4d homogeneousTransform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = rotation;
    T.block<3, 1>(0, 3) = translation;
    return T;
}

// 将输入的rx,ry,rz表示转换为Eigen::Matrix3d& rotation
Eigen::Matrix3d eulerAnglesToRotationMatrix(double rx, double ry, double rz) {
    // 将角度转换为弧度
    rx = rx * M_PI / 180.0;
    ry = ry * M_PI / 180.0;
    rz = rz * M_PI / 180.0;

    // 分别计算绕 X、Y、Z 轴的旋转矩阵
    Eigen::Matrix3d Rx = Eigen::Matrix3d::Identity();
    Rx(1,1) = cos(rx);
    Rx(1,2) = -sin(rx);
    Rx(2,1) = sin(rx);
    Rx(2,2) = cos(rx);

    Eigen::Matrix3d Ry = Eigen::Matrix3d::Identity();
    Ry(0,0) = cos(ry);
    Ry(0,2) = sin(ry);
    Ry(2,0) = -sin(ry);
    Ry(2,2) = cos(ry);

    Eigen::Matrix3d Rz = Eigen::Matrix3d::Identity();
    Rz(0,0) = cos(rz);
    Rz(0,1) = -sin(rz);
    Rz(1,0) = sin(rz);
    Rz(1,1) = cos(rz);

    // 组合旋转矩阵，顺序为 Z-Y-X
    return Rz * Ry * Rx;
}

// 将输入的X，Y，Z，rx,ry,rz 转换为齐次变换矩阵
Eigen::Matrix4d xyzRPYToHomogeneousTransform(double x, double y, double z, double rx, double ry, double rz) {
    Eigen::Matrix3d rotation = eulerAnglesToRotationMatrix(rx, ry, rz);
    Eigen::Vector3d translation(x, y, z);
    return homogeneousTransform(rotation, translation);
}

// 将齐次变换矩阵转换为 x, y, z, rx, ry, rz
std::array<double, 6> homogeneousTransformToXYZRPY(const Eigen::Matrix4d& T) {
    Eigen::Vector3d translation = T.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = T.block<3, 3>(0, 0);

    double sy = std::sqrt(rotation(0,0) * rotation(0,0) +  rotation(1,0) * rotation(1,0) );

    bool singular = sy < 1e-6; // 奇异情况

    double x, y, z, rx, ry, rz;
    x = translation(0);
    y = translation(1);
    z = translation(2);

    if (!singular) {
        rx = std::atan2(rotation(2,1), rotation(2,2));
        ry = std::atan2(-rotation(2,0), sy);
        rz = std::atan2(rotation(1,0), rotation(0,0));
    } else {
        rx = std::atan2(-rotation(1,2), rotation(1,1));
        ry = std::atan2(-rotation(2,0), sy);
        rz = 0;
    }

    // 将弧度转换为角度
    rx = rx * 180.0 / M_PI;
    ry = ry * 180.0 / M_PI;
    rz = rz * 180.0 / M_PI;

    return {x, y, z, rx, ry, rz};
}

// 输入A系下的机械臂位姿，返回A系下的内窥镜运动平台位姿
std::array<double, 6> pf_trans(std::array<double, 6> pos)
{
    // 计算传递矩阵
    double d1 = 0.12091;
    double d2 = 0.2078;
    double alpha = 45.0/180*M_PI;
    double l = sqrt(pow(d1, 2) + pow(d2, 2));
    double fi = atan2(d1, d2);

    std::array<double, 6UL> trans_pos = {l*sin(alpha-fi), 0, l*cos(alpha-fi), 0, alpha, 0};
    // 计算传递矩阵
    std::array<double, 16UL> transMatrix;
    rokae::Utils::postureToTransArray(trans_pos,transMatrix);
    Eigen::Matrix4d transMatrixEigen;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transMatrixEigen(i, j) = transMatrix[i * 4 + j];
        }
    }
    // 内窥镜运动平台位姿用旋转矩阵表示
    std::array<double, 16UL> posMatrix;
    rokae::Utils::postureToTransArray(pos,posMatrix);
    Eigen::Matrix4d posMatrixEigen;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            posMatrixEigen(i, j) = posMatrix[i * 4 + j];
        }
    }
    // 计算目标位姿
    Eigen::Matrix4d targetMatrixEigen = posMatrixEigen * transMatrixEigen;

    // 转换为RPY表示
    std::array<double, 6> targetPos = homogeneousTransformToXYZRPY(targetMatrixEigen);
    return targetPos;
}   

int main() {
    // 定义旋转角度，顺时针旋转为负角度
    double theta_y = -M_PI / 6; // 绕 Y 轴顺时针旋转 30 度
    double theta_z = -M_PI / 2; // 绕 Z 轴顺时针旋转 90 度

    // 分别计算绕 Y 轴和 Z 轴的旋转矩阵
    Eigen::Matrix3d rotation_y;
    rotation_y << std::cos(theta_y), 0, std::sin(theta_y),
                  0, 1, 0,
                  -std::sin(theta_y), 0, std::cos(theta_y);

    Eigen::Matrix3d rotation_z;
    rotation_z << std::cos(theta_z), -std::sin(theta_z), 0,
                  std::sin(theta_z), std::cos(theta_z), 0,
                  0, 0, 1;

    // 组合旋转矩阵，顺序为 Z 后 Y 先
    Eigen::Matrix3d rotation_OA =  rotation_y * rotation_z ;

    // 假设平移向量不变
    Eigen::Vector3d translation_OA(0, 0, 0);

    // 构建坐标系 A 相对于坐标系 O 的齐次变换矩阵 T_OA
    Eigen::Matrix4d T_OA = homogeneousTransform(rotation_OA, translation_OA);

    // 求 T_OA 的逆矩阵 T_AO
    Eigen::Matrix4d T_AO = T_OA.inverse();

    while (true) {
        int model_flag = 0;
        std::cout << "请输入坐标系转换方向:1（坐标系O到坐标系A），2（坐标系A到坐标系O） ,3（A系下机械臂末端到PF末端） " << std::endl;
        if (!(std::cin >> model_flag) || (model_flag != 1 && model_flag != 2 && model_flag != 3)) {
            std::cout << "输入错误，请重新输入。" << std::endl;
            continue; 
        }
        if (model_flag == 1) {
            double x, y, z, rx, ry, rz;
            std::cout << "请输入 O 坐标系下的初始位姿 P_O (x y z rx ry rz): ";
            std::cin >> x >> y >> z >> rx >> ry >> rz;
    
            // 构建 O 坐标系下的初始位姿 P_O 的齐次变换矩阵
            Eigen::Matrix4d P_O = xyzRPYToHomogeneousTransform(x, y, z, rx, ry, rz);
    
            // 计算 P_O 在 A 坐标系下的位姿 P_A
            Eigen::Matrix4d P_A = T_AO * P_O;
    
            // 将 P_A 转换为 x, y, z, rx, ry, rz 表示
            std::array<double, 6> P_A_rpy = homogeneousTransformToXYZRPY(P_A);
            std::cout << "P_A 在 A 坐标系下的位姿 (x, y, z, rx, ry, rz): ";
            for (double val : P_A_rpy) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
        else if (model_flag == 2) {
            double x, y, z, rx, ry, rz;
            std::cout << "请输入 A 坐标系下的初始位姿 P_A (x y z rx ry rz): ";
            std::cin >> x >> y >> z >> rx >> ry >> rz;
    
            // 构建 A 坐标系下的初始位姿 P_A 的齐次变换矩阵
            Eigen::Matrix4d P_A = xyzRPYToHomogeneousTransform(x, y, z, rx, ry, rz);
    
            // 计算 P_O 在 O 坐标系下的位姿 P_O
            Eigen::Matrix4d P_O = T_OA * P_A;
    
            // 将 P_O 转换为 x, y, z, rx, ry, rz 表示
            std::array<double, 6> P_O_rpy = homogeneousTransformToXYZRPY(P_O);
            std::cout << "P_O 在 O 坐标系下的位姿 (x, y, z, rx, ry, rz): ";
            for (double val : P_O_rpy) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
        else if (model_flag == 3) {
            double x, y, z, rx, ry, rz;
            std::cout << "请输入 A 坐标系下的机械臂位姿 P_A (x y z rx ry rz): ";
            std::cin >> x >> y >> z >> rx >> ry >> rz;

            std::array<double, 6> P_A = { x, y, z, rx, ry, rz };
            P_A[3] = P_A[3]/180*M_PI;
            P_A[4] = P_A[4]/180*M_PI;
            P_A[5] = P_A[5]/180*M_PI;
            std::array<double, 6> P_new_A_rpy = pf_trans(P_A);
            std::cout << " A 坐标系下的内窥镜运动平台位姿 (x, y, z, rx, ry, rz): ";
            for (double val : P_new_A_rpy) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }


        
        // double x, y, z, rx, ry, rz;
        // std::cout << "请输入 O 坐标系下的初始位姿 P_O (x y z rx ry rz): ";
        // std::cin >> x >> y >> z >> rx >> ry >> rz;

        // // 构建 O 坐标系下的初始位姿 P_O 的齐次变换矩阵
        // Eigen::Matrix4d P_O = xyzRPYToHomogeneousTransform(x, y, z, rx, ry, rz);

        // // 计算 P_O 在 A 坐标系下的位姿 P_A
        // Eigen::Matrix4d P_A = T_AO * P_O;

        // // 将 P_A 转换为 x, y, z, rx, ry, rz 表示
        // std::array<double, 6> P_A_rpy = homogeneousTransformToXYZRPY(P_A);
        // std::cout << "P_A 在 A 坐标系下的位姿 (x, y, z, rx, ry, rz): ";
        // for (double val : P_A_rpy) {
        //     std::cout << val << " ";
        // }

        // std::cout << "请输入 A 坐标系下的运动位姿 P_delta_A (x y z rx ry rz): ";
        // std::cin >> x >> y >> z >> rx >> ry >> rz;

        // // 构建 A 坐标系下的运动位姿 P_delta_A 的齐次变换矩阵
        // std::array<double, 6> P_delta_A = { x, y, z, rx, ry, rz };

        // std::array<double, 6> P_new_A_rpy;
        // for (size_t i = 0; i < P_new_A_rpy.size(); ++i) {
        //     P_new_A_rpy[i] = P_A_rpy[i] + P_delta_A[i];
        // }

        // Eigen::Matrix4d P_new_A = xyzRPYToHomogeneousTransform(P_new_A_rpy[0],P_new_A_rpy[1],P_new_A_rpy[2],P_new_A_rpy[3],P_new_A_rpy[4],P_new_A_rpy[5]);

        // // 计算 P_new_A 在 O 坐标系下的位姿 P_new_O
        // Eigen::Matrix4d P_new_O = T_OA * P_new_A;

        // // 将 P_new_O 转换为 x, y, z, rx, ry, rz 表示
        // std::array<double, 6> P_new_result = homogeneousTransformToXYZRPY(P_new_O);

        // std::cout << "P_new_O 在 O 坐标系下的位姿 (x, y, z, rx, ry, rz): ";
        // for (double val : P_new_result) {
        //     std::cout << val << " ";
        // }
        // std::cout << std::endl;
    }

    return 0;
}