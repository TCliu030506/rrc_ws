#include <iostream>
#include <cmath>

// 定义四元数结构体
struct Quaternion {
    double w, x, y, z;
};

// 定义 RPY 结构体
struct RPY {
    double roll, pitch, yaw;
};

// 欧拉角转四元数的函数
Quaternion eulerToQuaternion(const RPY& rpy) {
    // 计算半角
    double cr = std::cos(rpy.roll * 0.5);
    double sr = std::sin(rpy.roll * 0.5);
    double cp = std::cos(rpy.pitch * 0.5);
    double sp = std::sin(rpy.pitch * 0.5);
    double cy = std::cos(rpy.yaw * 0.5);
    double sy = std::sin(rpy.yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// 四元数转 RPY 的函数
RPY quaternionToRPY(const Quaternion& q) {
    RPY rpy;

    // 计算 roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    rpy.roll = std::atan2(sinr_cosp, cosr_cosp);

    // 计算 pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        rpy.pitch = std::copysign(M_PI / 2, sinp); // 使用符号函数处理奇点
    else
        rpy.pitch = std::asin(sinp);

    // 计算 yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    rpy.yaw = std::atan2(siny_cosp, cosy_cosp);

    return rpy;
}

int main() {
    // 示例四元数
    Quaternion q1 = {0.70, -0.11, 0.7, 0.11};

    // 转换为 RPY
    RPY rpy1 = quaternionToRPY(q1);

    // 输出结果（角度）
    std::cout << "Roll: " << rpy1.roll * (180 / M_PI) << " degrees" << std::endl;
    std::cout << "Pitch: " << rpy1.pitch * (180 / M_PI) << " degrees" << std::endl;
    std::cout << "Yaw: " << rpy1.yaw * (180 / M_PI) << " degrees" << std::endl;

    // 示例欧拉角
    RPY rpy2 = {-50/(180 * M_PI), -34/(180 * M_PI), 63/(180 * M_PI)}; // 单位为弧度

    // 转换为四元数
    Quaternion q2 = eulerToQuaternion(rpy2);

    // 输出结果
    std::cout << "Quaternion: w = " << q2.w << ", x = " << q2.x << ", y = " << q2.y << ", z = " << q2.z << std::endl;

    return 0;
}
