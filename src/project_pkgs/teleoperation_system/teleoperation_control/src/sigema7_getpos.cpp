#include "rclcpp/rclcpp.hpp"
#include "sigema7_msg/msg/sigema7_position.hpp"
#include "xmate_cr7_msg/msg/cr7_state.hpp"
#include "ui_control_msg/msg/ui_control.hpp"

#include <sstream>
#include <cstdio>
#include <cstdlib>
#define _USE_MATH_DEFINES
#include <math.h>

#include "sigema7_pkg/dhdc.h"
#include "sigema7_pkg/drdc.h"
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <array>
#include <thread>

#define REFRESH_INTERVAL 0.1 // sec

using namespace std;

// 主端设备初始化
void MasterDeviceInit()
{
    // 1 定义工作空间中心位置，开启专家模式
    double nullPose[DHD_MAX_DOF] = {0.0, 0.0, 0.0, // base  (translations)
                                    0.0, 0.0, 0.0, // wrist (rotations)
                                    0.0};          // gripper
    dhdEnableExpertMode();
    // 2 启动第一个手控器设备
    if (drdOpen() < 0)
    {
        printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return;
    }
    // 3 打印设备编码
    if (!drdIsSupported())
    {
        printf("unsupported device\n");
        printf("exiting...\n");
        dhdSleep(2.0);
        drdClose();
        return;
    }
    printf("%s haptic device detected\n", dhdGetSystemName());
    // 4 执行自动初始化
    if (!drdIsInitialized() && drdAutoInit() < 0)
    {
        printf("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return;
    }
    else if (drdStart() < 0)
    {
        printf("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return;
    }
    // 5 启动力控和激活手腕控制
    dhdEnableForce(DHD_ON);
    if (!dhdHasActiveWrist())
    {
        dhdClose();
        printf("error: device does not have an active wrist\n");
        dhdSleep(2.0);
        return;
    }
    // 6 移动到工作空间中心位置
    drdStart();
    drdMoveTo(nullPose);
    // 7 停止初始化，开启操作
    drdStop(true);
}

// 从设备中读取位姿信息，保存到函数输入数组中
// 输入6维数组 依次为X，Y，Y，OA，OB，OG（XYZ：m OA/OB/OG：rad）
void read_pos(double *sigema7_pos)
{
    dhdGetPositionAndOrientationRad(&sigema7_pos[0], &sigema7_pos[1], &sigema7_pos[2], &sigema7_pos[3], &sigema7_pos[4], &sigema7_pos[5]);
}

// 从设备中读入夹爪姿态，保存到函数输入变量中
void read_gripper(double *sigema7_gripper)
{
    dhdGetGripperAngleDeg(sigema7_gripper);
}

// 从设备中读取笛卡尔坐标系线性速度，保存到函数输入数组中
// 输入3维数组 依次为VX，VY，VY（m/s）
void read_vel_linear(double *sigema7_vel)
{
    dhdGetLinearVelocity(&sigema7_vel[0], &sigema7_vel[1], &sigema7_vel[2]);
}

// 从设备中读取受力信息，保存到函数输入数组中
// 输入7维数组 依次为FX，FY，FY，TX，TY，TZ，f（F：N T：Nm）
void read_force(double *sigema7_force)
{
    dhdGetForceAndTorqueAndGripperForce(&sigema7_force[0], &sigema7_force[1], &sigema7_force[2], &sigema7_force[3], &sigema7_force[4], &sigema7_force[5], &sigema7_force[6]);
}

// 移动平均滤波器类
class MovingAverageFilter
{
public:
    MovingAverageFilter(size_t windowSize) : windowSize(windowSize), sum(0.0) {}

    void setWindowSize(size_t newWindowSize)
    {
        windowSize = newWindowSize;
        while (buffer.size() > windowSize)
        {
            sum -= buffer.front();
            buffer.erase(buffer.begin());
        }
    }

    double filter(double input)
    {
        if (buffer.size() >= windowSize)
        {
            sum -= buffer.front();
            buffer.erase(buffer.begin());
        }
        buffer.push_back(input);
        sum += input;
        return sum / buffer.size();
    }

private:
    size_t windowSize;
    std::vector<double> buffer;
    double sum;
};

// 卡尔曼滤波器类
class KalmanFilter
{
public:
    KalmanFilter(double q, double r, double initial_estimate)
        : q(q), r(r), estimate(initial_estimate), error_estimate(1.0) {}

    double filter(double measurement)
    {
        // 预测步骤
        double predicted_estimate = estimate;
        double predicted_error_estimate = error_estimate + q;

        // 更新步骤
        double kalman_gain = predicted_error_estimate / (predicted_error_estimate + r);
        estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate);
        error_estimate = (1 - kalman_gain) * predicted_error_estimate;

        return estimate;
    }

    void setQ(double newQ)
    {
        q = newQ;
    }

    void setR(double newR)
    {
        r = newR;
    }

    void setInitialEstimate(double newInitialEstimate)
    {
        estimate = newInitialEstimate;
    }

private:
    double q;        // 过程噪声协方差
    double r;        // 测量噪声协方差
    double estimate; // 估计值
    double error_estimate;
};

class Sigema7PosPublisher : public rclcpp::Node
{
public:
    Sigema7PosPublisher() : Node("sigema7_position_publisher")
    {
        // 初始化订阅者，订阅名为 "ui_control_topic" 的话题
        ui_control_subscriber = this->create_subscription<ui_control_msg::msg::UiControl>(
            "ui_control_topic", 10, std::bind(&Sigema7PosPublisher::ui_control_callback, this, std::placeholders::_1));
    }

    ~Sigema7PosPublisher()
    {
        if (sigema7_thread_.joinable())
        {
            sigema7_thread_.join();
        }
    }

private:
    rclcpp::Publisher<sigema7_msg::msg::Sigema7Position>::SharedPtr publisher_;
    rclcpp::Subscription<xmate_cr7_msg::msg::Cr7State>::SharedPtr cr7_state_subscriber;
    rclcpp::Subscription<ui_control_msg::msg::UiControl>::SharedPtr ui_control_subscriber;
    std::thread sigema7_thread_;
    int sleeptime_ms = 5;

    std::array<double, 7> sigema7_force_default = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 7> sigema7_force_target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::array<double, 6> cr7_pos;
    std::array<double, 6> cr7_force;

    double force_ratio = 0.05;
    double torque_ratio = force_ratio / 50.0;
    double force_threshold = 0.4;   // 力的阈值，单位为N
    double torque_threshold = 0.05; // 力据对的阈值，单位为Nm

    bool open_flag = false;
    bool close_flag = false;

    // 发布者的卡尔曼滤波器
    KalmanFilter pos_filters[6] =
        {
            KalmanFilter(1.0, 0.5, 0.0), KalmanFilter(1.0, 0.5, 0.0), KalmanFilter(1.0, 0.5, 0.0),
            KalmanFilter(1.0, 0.5, 0.0), KalmanFilter(1.0, 0.5, 0.0), KalmanFilter(1.0, 0.5, 0.0)};
    KalmanFilter vel_filters[3] =
        {
            KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0)};
    KalmanFilter force_filters[7] =
        {
            KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0),
            KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0),
            KalmanFilter(0.1, 0.5, 0.0)};

    // 订阅者力的卡尔曼滤波器
    KalmanFilter force_filters_kalman[6] = {
        KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0),
        KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0), KalmanFilter(0.1, 0.5, 0.0)};

    void Sigema7InputListener()
    {
        // 定义初始量
        double handler_pose_[6];
        double handler_vel_linear[3];
        double handler_force[7];
        double handler_gripper;

        // 滤波后的量
        double filtered_pose[6];
        double filtered_vel[3];
        double filtered_force[7];

        // 循环读取并打印数据后发布
        while (rclcpp::ok())
        {
            // 设定初始力
            dhdSetForceAndWristJointTorquesAndGripperForce(0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

            // if(close_flag) continue;

            read_pos(handler_pose_);
            read_vel_linear(handler_vel_linear);
            read_force(handler_force);
            read_gripper(&handler_gripper);

            // std::cout << "手控器当前位姿：" << std::endl;
            // RCLCPP_INFO(get_logger(),"手控器当前位姿：%lf",handler_pose_[0]);
            // std::cout << "x, y, z = " << handler_pose_[0] << ' ' << handler_pose_[1] << ' ' << handler_pose_[2] << std::endl;
            // std::cout << "rx, ry, rz = " << handler_pose_[3] << ' ' << handler_pose_[4] << ' ' << handler_pose_[5]<< std::endl;

            // // std::cout << "rx, ry, rz = " << handler_pose_[3]*180/M_PI << ' ' << handler_pose_[4]*180/M_PI << ' ' << handler_pose_[5]*180/M_PI << std::endl;
            // std::cout << "手控器当前线性速度：" << std::endl;
            // std::cout << "Vx, Vy, Vz = " << handler_vel_linear[0] << ' ' << handler_vel_linear[1] << ' ' << handler_vel_linear[2] << std::endl;

            // 对采集到的位姿、速度和力信息进行滤波
            for (int i = 0; i < 6; ++i)
            {
                filtered_pose[i] = pos_filters[i].filter(handler_pose_[i]);
            }
            for (int i = 0; i < 3; ++i)
            {
                filtered_vel[i] = vel_filters[i].filter(handler_vel_linear[i]);
            }
            for (int i = 0; i < 7; ++i)
            {
                filtered_force[i] = force_filters[i].filter(handler_force[i]);
            }

            auto msg = sigema7_msg::msg::Sigema7Position();

            // 原始数据
            // msg.sigema7_pos[0] = handler_pose_[0];
            // msg.sigema7_pos[1] = handler_pose_[1];
            // msg.sigema7_pos[2] = handler_pose_[2];
            // msg.sigema7_pos[3] = handler_pose_[3];
            // msg.sigema7_pos[4] = handler_pose_[4];
            // msg.sigema7_pos[5] = handler_pose_[5];
            // msg.sigema7_vel_linear[0] = handler_vel_linear[0];
            // msg.sigema7_vel_linear[1] = handler_vel_linear[1];
            // msg.sigema7_vel_linear[2] = handler_vel_linear[2];
            // msg.sigema7_force[0] = handler_force[0];
            // msg.sigema7_force[1] = handler_force[1];
            // msg.sigema7_force[2] = handler_force[2];
            // msg.sigema7_force[3] = handler_force[3];
            // msg.sigema7_force[4] = handler_force[4];
            // msg.sigema7_force[5] = handler_force[5];
            // msg.sigema7_force[6] = handler_force[6];

            // 滤波后的数据
            msg.sigema7_pos[0] = filtered_pose[0];
            msg.sigema7_pos[1] = filtered_pose[1];
            msg.sigema7_pos[2] = filtered_pose[2];
            msg.sigema7_pos[3] = filtered_pose[3];
            msg.sigema7_pos[4] = filtered_pose[4];
            msg.sigema7_pos[5] = filtered_pose[5];
            msg.sigema7_vel_linear[0] = filtered_vel[0];
            msg.sigema7_vel_linear[1] = filtered_vel[1];
            msg.sigema7_vel_linear[2] = filtered_vel[2];
            msg.sigema7_force[0] = filtered_force[0];
            msg.sigema7_force[1] = filtered_force[1];
            msg.sigema7_force[2] = filtered_force[2];
            msg.sigema7_force[3] = filtered_force[3];
            msg.sigema7_force[4] = filtered_force[4];
            msg.sigema7_force[5] = filtered_force[5];
            msg.sigema7_force[6] = filtered_force[6];

            msg.sigema7_grip = handler_gripper;

            publisher_->publish(msg);

            //  // 构建要记录的消息字符串
            // std::stringstream ss;
            // ss << "Publishing:  Pos:";
            // for (int i = 0; i < 6; ++i) {
            //     ss << handler_pose_[i];
            //     if (i < 5) ss << ", ";
            // }
            // ss << ";  Vel:";
            // for (int i = 0; i < 3; ++i) {
            //     ss << handler_vel_linear[i];
            //     if (i < 3) ss << ", ";
            // }
            // ss << ";  Force:";
            // for (int i = 0; i < 7; ++i) {
            //     ss << handler_force[i];
            //     if (i < 7) ss << ", ";
            // }

            // // 使用RCLCPP_INFO宏输出消息
            // RCLCPP_INFO(this->get_logger(), ss.str().c_str());

            // 此线程延时
            std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime_ms));
        }
    }

    void sigema7_control_callback(const xmate_cr7_msg::msg::Cr7State::SharedPtr msg)
    {
        // 读取机械臂的位置和力
        for (size_t i = 0; i < msg->cr7_pos.size(); ++i)
        {
            cr7_pos[i] = msg->cr7_pos[i];
        }
        for (size_t i = 0; i < msg->cr7_force.size(); ++i)
        {
            cr7_force[i] = msg->cr7_force[i];
        }
        // 机械臂的位置和力旋转
        cr7_pos = rotatePose(cr7_pos);
        cr7_force = rotatePose(cr7_force);

        // // **方案一
        // // 机械臂的力直接映射到手柄当中
        // sigema7_force_target[0] = sigema7_force_default[0]+cr7_force[0] * force_ratio;
        // sigema7_force_target[1] = sigema7_force_default[1]+cr7_force[1] * force_ratio;
        // sigema7_force_target[2] = sigema7_force_default[2]+cr7_force[2] * force_ratio;
        // sigema7_force_target[3] = sigema7_force_default[3]+cr7_force[3] * torque_ratio;
        // sigema7_force_target[4] = sigema7_force_default[4]+cr7_force[4] * torque_ratio;
        // sigema7_force_target[5] = sigema7_force_default[5]+cr7_force[5] * torque_ratio;

        // // **方案二
        // // // 进行平均滤波处理
        // static MovingAverageFilter forceFilter(5); // 窗口大小为 5
        // static MovingAverageFilter torqueFilter(5);
        // // 动态调整滤波窗口大小
        // double linear_velocity = std::sqrt(std::pow(msg->cr7_vel[0], 2) + std::pow(msg->cr7_vel[1], 2) + std::pow(msg->cr7_vel[2], 2));
        // if (linear_velocity > 0.3) {
        //     forceFilter.setWindowSize(1000); // 运动时增加滤波窗口大小
        //     torqueFilter.setWindowSize(500);
        // } else {
        //     forceFilter.setWindowSize(800); // 恢复默认窗口大小
        //     torqueFilter.setWindowSize(400);
        // }
        // // 机械臂的力映射到手柄当中
        // for (size_t i = 0; i < 3; ++i) {
        //     forceFilter.filter(cr7_force[i]);
        //     if (std::abs(cr7_force[i]) < force_threshold) {
        //         sigema7_force_target[i] = sigema7_force_default[i];
        //     } else {
        //         sigema7_force_target[i] = sigema7_force_default[i] + cr7_force[i] * force_ratio;
        //     }
        // }
        // for (size_t i = 3; i < 6; ++i) {
        //     torqueFilter.filter(cr7_force[i]);
        //     if (std::abs(cr7_force[i]) < torque_threshold) {
        //         sigema7_force_target[i] = sigema7_force_default[i];
        //     } else {
        //         sigema7_force_target[i] = sigema7_force_default[i] + cr7_force[i] * torque_ratio;
        //     }
        // }

        // **方案三
        // // 卡尔曼滤波处理
        double linear_velocity = std::sqrt(std::pow(msg->cr7_vel[0], 2) + std::pow(msg->cr7_vel[1], 2) + std::pow(msg->cr7_vel[2], 2));
        // 动态调整卡尔曼滤波参数
        if (linear_velocity > 0.3)
        {
            // 运动时增加过程噪声协方差
            for (int i = 0; i < 6; ++i)
            {
                if (i < 3)
                {
                    force_filters_kalman[i].setQ(0.8); // 力的过程噪声协方差
                    force_filters_kalman[i].setR(2.2); // 力的测量噪声协方差
                }
                else
                {
                    force_filters_kalman[i].setQ(0.8); // 扭矩的过程噪声协方差
                    force_filters_kalman[i].setR(2.2); // 扭矩的测量噪声协方差
                }
            }
        }
        else
        {
            // 恢复默认参数
            for (int i = 0; i < 6; ++i)
            {
                if (i < 3)
                {
                    force_filters_kalman[i].setQ(1.0); // 力的过程噪声协方差
                    force_filters_kalman[i].setR(0.5); // 力的测量噪声协方差
                }
                else
                {
                    force_filters_kalman[i].setQ(1.0); // 扭矩的过程噪声协方差
                    force_filters_kalman[i].setR(0.5); // 扭矩的测量噪声协方差
                }
            }
        }

        // 机械臂的力映射到手柄当中
        for (size_t i = 0; i < 6; ++i)
        {
            double filtered_force = force_filters_kalman[i].filter(cr7_force[i]);
            if (i < 3)
            {
                if (std::abs(filtered_force) < force_threshold)
                {
                    sigema7_force_target[i] = sigema7_force_default[i];
                }
                else
                {
                    sigema7_force_target[i] = sigema7_force_default[i] + filtered_force * force_ratio;
                }
            }
            else
            {
                if (std::abs(filtered_force) < torque_threshold)
                {
                    sigema7_force_target[i] = sigema7_force_default[i];
                }
                else
                {
                    sigema7_force_target[i] = sigema7_force_default[i] + filtered_force * torque_ratio;
                }
            }
        }

        // 将力发送至手柄
        dhdSetForceAndTorque(sigema7_force_target[0], sigema7_force_target[1], sigema7_force_target[2], sigema7_force_target[3], sigema7_force_target[4], sigema7_force_target[5]);
        // dhdSetForceAndWristJointTorquesAndGripperForce(sigema7_force_target[0], sigema7_force_target[1], sigema7_force_target[2],sigema7_force_target[3], sigema7_force_target[4], sigema7_force_target[5],0.0);
        // dhdSetForceAndWristJointTorques(sigema7_force_target[0], sigema7_force_target[1], sigema7_force_target[2],sigema7_force_target[3], sigema7_force_target[4], sigema7_force_target[5]);
        // 打印手柄的力
        std::cout << "手柄的力：" << sigema7_force_target[0] << ", " << sigema7_force_target[1] << ", " << sigema7_force_target[2] << ", " << sigema7_force_target[3] << ", " << sigema7_force_target[4] << ", " << sigema7_force_target[5] << std::endl;
        // 打印通讯频率
        // double frequency = dhdGetComFreq();
        // std::cout << "交互频率：" << frequency*1000.0 << "Hz" <<std::endl;
    }

    // 函数用于计算绕原点旋转后的新位姿
    std::array<double, 6> rotatePose(const std::array<double, 6> &pose)
    {
        // 提取原始位姿
        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double original_rx = pose[3];
        double original_ry = pose[4];
        double original_rz = pose[5];

        // 手动调整xyz
        double rotated_x = y;
        double rotated_y = -x;
        double rotated_z = z;

        // 单独调整合适的RPY
        double rotated_rx = -1 * original_rx;
        double rotated_ry = -1 * original_ry;
        double rotated_rz = original_rz;

        // 返回新的位姿
        return {rotated_x, rotated_y, rotated_z, rotated_rx, rotated_ry, rotated_rz};
    }

    void ui_control_callback(const ui_control_msg::msg::UiControl::SharedPtr msg)
    {
        int control_flag = msg->control_flag;

        if (control_flag == 2) // 开启遥操作
        {

            if (open_flag == false)
            {
                // 主端手控器初始化
                MasterDeviceInit();
                // 创建发布者，发布名为 "sigema7_getpos" 的话题
                publisher_ = this->create_publisher<sigema7_msg::msg::Sigema7Position>("sigema7_getpos", 10);
                // 创建订阅者，订阅名为 "xmate_state" 的话题
                cr7_state_subscriber = this->create_subscription<xmate_cr7_msg::msg::Cr7State>(
                    "xmate_state", 2, std::bind(&Sigema7PosPublisher::sigema7_control_callback, this, std::placeholders::_1));
                // 开启发布线程
                sigema7_thread_ = std::thread(std::bind(&Sigema7PosPublisher::Sigema7InputListener, this));
                open_flag = true;
            }
            // 等待10ms
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            close_flag = false;
        }
        if (control_flag == 3) // 关闭遥操作模式
        {
            // // 关闭订阅者
            // cr7_state_subscriber.reset();
            // // 关闭发布者
            // publisher_.reset();

            // 停止发布
            close_flag = true;
        }
        if (control_flag == 4) // 关闭当前节点
        {
            RCLCPP_INFO(get_logger(), "关闭主端节点");
            // 关闭手控器
            dhdClose();
            // 关闭订阅者
            cr7_state_subscriber.reset();
            // 关闭发布者
            publisher_.reset();
            // 关闭线程
            if (sigema7_thread_.joinable())
            {
                sigema7_thread_.join();
            }
        }
    }
};

int main(int argc, char *argv[])
{
    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // // 将任务绑定到 CPU 核心 0
    // CPU_SET(1, &cpuset);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<Sigema7PosPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
