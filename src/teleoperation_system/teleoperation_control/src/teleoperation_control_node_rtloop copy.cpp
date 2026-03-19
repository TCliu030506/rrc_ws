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
#include "sigema7_msg/msg/sigema7_position.hpp"
#include "xmate_cr7_msg/msg/cr7_state.hpp"

#include <Eigen/Dense>

const double PI = 3.14159265359;
int flag_begin = 1;

using namespace std;
using namespace rokae;
using namespace rokae::RtSupportedFields;

// 定义卡尔曼滤波器类
class KalmanFilter {
    public:
        KalmanFilter(int dim_state, int dim_measurement, double dt, double u, double std_a, double std_z)
            : dim_state(dim_state), dim_measurement(dim_measurement), dt(dt), u(u), std_a(std_a), std_z(std_z) {
            // 初始化状态转移矩阵
            A = Eigen::MatrixXd::Identity(dim_state, dim_state);
            A.block(0, dim_state / 2, dim_state / 2, dim_state / 2) = dt * Eigen::MatrixXd::Identity(dim_state / 2, dim_state / 2);
            // 初始化控制输入矩阵
            B = Eigen::VectorXd::Zero(dim_state);
            B.segment(dim_state / 2, dim_state / 2) = dt * Eigen::VectorXd::Ones(dim_state / 2);
            // 初始化测量矩阵
            H = Eigen::MatrixXd::Zero(dim_measurement, dim_state);
            H.block(0, 0, dim_measurement, dim_measurement) = Eigen::MatrixXd::Identity(dim_measurement, dim_measurement);
            // 初始化过程噪声协方差矩阵
            Q = Eigen::MatrixXd::Zero(dim_state, dim_state);
            Q.block(0, 0, dim_state / 2, dim_state / 2) = 0.25 * std::pow(dt, 4) * std_a * std_a * Eigen::MatrixXd::Identity(dim_state / 2, dim_state / 2);
            Q.block(0, dim_state / 2, dim_state / 2, dim_state / 2) = 0.5 * std::pow(dt, 3) * std_a * std_a * Eigen::MatrixXd::Identity(dim_state / 2, dim_state / 2);
            Q.block(dim_state / 2, 0, dim_state / 2, dim_state / 2) = 0.5 * std::pow(dt, 3) * std_a * std_a * Eigen::MatrixXd::Identity(dim_state / 2, dim_state / 2);
            Q.block(dim_state / 2, dim_state / 2, dim_state / 2, dim_state / 2) = std::pow(dt, 2) * std_a * std_a * Eigen::MatrixXd::Identity(dim_state / 2, dim_state / 2);
            // 初始化测量噪声协方差矩阵
            R = std_z * std_z * Eigen::MatrixXd::Identity(dim_measurement, dim_measurement);
            // 初始化状态协方差矩阵
            P = Eigen::MatrixXd::Identity(dim_state, dim_state);
            // 初始化状态向量
            x_hat = Eigen::VectorXd::Zero(dim_state);
        }
    
        Eigen::VectorXd predict() {
            // 预测状态
            x_hat = A * x_hat + B * u;
            // 预测协方差
            P = A * P * A.transpose() + Q;
            return x_hat;
        }
    
        Eigen::VectorXd update(const Eigen::VectorXd& z) {
            // 计算卡尔曼增益
            Eigen::MatrixXd S = H * P * H.transpose() + R;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse();
            // 更新状态
            x_hat = x_hat + K * (z - H * x_hat);
            // 更新协方差
            P = (Eigen::MatrixXd::Identity(dim_state, dim_state) - K * H) * P;
            return x_hat;
        }
    
    private:
        int dim_state;
        int dim_measurement;
        double dt;  // 时间步长
        double u;   // 控制输入
        double std_a; // 过程噪声标准差
        double std_z; // 测量噪声标准差
    
        Eigen::MatrixXd A; // 状态转移矩阵
        Eigen::VectorXd B; // 控制输入矩阵
        Eigen::MatrixXd H; // 测量矩阵
        Eigen::MatrixXd Q; // 过程噪声协方差矩阵
        Eigen::MatrixXd R; // 测量噪声协方差矩阵
        Eigen::MatrixXd P; // 状态协方差矩阵
        Eigen::VectorXd x_hat; // 状态向量
};


class TeleControlClient : public rclcpp::Node
{
public:
    TeleControlClient() : Node("tele_control_client"), robot("192.168.2.160", "192.168.2.101")
    {
        // 初始化卡尔曼滤波器
        kalmanfilter_initialize();

        // 初始化机器人：
        robot_initialize();

        // 创建并启动新线程来运行rtloop函数
        open_rtControl_loop();

        // 创建机械臂状态发布者
        xmate_state_publisher = this->create_publisher<xmate_cr7_msg::msg::Cr7State>("xmate_state", 10);

        // 睡眠一段时间，确保新线程已经启动
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // // 创建订阅者，订阅名为 "sigema7_getpos" 的话题
        tele_control_subscriber = this->create_subscription<sigema7_msg::msg::Sigema7Position>(
            "sigema7_getpos", 10, std::bind(&TeleControlClient::tele_control_callback, this, std::placeholders::_1));

        pubThread= std::thread(&TeleControlClient::xmate_state_pub, this);

    }
    ~TeleControlClient() {
        // 停止新线程
        if (rtConThread.joinable()) {
            rtConThread.join();
        }
        // 关闭实时模式
        robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot.setPowerState(false, ec);
        robot.stopReceiveRobotState();
    };

private:
    rclcpp::Subscription<sigema7_msg::msg::Sigema7Position>::SharedPtr tele_control_subscriber;
    rclcpp::Publisher<xmate_cr7_msg::msg::Cr7State>::SharedPtr xmate_state_publisher;

    std::thread pubThread;

    xMateRobot robot;
    std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> rtCon;
    std::error_code ec;
    std::string id;

    std::array<double, 6> target_joint; // 目标关节位置
    std::array<double, 16> target_pose; //目标位姿，旋转矩阵表示
    thread rtConThread;

    std::array<double, 6> rt_pose; // 实时位姿，XYZrxryrz表示
    std::array<double, 6> rt_tau;  // 实时力矩
    std::array<double, 6> rt_vel;  // 实时速度

    bool running;
    double mapping_ratio = 2;
    double mapping_rad_ratio = 0.8;
    double speed_ratio = 0.7;
    double default_speed = 50;
    double sigema7_pos_default[6] = {-0.0053974, -0.000809313, -0.00275382, 0.00204075, 0.001035195, -0.00308807};
    double xmate_pos_default[6] = {0.150136, 0.360189, 0.659598, 179.999/180*M_PI, -0.002/180*M_PI, -90.0/180*M_PI};
    double xmate_pos_default_new[6] = {0.150136, 0.360189, 0.659598, 179.999/180*M_PI, -0.002/180*M_PI, -90.0/180*M_PI};


    double zone = 80;

    // 定义卡尔曼滤波器
    std::unique_ptr<KalmanFilter> kalman_filter;
    double dt;
    double u;
    double std_a;
    double std_z;
    int dim_state;
    int dim_measurement;

    // 初始化卡尔曼滤波器
    void kalmanfilter_initialize()
    {
        dt = 0.005; // 时间步长
        u = 0;     // 控制输入
        std_a = 20; // 过程噪声标准差
        std_z = 0.005; // 测量噪声标准差
        dim_state = 12; // 状态维度
        dim_measurement = 6; // 测量维度
        kalman_filter = std::make_unique<KalmanFilter>(dim_state, dim_measurement, dt, u, std_a, std_z);
    }


    void robot_initialize()
    {
        // 打印出测试字符
        std::cout << "机器人初始化开始" << std::endl;
        // 初始化机器人：设置操作模式、电源状态和运动重置
        double scale = 1;
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setPowerState(true, ec);
        robot.moveReset(ec);
        robot.adjustSpeedOnline(scale, ec);
        robot.setDefaultSpeed(default_speed, ec);
        robot.setDefaultZone(10, ec);
        robot.adjustAcceleration(0.2,0.1, ec);
        // 运动至机器人初始位置
        MoveAbsJCommand  startPosjoint({M_PI/2, 0, -M_PI/2, 0, -M_PI/2, 0});
        MoveAbsJCommand  startPosjointNew({-90.996/180*M_PI, 23.733/180*M_PI, -93.433/180*M_PI, 69.207/180*M_PI, -31.653/180*M_PI, -40.775/180*M_PI});
        MoveJCommand startPosNew({0.00820508,-0.53,0.385788,140.768/180*M_PI,37.7612/180*M_PI,63.4349/180*M_PI});
        robot.moveAppend({startPosNew}, id, ec);
        robot.moveStart(ec);
        waitRobot(robot, running);
        // 打印出测试字符 
        std::cout << "机器人初始化完成" << std::endl;
    }

    void rtControl_loop()
    {
        robot.setRtNetworkTolerance(4, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            std::cerr << "获取实时运动控制器失败" << std::endl;
            return;
        }
        robot.stopReceiveRobotState();
        robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                     {tcpPoseAbc_m, tauExt_inBase, tcpVel_m});

        // 定义回调函数
        std::function<CartesianPosition()> rtC_loop_callback = [&](){
            // 读取当前信息
            // robot.getStateData(tcpPoseAbc_m, rt_pose);
            robot.getStateData(tauExt_inBase, rt_tau);
            robot.getStateData(tcpVel_m, rt_vel);
            // 更新目标指令
            // JointPosition cmd = {{target_joint[0], target_joint[1], target_joint[2], target_joint[3], target_joint[4], target_joint[5] }};
            CartesianPosition cmd ;
            cmd.pos = target_pose;

            return cmd;
        };

        // 设置滤波器参数，用于平滑？
        rtCon->setFilterFrequency(10, 40, 10, ec) ;

        // 设置回调函数
        rtCon->setControlLoop(rtC_loop_callback, 0, true);
        // 开始轴空间位置控制
        // rtCon->startMove(RtControllerMode::jointPosition);
        rtCon->startMove(RtControllerMode::cartesianPosition);
        // 阻塞loop
        rtCon->startLoop(true);
    }

    void open_rtControl_loop()
    {
        target_joint = robot.jointPos(ec);
        std::array<double, 6UL> current_posture;
        current_posture = robot.posture(rokae::CoordinateType::flangeInBase, ec);
        Utils::postureToTransArray(current_posture,target_pose);
        rtConThread= std::thread(&TeleControlClient::rtControl_loop, this);
    }

    void rtControl_pos_update(std::array<double, 6> joint_angles)
    {
        // 更新目标位置
        target_joint = joint_angles;
    }

    // 函数用于计算绕原点旋转后的新位姿
    std::array<double, 6> rotatePose(const std::array<double, 6>& pose, double rx, double ry, double rz) {
        // 提取原始位姿
        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double original_rx = pose[3];
        double original_ry = pose[4];
        double original_rz = pose[5];

        // // 计算旋转矩阵
        // double cos_rx = std::cos(rx);
        // double sin_rx = std::sin(rx);
        // double cos_ry = std::cos(ry);
        // double sin_ry = std::sin(ry);
        // double cos_rz = std::cos(rz);
        // double sin_rz = std::sin(rz);

        // // 绕 X 轴旋转矩阵
        // double Rx[3][3] = {
        //     {1, 0, 0},
        //     {0, cos_rx, -sin_rx},
        //     {0, sin_rx, cos_rx}
        // };

        // // 绕 Y 轴旋转矩阵
        // double Ry[3][3] = {
        //     {cos_ry, 0, sin_ry},
        //     {0, 1, 0},
        //     {-sin_ry, 0, cos_ry}
        // };

        // // 绕 Z 轴旋转矩阵
        // double Rz[3][3] = {
        //     {cos_rz, -sin_rz, 0},
        //     {sin_rz, cos_rz, 0},
        //     {0, 0, 1}
        // };

        // // 组合旋转矩阵 R = Rz * Ry * Rx
        // double R[3][3];
        // for (int i = 0; i < 3; ++i) {
        //     for (int j = 0; j < 3; ++j) {
        //         R[i][j] = 0;
        //         for (int k = 0; k < 3; ++k) {
        //             R[i][j] += Rz[i][k] * (Ry[k][0] * Rx[0][j] + Ry[k][1] * Rx[1][j] + Ry[k][2] * Rx[2][j]);
        //         }
        //     }
        // }

        // // 计算旋转后的位置
        // double rotated_x = R[0][0] * x + R[0][1] * y + R[0][2] * z;
        // double rotated_y = R[1][0] * x + R[1][1] * y + R[1][2] * z;
        // double rotated_z = R[2][0] * x + R[2][1] * y + R[2][2] * z;
        
        //手动调整xyz
        double rotated_x = -y;
        double rotated_y = x;
        double rotated_z = z;

        // 单独调整合适的RPY
        double rotated_rx = -1*original_rx;
        double rotated_ry = -1*original_ry;
        double rotated_rz = original_rz;

        // 返回新的位姿
        return {rotated_x, rotated_y, rotated_z, rotated_rx, rotated_ry, rotated_rz};
    }

    /**
     * @brief 回调函数，处理手柄输入并更新目标点位
     */

    void tele_control_callback(const sigema7_msg::msg::Sigema7Position::SharedPtr msg)
    {   
        // std::cout << "进入回调函数: " << std::endl;
        // 记录开始时间
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        // **方法一：直接进行映射
        // 获取手柄的输入位置、速度和力
        std::array<double, 6> sigema7_pos = {0, 0, 0, 0, 0, 0};
        std::array<double, 3> sigema7_vel_linear= {0, 0, 0};
        std::array<double, 7> sigema7_force= {0, 0, 0, 0, 0, 0, 0};

        for (size_t i = 0; i < msg->sigema7_pos.size(); ++i)
        {
            sigema7_pos[i] = msg->sigema7_pos[i];
        }
        for (size_t i = 0; i < msg->sigema7_vel_linear.size(); ++i)
        {
            sigema7_vel_linear[i] = msg->sigema7_vel_linear[i];
        }
        for (size_t i = 0; i < msg->sigema7_force.size(); ++i)
        {
            sigema7_force[i] = msg->sigema7_force[i];
        }
        // 手柄位置映射到机械臂位置
        // 先计算手柄运动的映射
        std::array<double, 6> sigema7_pos_mapping = {0, 0, 0, 0, 0, 0};
        sigema7_pos_mapping[0] = sigema7_pos[0] - sigema7_pos_default[0];
        sigema7_pos_mapping[1] = sigema7_pos[1] - sigema7_pos_default[1];
        sigema7_pos_mapping[2] = sigema7_pos[2] - sigema7_pos_default[2];
        sigema7_pos_mapping[3] = sigema7_pos[3] - sigema7_pos_default[3];
        sigema7_pos_mapping[4] = sigema7_pos[4] - sigema7_pos_default[4];
        sigema7_pos_mapping[5] = sigema7_pos[5] - sigema7_pos_default[5];


        // // **方法二：尝试卡尔曼滤波器进行预测
        // Eigen::VectorXd sigema7_pos(6);
        // for (size_t i = 0; i < 6; ++i) {
        //     sigema7_pos[i] = msg->sigema7_pos[i];
        // }
        // // 更新卡尔曼滤波器
        // kalman_filter->predict();
        // Eigen::VectorXd sigema7_pos_predicted = kalman_filter->update(sigema7_pos).head(6);
        // std::array<double, 6> sigema7_pos_arr;
        // for (size_t i = 0; i < 6; ++i) {
        //     sigema7_pos_arr[i] = sigema7_pos_predicted[i];
        // }
        // // 使用预测的位置进行映射
        // std::array<double, 6> sigema7_pos_mapping = {0, 0, 0, 0, 0, 0};
        // sigema7_pos_mapping[0] = sigema7_pos_arr[0] - sigema7_pos_default[0];
        // sigema7_pos_mapping[1] = sigema7_pos_arr[1] - sigema7_pos_default[1];
        // sigema7_pos_mapping[2] = sigema7_pos_arr[2] - sigema7_pos_default[2];
        // sigema7_pos_mapping[3] = sigema7_pos_arr[3] - sigema7_pos_default[3];
        // sigema7_pos_mapping[4] = sigema7_pos_arr[4] - sigema7_pos_default[4];
        // sigema7_pos_mapping[5] = sigema7_pos_arr[5] - sigema7_pos_default[5];
      


        // 根据机械臂实际情况进行旋转
        sigema7_pos_mapping = rotatePose(sigema7_pos_mapping, 0, 0, 0);
        // 计算机械臂的目标位置
        std::array<double, 6> pos_mapping = {0, 0, 0, 0, 0, 0};
        pos_mapping[0] = xmate_pos_default[0] + sigema7_pos_mapping[0] * mapping_ratio;
        pos_mapping[1] = xmate_pos_default[1] + sigema7_pos_mapping[1] * mapping_ratio;
        pos_mapping[2] = xmate_pos_default[2] + sigema7_pos_mapping[2] * mapping_ratio;
        pos_mapping[3] = xmate_pos_default[3] + sigema7_pos_mapping[3] * mapping_rad_ratio;
        pos_mapping[4] = xmate_pos_default[4] + sigema7_pos_mapping[4] * mapping_rad_ratio;
        pos_mapping[5] = xmate_pos_default[5] + sigema7_pos_mapping[5] * mapping_rad_ratio;

        // 转换位姿表示形式并更新目标点位
        std::array<double, 6UL> pos_mapping_arr= {pos_mapping[0], pos_mapping[1], pos_mapping[2], pos_mapping[3], pos_mapping[4], pos_mapping[5]};
        Utils::postureToTransArray(pos_mapping_arr,target_pose);

        // // 手柄速度映射到机械臂速度
        // std::array<double, 3> vel_mapping = {0, 0, 0};
        // vel_mapping[0] = sigema7_vel_linear[0] * mapping_ratio * 3;
        // vel_mapping[1] = sigema7_vel_linear[1] * mapping_ratio * 3;
        // vel_mapping[2] = sigema7_vel_linear[2] * mapping_ratio * 3;

        // // 调整末端执行器的速度
        // double tool_speed = std::sqrt(std::pow(vel_mapping[0], 2) + std::pow(vel_mapping[1], 2) + std::pow(vel_mapping[2], 2)) * 6;
        // tool_speed = tool_speed*1000; // 单位为mm/s
        // // if (tool_speed > 800)tool_speed = 800;
        // speed_ratio = tool_speed/default_speed * 2;


        // CoordinateType ct = CoordinateType::flangeInBase;
        // CartesianPosition startPosition = robot.posture(ct,ec);
        // CartesianPosition targetPosition({pos_mapping[0], pos_mapping[1], pos_mapping[2], pos_mapping[3], pos_mapping[4], pos_mapping[5]});

        // std::cout << "运动速度系数： " << std::endl;
        // std::cout << speed_ratio << std::endl;
        // std::cout << "当前位置： " << std::endl;
        // std::cout << startPosition.trans[0] << ',' << startPosition.trans[1] << ',' << startPosition.trans[2] << ',' << startPosition.rpy[0] << ',' << startPosition.rpy[1] << ',' << startPosition.rpy[2] << std::endl;
        // std::cout << "目标位置： " << std::endl;
        // std::cout << targetPosition.trans[0] << ',' << targetPosition.trans[1] << ',' << targetPosition.trans[2] << ',' << targetPosition.rpy[0] << ',' << targetPosition.rpy[1] << ',' << targetPosition.rpy[2] << std::endl;
        

        // // 记录逆运动学开始时间
        // // auto inv_start_time = std::chrono::high_resolution_clock::now();
        // // 将映射后的目标位置转换为旋转矩阵，随后进行逆运动学求解
        // std::array<double, 6UL> pos_mapping_arr= {pos_mapping[0], pos_mapping[1], pos_mapping[2], pos_mapping[3], pos_mapping[4], pos_mapping[5]};
        // std::array<double, 16UL> targetPositionMatrix;
        // Utils::postureToTransArray(pos_mapping_arr,targetPositionMatrix);

        // auto inv_start_time = std::chrono::high_resolution_clock::now();
        // std::array<double, 6> init_jointPos = robot.jointPos(ec);
        // std::array<double, 6> target_jointPos = {};
        // auto model = robot.model();
        // model.getJointPos(targetPositionMatrix,0,init_jointPos,target_jointPos);
        // // 记录逆运动学结束时间
        // auto inv_end_time = std::chrono::high_resolution_clock::now();
        // // 计算耗时
        // auto duration_inv = std::chrono::duration_cast<std::chrono::microseconds>(inv_end_time - inv_start_time).count();
        // std::cout << "逆运动学耗时: " << duration_inv << " 微秒" << std::endl;
        // // 点位跟随目标点更改
        // rtControl_pos_update(target_jointPos);

        // 记录结束时间
        // auto end_time = std::chrono::high_resolution_clock::now();
        // 计算耗时
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        // std::cout << "此部分代码耗时: " << duration << " 微秒" << std::endl;


    }

    /**
     * @brief 等待运动结束 - 通过查询机械臂是否处于运动中的方式
     */
    void waitRobot(BaseRobot &robot, bool &running) {
    running = true;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        error_code ec;
        auto st = robot.operationState(ec);
        if(st == OperationState::idle || st == OperationState::unknown){
        running = false;
        }
    }
    }

    // 发布机械臂状态
    void xmate_state_pub()
    {
        while (rclcpp::ok()) 
        {
            // // 打印信息
            // std::cout << "当前位姿：" << std::endl;
            // std::cout << "X: " << rt_pose[0] << std::endl;
            // std::cout << "Y: " << rt_pose[1] << std::endl;
            // std::cout << "Z: " << rt_pose[2] << std::endl;
            // std::cout << "RX: " << rt_pose[3] << std::endl;
            // std::cout << "RY: " << rt_pose[4] << std::endl;
            // std::cout << "RZ: " << rt_pose[5] << std::endl;
            // std::cout << "当前力矩：" << std::endl;
            // for (int i = 0; i < 6; ++i) {
            //     std::cout << "关节 " << i << ": " << rt_tau[i] << std::endl;
            // }
            // 构建消息并发布实时状态
            auto msg = xmate_cr7_msg::msg::Cr7State();
            msg.cr7_pos[0] = rt_pose[0];
            msg.cr7_pos[1] = rt_pose[1];
            msg.cr7_pos[2] = rt_pose[2];
            msg.cr7_pos[3] = rt_pose[3];
            msg.cr7_pos[4] = rt_pose[4];
            msg.cr7_pos[5] = rt_pose[5];
            msg.cr7_force[0] = rt_tau[0];
            msg.cr7_force[1] = rt_tau[1];
            msg.cr7_force[2] = rt_tau[2];
            msg.cr7_force[3] = rt_tau[3];
            msg.cr7_force[4] = rt_tau[4];
            msg.cr7_force[5] = rt_tau[5];
            msg.cr7_vel[0] = rt_vel[0];
            msg.cr7_vel[1] = rt_vel[1];
            msg.cr7_vel[2] = rt_vel[2];
            msg.cr7_vel[3] = rt_vel[3];
            msg.cr7_vel[4] = rt_vel[4];
            msg.cr7_vel[5] = rt_vel[5];

            xmate_state_publisher->publish(msg);
            // 休眠一段时间
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

    
int main(int argc, char * argv[])
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    // 将任务绑定到 CPU 核心 0
    CPU_SET(0, &cpuset); 

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleControlClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
