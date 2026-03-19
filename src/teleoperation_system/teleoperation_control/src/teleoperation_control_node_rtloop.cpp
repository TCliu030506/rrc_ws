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

class TransCoordinate
{
public:
    TransCoordinate()
    {
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
        T_OA = homogeneousTransform(rotation_OA, translation_OA);

        // 求 T_OA 的逆矩阵 T_AO
        T_AO = T_OA.inverse();

    };
    ~TransCoordinate(){};

    // 定义构建齐次变换矩阵的函数
    Eigen::Matrix4d homogeneousTransform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation;
        T.block<3, 1>(0, 3) = translation;
        return T;
    }

    // 将输入的rx,ry,rz表示转换为Eigen::Matrix3d& rotation
    // 输入的rx,ry,rz表示为弧度
    Eigen::Matrix3d eulerAnglesToRotationMatrix(double rx, double ry, double rz) {

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
    // 输入的rx,ry,rz表示为弧度
    Eigen::Matrix4d xyzRPYToHomogeneousTransform(double x, double y, double z, double rx, double ry, double rz) {
        Eigen::Matrix3d rotation = eulerAnglesToRotationMatrix(rx, ry, rz);
        Eigen::Vector3d translation(x, y, z);
        return homogeneousTransform(rotation, translation);
    }

    // 将齐次变换矩阵转换为 x, y, z, rx, ry, rz
    // 输出的rx,ry,rz表示为弧度
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

        // // 将弧度转换为角度
        // rx = rx * 180.0 / M_PI;
        // ry = ry * 180.0 / M_PI;
        // rz = rz * 180.0 / M_PI;

        return {x, y, z, rx, ry, rz};
    }

    // 输入A系下的机械臂位姿，返回A系下的内窥镜运动平台位姿
    // 输入和输出的rx,ry,rz表示为弧度
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

    // 输入A系下的内窥镜运动平台位姿，返回A系下的机械臂位姿
    std::array<double, 6> pf_trans_inv(std::array<double, 6> pos)
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
        Eigen::Matrix4d targetMatrixEigen = posMatrixEigen * transMatrixEigen.inverse();

        // 转换为RPY表示
        std::array<double, 6> targetPos = homogeneousTransformToXYZRPY(targetMatrixEigen);
        return targetPos;
    }   

    // 输入O系下的机械臂位姿，返回A系下的机械臂位姿
    std::array<double, 6> trans_OA(std::array<double, 6> pos)
    {
        // 构建 O 坐标系下的初始位姿 P_O 的齐次变换矩阵
        Eigen::Matrix4d P_O = xyzRPYToHomogeneousTransform(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

        // 计算 P_O 在 A 坐标系下的位姿 P_A
        Eigen::Matrix4d P_A = T_AO * P_O;

        // 将 P_A 转换为 x, y, z, rx, ry, rz 表示
        std::array<double, 6> P_A_rpy = homogeneousTransformToXYZRPY(P_A);

        return P_A_rpy;
            
    }

    // 输入A系下的机械臂位姿，返回O系下的机械臂位姿
    std::array<double, 6> trans_AO(std::array<double, 6> pos)
    {
        // 构建 A 坐标系下的初始位姿 P_A 的齐次变换矩阵
        Eigen::Matrix4d P_A = xyzRPYToHomogeneousTransform(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

        // 计算 P_O 在 O 坐标系下的位姿 P_O
        Eigen::Matrix4d P_O = T_OA * P_A;

        // 将 P_O 转换为 x, y, z, rx, ry, rz 表示
        std::array<double, 6> P_O_rpy = homogeneousTransformToXYZRPY(P_O);

        return P_O_rpy;
    }

    // 输入A系下的机械臂位姿，返回O系下的机械臂位姿(单行齐次矩阵)
    std::array<double, 16UL> trans_AO_16(std::array<double, 6> pos)
    {
        // 构建 A 坐标系下的初始位姿 P_A 的齐次变换矩阵
        Eigen::Matrix4d P_A = xyzRPYToHomogeneousTransform(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

        // 计算 P_O 在 O 坐标系下的位姿 P_O
        Eigen::Matrix4d P_O = T_OA * P_A;

        // 转换回单行齐次矩阵形式
        std::array<double, 16UL> xmate_posMatrix;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                xmate_posMatrix[i * 4 + j] = P_O(i, j);
            }
        }
        return xmate_posMatrix;
    }

    // 输入内窥镜运动平台的末端位姿，返回O系下的机械臂位姿（单行齐次矩阵）
    std::array<double, 16UL> trans_PFtoO(std::array<double, 6> pos)
    {
        // 求A坐标系下机械臂的末端位姿
        std::array<double, 6> P_A = pf_trans_inv(pos);
        // 求O坐标系下的机械臂位姿
        std::array<double, 16UL> P_O = trans_AO_16(P_A);
        return P_O;
    }


private:
    Eigen::Matrix4d T_OA; // 坐标系 A 相对于坐标系 O 的齐次变换矩阵(O-A)
    Eigen::Matrix4d T_AO; // 坐标系 O 相对于坐标系 A 的齐次变换矩阵(A-O)
};

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

        // // 创建机械臂状态发布者
        // xmate_state_publisher = this->create_publisher<xmate_cr7_msg::msg::Cr7State>("xmate_state", 10);

        // 睡眠一段时间，确保新线程已经启动
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // // 创建订阅者，订阅名为 "sigema7_getpos" 的话题
        tele_control_subscriber = this->create_subscription<sigema7_msg::msg::Sigema7Position>(
            "sigema7_getpos", 10, std::bind(&TeleControlClient::tele_control_callback, this, std::placeholders::_1));

        // pubThread= std::thread(&TeleControlClient::xmate_state_pub, this);

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

    TransCoordinate trans;

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
    double mapping_ratio = 1;
    double mapping_rad_ratio = 0.6;
    double default_speed = 50;
    double sigema7_pos_default[6] = {-0.0053974, -0.000809313, -0.00275382, 0.00204075, 0.001035195, -0.00308807}; // 用于矫正初始误差

    double end_pos_default[6] = {0.65091, 0.2, 0.1222, M_PI, 0, M_PI}; // A坐标系下的内窥镜运动平台末端位姿

    std::array<double, 6> sigema7_pos_mapping = {0, 0, 0, 0, 0, 0}; //手柄当前位置的映射
    std::array<double, 6> xmate_pos_mapping = {0, 0, 0, 0, 0, 0};   //手柄位置映射到机械臂控制模式末端位置


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

            // robot.getStateData(tauExt_inBase, rt_tau);
            // robot.getStateData(tcpVel_m, rt_vel);
            // 更新目标指令
            // JointPosition cmd = {{target_joint[0], target_joint[1], target_joint[2], target_joint[3], target_joint[4], target_joint[5] }};
            CartesianPosition cmd ;
            cmd.pos = target_pose;

            return cmd;
        };

        // 设置滤波器参数，用于平滑？
        rtCon->setFilterFrequency(10, 60, 10, ec) ;

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

    /**
     * @brief 回调函数，处理手柄输入并更新目标点位
     */

    void tele_control_callback(const sigema7_msg::msg::Sigema7Position::SharedPtr msg)
    {   
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
        sigema7_pos_mapping[0] = sigema7_pos[0] - sigema7_pos_default[0];
        sigema7_pos_mapping[1] = sigema7_pos[1] - sigema7_pos_default[1];
        sigema7_pos_mapping[2] = sigema7_pos[2] - sigema7_pos_default[2];
        sigema7_pos_mapping[3] = sigema7_pos[3] - sigema7_pos_default[3];
        sigema7_pos_mapping[4] = sigema7_pos[4] - sigema7_pos_default[4];
        sigema7_pos_mapping[5] = sigema7_pos[5] - sigema7_pos_default[5];
        // 调整映射旋转关系
        sigema7_pos_mapping[3] = sigema7_pos_mapping[3]*(-1); 
        sigema7_pos_mapping[4] = sigema7_pos_mapping[4]*(-1); 

        // 计算末端的目标位置
        xmate_pos_mapping[0] = end_pos_default[0] + sigema7_pos_mapping[0] * mapping_ratio;
        xmate_pos_mapping[1] = end_pos_default[1] + sigema7_pos_mapping[1] * mapping_ratio;
        xmate_pos_mapping[2] = end_pos_default[2] + sigema7_pos_mapping[2] * mapping_ratio;
        xmate_pos_mapping[3] = end_pos_default[3] + sigema7_pos_mapping[3] * mapping_rad_ratio;
        xmate_pos_mapping[4] = end_pos_default[4] + sigema7_pos_mapping[4] * mapping_rad_ratio;
        xmate_pos_mapping[5] = end_pos_default[5] + sigema7_pos_mapping[5] * mapping_rad_ratio * 0.6;

        // // 开始计时
        // auto start = std::chrono::high_resolution_clock::now();

        // 求出机械臂的末端位姿
        target_pose = trans.trans_PFtoO(xmate_pos_mapping);

        // // 结束计时
        // auto end = std::chrono::high_resolution_clock::now();
        // // 计算执行时间
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // // 打印执行时间
        // std::cout << "执行时间: " << duration.count() << " 毫秒" << std::endl;

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
