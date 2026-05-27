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
#include "pf_control/pf_control.h"
#include "ui_control_msg/msg/ui_control.hpp"
#include "ui_control_msg/msg/slaver_position.hpp"

const double PI = 3.14159265359;
int flag_begin = 1;

using namespace std;
using namespace rokae;
using namespace rokae::RtSupportedFields;

class TransCoordinate
{
    // 相关坐标系定义备注：
    // O系：机械臂的基坐标系/世界坐标系（为机械臂运动控制指令的默认参考坐标系）
    // A系：相机坐标系（坐标系原点在机械臂坐标系原点，x轴指向视角右侧，y轴指向前侧，z轴指向上方）
    // PF系：PF机构的工具坐标系（坐标系原点在PF的前端连续体基座，指向视角右侧，y轴指向前侧，z轴指向上方）

public:
    TransCoordinate()
    {
        // 定义机械臂安装位置（世界坐标系）旋转角度，用于计算T_OA，顺时针旋转为负角度
        // double theta_y = -M_PI / 6; // 绕 Y 轴顺时针旋转 30 度
        double theta_y = 0.0; // 绕 Y 轴旋转取消（即将O系改为机械臂的世界坐标系）
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
        // 组合旋转矩阵，顺序为 Y 先 Z 后 
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

    // 输入O系下的机械臂位姿态（[x,y,z,rx,ry,rz]），返回内窥镜运动平台的末端位姿
    std::array<double, 6UL> trans_OtoPF(std::array<double, 6> pos)
    {
        // 求A坐标系下机械臂的末端位姿
        std::array<double, 6> P_A = trans_OA(pos);
        // 求A坐标系下内窥镜运动平台的位姿
        std::array<double, 6> P_A_PF = pf_trans(P_A);
        return P_A_PF;
    }


private:
    Eigen::Matrix4d T_OA; // 坐标系 A 相对于坐标系 O 的齐次变换矩阵(O-A)
    Eigen::Matrix4d T_AO; // 坐标系 O 相对于坐标系 A 的齐次变换矩阵(A-O)

    // 定义PF的几何参数及坐标系转换（法兰至PF末端）
    // // 版本1
    // double d1 = 0.12091;
    // double d2 = 0.2078;
    // double alpha = 45.0/180*M_PI;
    // double l = sqrt(pow(d1, 2) + pow(d2, 2));
    // double fi = atan2(d1, d2);
    // std::array<double, 6UL> trans_pos = {l*sin(alpha-fi), 0, l*cos(alpha-fi), 0, alpha, 0};
    // 版本2
    std::array<double, 6UL> trans_pos = {-0.04196, 0, 0.16989, 0, 135.0/180*M_PI, 0};
};


class TeleControlClient : public rclcpp::Node
{
public:
    TeleControlClient() : Node("tele_control_client"), robot("192.168.2.160", "192.168.2.101"),controller("/dev/ttyUSB0")
    {
        // 初始化订阅者，订阅名为 "ui_control_topic" 的话题
        ui_control_subscriber = this->create_subscription<ui_control_msg::msg::UiControl>(
            "ui_control_topic", 10, std::bind(&TeleControlClient::ui_control_callback, this, std::placeholders::_1));

        // 初始化订阅者，订阅名为 "ui_mode_topic" 的话题 
        ui_mode_subscriber = this->create_subscription<ui_control_msg::msg::UiControl>(
            "ui_mode_topic", 10, std::bind(&TeleControlClient::ui_mode_callback, this, std::placeholders::_1));

        // 初始化发布者，发布名为 "slave_position_topic" 的话题
        slaver_position_publisher = this->create_publisher<ui_control_msg::msg::SlaverPosition>("slave_position_topic", 10);

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
    rclcpp::Publisher<xmate_cr7_msg::msg::Cr7State>::SharedPtr xmate_state_publisher;
    rclcpp::Subscription<sigema7_msg::msg::Sigema7Position>::SharedPtr tele_control_subscriber;
    rclcpp::Subscription<ui_control_msg::msg::UiControl>::SharedPtr ui_control_subscriber;
    rclcpp::Subscription<ui_control_msg::msg::UiControl>::SharedPtr ui_mode_subscriber;
    rclcpp::Publisher<ui_control_msg::msg::SlaverPosition>::SharedPtr slaver_position_publisher;

    std::thread rtConThread;
    std::thread xmate_pubThread;

    xMateRobot robot;
    std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> rtCon;
    std::error_code ec;
    std::string id;

    pf_control controller;

    TransCoordinate trans;

    int control_mode_falg = 0; // 0: 机械臂控制 1: PF控制
    int mode_change_flag = 0; // 0: 机械臂模式 1: PF模式

    array<double, 6> target_joint; // 机械臂的的目标关节位置
    std::array<double, 16> target_pose; //机械臂的目标位姿，旋转矩阵表示

    std::array<double, 6> sigema7_pos = {0, 0, 0, 0, 0, 0};     // 手柄的当前位置
    std::array<double, 3> sigema7_vel_linear= {0, 0, 0};        // 手柄的当前速度
    std::array<double, 7> sigema7_force= {0, 0, 0, 0, 0, 0, 0}; // 手柄的当前受力
    double sigema7_grip = 0;                                    // 手柄的当前夹爪位置

    std::array<double, 6> sigema7_pos_mapping = {0, 0, 0, 0, 0, 0}; //手柄当前位置的映射
    std::array<double, 6> xmate_pos_mapping = {0, 0, 0, 0, 0, 0};   //手柄位置映射到机械臂控制模式末端位置
    std::array<double, 3> pf_pos_mapping = {0.020696, 0.0152364, -56.43};   //手柄位置映射到PF控制模式末端位置


    std::array<double, 6> rt_pose; // 实时位姿，XYZrxryrz表示
    std::array<double, 6> rt_tau;  // 实时力矩
    std::array<double, 6> rt_vel;  // 实时速度

    bool running;
    double mapping_ratio = 1.0;
    double mapping_rad_ratio = 0.5;
    double speed_ratio = 0.7;
    double default_speed = 50;
    // double sigema7_pos_default[6] = {-0.0029, -0.00034, -0.0015, 0.00204075, 0.001035195, -0.00308807};
    double sigema7_pos_default[6] = {-0.00106, 0.000537754, -0.000932163, 0.00339632, 0.00214078, -0.00370568};

    double end_pos_default[6] = {0.65091, 0.2, 0.1222, M_PI, 0, M_PI}; // A坐标系下的内窥镜运动平台末端位姿
    // double pf_pos_default[3] = {-0.0202441, -0.0287207, -62.6799};  //单位为mm
    double pf_pos_default[3] = {0.0147391, -0.0318873, -62.6703};  //单位为mm

    double loop_time = 0.001; //单位为秒

    double zone = 80;

    bool open_flag = false;
    bool reopen_flag = false;
    bool pf_open_flag = false;

    int control_flag;


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
        robot.adjustAcceleration(0.5,0.5, ec);
        // 运动至机器人初始位置
        // MoveJCommand startPosNew({0.008312,-0.622970,0.66055,140.591/180*M_PI,37.393/180*M_PI,63.092/180*M_PI});//第一次
        // MoveJCommand startPosNew({0.039334,-0.513416,0.756946,140.593/180*M_PI,37.389/180*M_PI,63.096/180*M_PI});//第二次
        // MoveJCommand startPosNew({-0.091,-0.651,0.453,140.593/180*M_PI,37.389/180*M_PI,63.096/180*M_PI});//第毛笔字测试
        // MoveJCommand startPosNew({0.400,-0.450,0.450,-M_PI,0.0,0.0});//第毛笔字测试
        // MoveJCommand startPosNew({0.300,-0.450,0.550,M_PI,-42.0/180*M_PI,-90.0/180*M_PI});//260109 直线版本遥操作 初始位置
        MoveJCommand startPosNew({0.300,-0.450,0.450,M_PI,-42.0/180*M_PI,-90.0/180*M_PI});//260112 直线版本遥操作 初始位置
        robot.moveAppend({startPosNew}, id, ec);
        robot.moveStart(ec);
        waitRobot(robot, running);
        // 打印出测试字符
        std::cout << "机器人初始化完成" << std::endl;
    }

    // 从当前位置移动到指定位置
    void automatic_navigation()
    {
        // MoveJCommand puncture_pos({0.00820508,-0.53,0.455788,140.768/180*M_PI,37.7612/180*M_PI,63.4349/180*M_PI});
        // MoveJCommand puncture_pos({0.008312,-0.622970,0.66055,158.067/180*M_PI,46.723/180*M_PI,79.921/180*M_PI});
        // MoveJCommand puncture_pos({0.159554,-0.784473,0.529205,162.029/180*M_PI,47.613/180*M_PI,78.528/180*M_PI});// 第一次
        MoveJCommand puncture_pos({0.198030,-0.698607,0.697987,159.541/180*M_PI,48.943/180*M_PI,68.278/180*M_PI});// 第二次
        robot.moveAppend({puncture_pos}, id, ec);
        robot.moveStart(ec);
        waitRobot(robot, running);
        // 打印出测试字符
        std::cout << "机器人自动导航完成" << std::endl;
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
            // 关节空间位置控制
            // JointPosition cmd = {{target_joint[0], target_joint[1], target_joint[2], target_joint[3], target_joint[4], target_joint[5] }};
            // 笛卡尔空间位置控制
            CartesianPosition cmd ;
            cmd.pos = target_pose;

            return cmd;
        };

        // 设置滤波器参数，用于平滑？
        rtCon->setFilterFrequency(10, 50, 10, ec) ;

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
     * @brief 订阅者回调函数，处理手柄输入并更新目标点位
     */
    void tele_control_callback(const sigema7_msg::msg::Sigema7Position::SharedPtr msg)
    {   
        // std::cout << "进入回调函数: " << std::endl;
        // 获取手柄的输入位置、速度和力
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
        sigema7_grip = msg->sigema7_grip;

        // 停止遥操作后不映射
        if(control_flag == 3) return;

        // 机械臂控制模式
        if(control_mode_falg == 0)
        {   
            // 模式切换衔接
            if(mode_change_flag == 1)
            {
                mode_change_flag = 0;
                mapping_ratio = 1.0;
                //将手柄当前位置认为是初始值
                for(int i = 0; i < 6; i++)
                {
                    sigema7_pos_default[i] = sigema7_pos[i];
                }
                // 将末端当前位置认为是初始值
                for(int i = 0; i < 6; i++)
                {
                    end_pos_default[i] = xmate_pos_mapping[i];
                }
                
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
            xmate_pos_mapping[5] = end_pos_default[5] + sigema7_pos_mapping[5] * mapping_rad_ratio;

            // 求出机械臂的末端位姿
            target_pose = trans.trans_PFtoO(xmate_pos_mapping);
            
        }
        // 穿刺模式
        else if(control_mode_falg == 1)
        {
            // 模式切换衔接
            if(mode_change_flag == 1)
            {
                mode_change_flag = 0;
                mapping_ratio = 0.2;
                //将手柄当前位置认为是初始值
                for(int i = 0; i < 6; i++)
                {
                    sigema7_pos_default[i] = sigema7_pos[i];
                }
                // 将末端当前位置认为是初始值
                for(int i = 0; i < 6; i++)
                {
                    end_pos_default[i] = xmate_pos_mapping[i];
                }
            }
            // 限制机械臂沿着当前末端方向做插入运动
            // RPY转换为旋转矩阵
            Eigen::Matrix3d R = trans.eulerAnglesToRotationMatrix(end_pos_default[3], end_pos_default[4], end_pos_default[5]);
            // 末端Z方向；向量
            R.col(2).normalize();
            Eigen::Vector3d direction = R.col(2);
            // 先不对主端进行限制，进行映射

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

            // double distance = sqrt(pow(sigema7_pos_mapping[0], 2) + pow(sigema7_pos_mapping[1], 2) + pow(sigema7_pos_mapping[2], 2)); // 将手柄距离初始位置的距离作为插入量
            double distance = -sigema7_pos_mapping[2]; // 将手柄距离Z向运动作为插入量
            // 计算末端的目标位置
            xmate_pos_mapping[0] = end_pos_default[0] + distance * direction[0] * mapping_ratio * 5;
            xmate_pos_mapping[1] = end_pos_default[1] + distance * direction[1] * mapping_ratio * 5;
            xmate_pos_mapping[2] = end_pos_default[2] + distance * direction[2] * mapping_ratio * 5;
            xmate_pos_mapping[3] = end_pos_default[3] ;
            xmate_pos_mapping[4] = end_pos_default[4] ;
            xmate_pos_mapping[5] = end_pos_default[5] ;

            // 求出机械臂的末端位姿
            target_pose = trans.trans_PFtoO(xmate_pos_mapping);
        }
        // PF控制模式
        else if(control_mode_falg == 2)
        {
            // 如果是第一次运行PF控制模式，则读取当前末端位值认为是初始位置
            if(!pf_open_flag)
            {
                controller.pr_readp(pf_pos_default[0], pf_pos_default[1], pf_pos_default[2]);
                pf_open_flag = true;
            }
            // 模式切换衔接
            if(mode_change_flag == 1)
            {
                mode_change_flag = 0;
                mapping_ratio = 0.12;
                //将手柄当前位置认为是初始值
                for(int i = 0; i < 6; i++)
                {
                    sigema7_pos_default[i] = sigema7_pos[i];
                }
                // 将末端当前位置认为是初始值
                for(int i = 0; i < 3; i++)
                {
                    pf_pos_default[i] = pf_pos_mapping[i];
                }
            }

            // 当目标点不超出运动范围,仅控制内窥镜运动平台运动
            // 手柄位置映射到机械臂位置
            // 先计算手柄运动的映射
            sigema7_pos_mapping[0] = sigema7_pos[0] - sigema7_pos_default[0];
            sigema7_pos_mapping[1] = sigema7_pos[1] - sigema7_pos_default[1];
            sigema7_pos_mapping[2] = sigema7_pos[2] - sigema7_pos_default[2];
            sigema7_pos_mapping[3] = sigema7_pos[3] - sigema7_pos_default[3];
            sigema7_pos_mapping[4] = sigema7_pos[4] - sigema7_pos_default[4];
            sigema7_pos_mapping[5] = sigema7_pos[5] - sigema7_pos_default[5];
            // 根据机械臂实际情况进行旋转
            // sigema7_pos_mapping = rotatePose(sigema7_pos_mapping);
            
            //计算偏心距离
            double distance_to_axil = sqrt(pow(sigema7_pos_mapping[0]*1000, 2) + pow(sigema7_pos_mapping[1]*1000, 2))* mapping_ratio;

            // 计算PF的目标位置
            pf_pos_mapping[0] = pf_pos_default[0] + sigema7_pos_mapping[0] * 1000 * mapping_ratio;
            pf_pos_mapping[1] = pf_pos_default[1] + sigema7_pos_mapping[1] * 1000 * mapping_ratio;
            pf_pos_mapping[2] = pf_pos_default[2] + sigema7_pos_mapping[2] * 1000 * distance_to_axil/15 * mapping_ratio;
            
            // 将位置发送至内窥镜运动平台
            // controller.pf_movep(pf_pos_mapping[0], pf_pos_mapping[1], pf_pos_mapping[2], loop_time);

            // // 测试用
            // std::cout << "进入PF控制" << std::endl;
            
            bool safe_range_flag = controller.pf_movep_withflag(pf_pos_mapping[0], pf_pos_mapping[1], pf_pos_mapping[2], loop_time);

            // 如果目标点超出运动范围,在ROS2中广播出来
            if(!safe_range_flag)
            {
                RCLCPP_INFO(get_logger(),"目标点超出运动范围");
            }

            // 当目标点超出运动范围,通过机械臂运动补偿
            // if(!safe_range_flag)
            // {
            //     // 说明开始机械臂补偿
            //     RCLCPP_INFO(get_logger(),"开始机械臂补偿");
            //     // 定义补偿后的内窥镜运动平台末端位置
            //     double pf_target[3] = {pf_pos_mapping[0],pf_pos_mapping[1],pf_pos_mapping[2]};
            //     // 先尝试机械臂正向补偿(0-30mm)
            //     for(double z_delta=0;z_delta<30;z_delta+=0.5)
            //     {
            //         RCLCPP_INFO(get_logger(),"开始机械臂正向补偿");

            //         pf_target[2] = pf_pos_mapping[2] + z_delta;
            //         safe_range_flag = controller.pf_movep_withflag(pf_target[0], pf_target[1], pf_target[2], loop_time);
            //         if (safe_range_flag) 
            //         {
            //             RCLCPP_INFO(get_logger(),"机械臂正向补偿成功");

            //             // 补偿成功,控制机械臂运动
            //             // 将机械臂当前位置作为机械臂运动初始位置
            //             for(int i = 0; i < 6; i++)
            //             {
            //                 end_pos_default[i] = xmate_pos_mapping[i];
            //             }
            //             // 限制机械臂沿着当前末端方向做插入运动
            //             // RPY转换为旋转矩阵
            //             Eigen::Matrix3d R = trans.eulerAnglesToRotationMatrix(end_pos_default[3], end_pos_default[4], end_pos_default[5]);
            //             // 末端Z方向；向量
            //             R.col(2).normalize();
            //             Eigen::Vector3d direction = R.col(2);
            //             // 补偿z_delta
            //             xmate_pos_mapping[0] = end_pos_default[0] + z_delta * 0.001 * direction[0];
            //             xmate_pos_mapping[1] = end_pos_default[1] + z_delta * 0.001 * direction[1];
            //             xmate_pos_mapping[2] = end_pos_default[2] + z_delta * 0.001 * direction[2];

            //             // 求出机械臂的末端位姿
            //             // target_pose = trans.trans_PFtoO(xmate_pos_mapping);

            //             // 更新运动平台目标位置
            //             pf_pos_mapping[0] = pf_target[0];
            //             pf_pos_mapping[1] = pf_target[1];
            //             pf_pos_mapping[2] = pf_target[2];
            //             // 再次进行模式切换转换
            //             mode_change_flag = 1;

            //             break;
            //         }
            //     }
            //     // 当机械臂正向补偿失败,尝试机械臂反向补偿(0-30mm)
            //     if(!safe_range_flag)
            //     {   
            //         RCLCPP_INFO(get_logger(),"正向补偿失败，开始机械臂反向补偿");
            //         for(double z_delta=0;z_delta<30;z_delta+=0.5)
            //         {
            //             pf_target[2] = pf_pos_mapping[2] - z_delta;
            //             safe_range_flag = controller.pf_movep_withflag(pf_target[0], pf_target[1], pf_target[2], loop_time);
            //             if (safe_range_flag) 
            //             {
            //                 RCLCPP_INFO(get_logger(),"机械臂反向补偿成功");
            //                 // 补偿成功,控制机械臂运动
            //                 // 将机械臂当前位置作为机械臂运动初始位置
            //                 for(int i = 0; i < 6; i++)
            //                 {
            //                     end_pos_default[i] = xmate_pos_mapping[i];
            //                 }
            //                 // 限制机械臂沿着当前末端方向做插入运动
            //                 // RPY转换为旋转矩阵
            //                 Eigen::Matrix3d R = trans.eulerAnglesToRotationMatrix(end_pos_default[3], end_pos_default[4], end_pos_default[5]);
            //                 // 末端Z方向；向量
            //                 R.col(2).normalize();
            //                 Eigen::Vector3d direction = R.col(2);
            //                 // 补偿z_delta
            //                 xmate_pos_mapping[0] = end_pos_default[0] - z_delta * 0.001 * direction[0];
            //                 xmate_pos_mapping[1] = end_pos_default[1] - z_delta * 0.001 * direction[1];
            //                 xmate_pos_mapping[2] = end_pos_default[2] - z_delta * 0.001 * direction[2];

            //                 // 求出机械臂的末端位姿
            //                 // target_pose = trans.trans_PFtoO(xmate_pos_mapping);

            //                 // 更新运动平台目标位置
            //                 pf_pos_mapping[0] = pf_target[0];
            //                 pf_pos_mapping[1] = pf_target[1];
            //                 pf_pos_mapping[2] = pf_target[2];
            //                 // 再次进行模式切换转换
            //                 mode_change_flag = 1;
                            
            //                 break;
            //             }
            //         }
            //     }
            //     // 当机械臂反向补偿失败,尝试机械臂Z轴正向旋转补偿(0-60度)
            //     if(!safe_range_flag)
            //     {
            //         RCLCPP_INFO(get_logger(),"反向补偿失败，开始机械臂正向旋转补偿");
            //         for(double theta_delta=0;theta_delta<60;theta_delta+=1)
            //         {
            //             // 对pf_target绕Z轴旋转theta_delta度
            //             pf_target[0] = pf_pos_mapping[0] * cos(theta_delta/180*M_PI) - pf_pos_mapping[1] * sin(theta_delta/180*M_PI);
            //             pf_target[1] = pf_pos_mapping[0] * sin(theta_delta/180*M_PI) + pf_pos_mapping[1] * cos(theta_delta/180*M_PI);
            //             safe_range_flag = controller.pf_movep_withflag(pf_target[0], pf_target[1], pf_target[2], loop_time);
            //             if (safe_range_flag) 
            //             {
            //                 RCLCPP_INFO(get_logger(),"机械臂正向旋转补偿成功");
            //                 // 补偿成功,控制机械臂运动
            //                 // 将机械臂当前位置作为机械臂运动初始位置
            //                 for(int i = 0; i < 6; i++)
            //                 {
            //                     end_pos_default[i] = xmate_pos_mapping[i];
            //                 }
            //                 // 旋转机械臂Z轴theta_delta度
            //                 xmate_pos_mapping[5] = end_pos_default[5] + theta_delta/180*M_PI;
            //                 // 求出机械臂的末端位姿
            //                 // target_pose = trans.trans_PFtoO(xmate_pos_mapping);

            //                 // 更新运动平台目标位置
            //                 pf_pos_mapping[0] = pf_target[0];
            //                 pf_pos_mapping[1] = pf_target[1];
            //                 pf_pos_mapping[2] = pf_target[2];
            //                 // 再次进行模式切换转换
            //                 mode_change_flag = 1;

            //                 break;
            //             }
            //         }
            //     }
            //     // 当机械臂Z轴正向旋转补偿失败,尝试机械臂Z轴反向旋转补偿(0-60度)
            //     if(!safe_range_flag)
            //     {
            //         RCLCPP_INFO(get_logger(),"正向旋转补偿失败，开始机械臂反向旋转补偿");
            //         for(double theta_delta=0;theta_delta<60;theta_delta+=1)
            //         {
            //             // 对pf_target绕Z轴旋转theta_delta度
            //             pf_target[0] = pf_pos_mapping[0] * cos(-theta_delta/180*M_PI) - pf_pos_mapping[1] * sin(-theta_delta/180*M_PI);
            //             pf_target[1] = pf_pos_mapping[0] * sin(-theta_delta/180*M_PI) + pf_pos_mapping[1] * cos(-theta_delta/180*M_PI);
            //             safe_range_flag = controller.pf_movep_withflag(pf_target[0], pf_target[1], pf_target[2], loop_time);
            //             if (safe_range_flag) 
            //             {
            //                 RCLCPP_INFO(get_logger(),"机械臂反向旋转补偿成功");
            //                 // 补偿成功,控制机械臂运动
            //                 // 将机械臂当前位置作为机械臂运动初始位置
            //                 for(int i = 0; i < 6; i++)
            //                 {
            //                     end_pos_default[i] = xmate_pos_mapping[i];
            //                 }
            //                 // 旋转机械臂Z轴theta_delta度
            //                 xmate_pos_mapping[5] = end_pos_default[5] - theta_delta/180*M_PI;
            //                 // 求出机械臂的末端位姿
            //                 // target_pose = trans.trans_PFtoO(xmate_pos_mapping);

            //                 // 更新运动平台目标位置
            //                 pf_pos_mapping[0] = pf_target[0];
            //                 pf_pos_mapping[1] = pf_target[1];
            //                 pf_pos_mapping[2] = pf_target[2];
            //                 // 再次进行模式切换转换
            //                 mode_change_flag = 1;

            //                 break;
            //             }
            //         }
            //     }
            //     // 打印是否补偿成功
            //     if(safe_range_flag)  RCLCPP_INFO(get_logger(),"机械臂补偿成功！");
            //     else RCLCPP_INFO(get_logger(),"机械臂补偿失败！");
            // }

            
        }
        // 穿刺模式
        else if(control_mode_falg == 3)
        {
            // 模式切换衔接
            if(mode_change_flag == 1)
            {
                mode_change_flag = 0;
                mapping_ratio = 0.05;
                //将手柄当前位置认为是初始值
                for(int i = 0; i < 6; i++)
                {
                    sigema7_pos_default[i] = sigema7_pos[i];
                }
                // 将末端当前位置认为是初始值
                for(int i = 0; i < 6; i++)
                {
                    end_pos_default[i] = xmate_pos_mapping[i];
                }
            }
            // 限制机械臂沿着当前末端方向做插入运动
            // RPY转换为旋转矩阵
            Eigen::Matrix3d R = trans.eulerAnglesToRotationMatrix(end_pos_default[3], end_pos_default[4], end_pos_default[5]);
            // 末端Z方向；向量
            R.col(2).normalize();
            Eigen::Vector3d direction = R.col(2);
            // 先不对主端进行限制，进行映射

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

            // double distance = sqrt(pow(sigema7_pos_mapping[0], 2) + pow(sigema7_pos_mapping[1], 2) + pow(sigema7_pos_mapping[2], 2)); // 将手柄距离初始位置的距离作为插入量
            double distance = -sigema7_pos_mapping[2]; // 将手柄距离Z向运动作为插入量
            // 计算末端的目标位置
            xmate_pos_mapping[0] = end_pos_default[0] + distance * direction[0] * mapping_ratio * 5;
            xmate_pos_mapping[1] = end_pos_default[1] + distance * direction[1] * mapping_ratio * 5;
            xmate_pos_mapping[2] = end_pos_default[2] + distance * direction[2] * mapping_ratio * 5;
            xmate_pos_mapping[3] = end_pos_default[3] ;
            xmate_pos_mapping[4] = end_pos_default[4] ;
            xmate_pos_mapping[5] = end_pos_default[5] ;

            // 求出机械臂的末端位姿
            target_pose = trans.trans_PFtoO(xmate_pos_mapping);
        }
        // 发布从端机器人状态
        auto slaverMsg = ui_control_msg::msg::SlaverPosition();
        slaverMsg.xmate_position[0] = xmate_pos_mapping[0];
        slaverMsg.xmate_position[1] = xmate_pos_mapping[1];
        slaverMsg.xmate_position[2] = xmate_pos_mapping[2];
        slaverMsg.pf_position[0] = pf_pos_mapping[0];
        slaverMsg.pf_position[1] = pf_pos_mapping[1];
        slaverMsg.pf_position[2] = pf_pos_mapping[2];
        slaverMsg.mapping_mode = control_mode_falg;
        slaver_position_publisher->publish(slaverMsg);

    }

    void ui_control_callback(const ui_control_msg::msg::UiControl::SharedPtr msg)
    {   
        control_flag = msg->control_flag;
        // 初始化
        if(control_flag == 0) robot_initialize();
        if(control_flag == 1) automatic_navigation();
        if(control_flag == 2)
        {
            if(open_flag)
            {
                // 更新映射原点
                for(int i = 0; i < 6; i++)
                {
                    sigema7_pos_default[i] = sigema7_pos[i];
                    end_pos_default[i] = xmate_pos_mapping[i];
                } 
            }

            // 首次启动遥操作节点，初始化订阅者，订阅名为 "sigema7_getpos" 的话题
            if(!open_flag)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                // 读取此时机械臂的位置作为遥操作映射初始位置
                std::array<double, 6UL> current_posture;
                current_posture = robot.posture(rokae::CoordinateType::flangeInBase, ec);
                std::array<double, 6UL> current_posture_pf = trans.trans_OtoPF(current_posture);
                // 更新映射原点
                for(int i = 0; i < 6; i++)
                {
                    end_pos_default[i] = current_posture_pf[i];
                } 

                // 创建并启动新线程来运行rtloop函数
                open_rtControl_loop();
                tele_control_subscriber = this->create_subscription<sigema7_msg::msg::Sigema7Position>(
                    "sigema7_getpos", 10, std::bind(&TeleControlClient::tele_control_callback, this, std::placeholders::_1));
                open_flag = true;
            }
            // 初始化机械臂状态发布者
            // xmate_state_publisher = this->create_publisher<xmate_cr7_msg::msg::Cr7State>("xmate_state", 10);
            // xmate_pubThread= std::thread(&TeleControlClient::xmate_state_pub, this);
        }
        if(control_flag == 3)//关闭遥操作模式
        {
            // // 关闭实时模式
            // rtCon->stopLoop();
            // rtCon->stopMove();
        }
        if(control_flag == 4)//关闭当前节点
        {
            // 关闭实时模式
            robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
            robot.setPowerState(false, ec);
            robot.stopReceiveRobotState();
            // 停止 ROS 2 节点
            robot.useRciClient(true,ec);
            rclcpp::shutdown();
        } 
    }

    void ui_mode_callback(const ui_control_msg::msg::UiControl::SharedPtr msg)
    {   
        control_mode_falg = msg->teleoperation_mode;
        mode_change_flag = 1;
        std::cout << "control_mode_falg: " << control_mode_falg << std::endl;
    }

    /**
     * @brief 循环发布xmate机械臂的状态
     */
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
};
    
int main(int argc, char * argv[])
{
    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // // 将任务绑定到 CPU 核心 0
    // CPU_SET(0, &cpuset); 

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleControlClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
