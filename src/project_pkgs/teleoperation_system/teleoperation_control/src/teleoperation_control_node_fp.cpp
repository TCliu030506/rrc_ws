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


const double PI = 3.14159265359;
int flag_begin = 1;

using namespace std;
using namespace rokae;
using namespace rokae::RtSupportedFields;


class TeleControlClient : public rclcpp::Node
{
public:
    TeleControlClient() : Node("tele_control_client"), robot("192.168.2.160", "192.168.2.101")
    {
        // 初始化机器人：
        robot_initialize();

        // // 创建订阅者，订阅名为 "sigema7_getpos" 的话题
        tele_control_subscriber = this->create_subscription<sigema7_msg::msg::Sigema7Position>(
            "sigema7_getpos", 10, std::bind(&TeleControlClient::tele_control_callback, this, std::placeholders::_1));

        // 创建并启动新线程来运行 followPosition 函数
        followThread = std::thread(&TeleControlClient::followPosition, this, std::ref(robot), std::ref(target_jnt), speed_ratio);
  
    }
    ~TeleControlClient() {
        // 停止 followPosition 线程
        if (followThread.joinable()) {
            running = false;
            followThread.join();
        }
    };

    /**
     * @brief 点位跟随函数
     */
    void followPosition(rokae::xMateRobot &robot, std::array<double, 6> &target_jnt, double speed_ratio) 
    {
        std::thread updater;
        error_code ec;
        std::atomic_bool running = true; ///< running state flag

        FollowPosition<6> follow_pose;
        auto model = robot.model();
        follow_pose.init(robot, model);
        target_jnt = {M_PI/2, 0, -M_PI/2, 0, -M_PI/2, 0};
        follow_pose.start(target_jnt);
        std::cout<< "开始跟随"<< std::endl;

        // 启动一个线程来更新目标位置
        updater = std::thread([&]() 
        {
            while(running) {
                // 更新位置
                follow_pose.setScale(speed_ratio);
                follow_pose.update(target_jnt);
                std::this_thread::sleep_for(std::chrono::milliseconds (1));
            }
        });

        while(running);

        follow_pose.stop();
        updater.join();

    }

private:
    std::thread followThread;
    xMateRobot robot;
    std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> rtCon;
    std::error_code ec;
    std::string id;
    std::array<double, 6> target_jnt;
    bool running;
    double mapping_ratio = 1.5;
    double mapping_rad_ratio = 0.8;
    double speed_ratio = 20.0;
    double default_speed = 2000;
    double sigema7_pos_default[6] = {-0.00544945, -0.000884445, -0.00275382, 0.00194075, 0.000535195, -0.00308807};


    rclcpp::Subscription<sigema7_msg::msg::Sigema7Position>::SharedPtr tele_control_subscriber;

    double zone = 80;

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
        robot.setDefaultZone(30, ec);
        robot.adjustAcceleration(1.5,2, ec);


        // 运动至机器人初始位置
        MoveAbsJCommand  startPosjoint({M_PI/2, 0, -M_PI/2, 0, -M_PI/2, 0});
        robot.moveAppend({startPosjoint}, id, ec);
        robot.moveStart(ec);
        waitRobot(robot, running);

        // 切换到实施控制模式
        robot.setRtNetworkTolerance(20, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            std::cerr << "获取实时运动控制器失败" << std::endl;
            return;
        }

        // 打印出测试字符
        std::cout << "机器人初始化完成" << std::endl;

        // 测试位置跟随——————
        // example_followPosition(robot);
    

    }

    // 函数用于计算绕原点旋转后的新位姿
    std::array<double, 6> rotatePose(const std::array<double, 6>& pose) {
        // 提取原始位姿
        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double original_rx = pose[3];
        double original_ry = pose[4];
        double original_rz = pose[5];
        
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
       std::cout << "进入回调函数: " << std::endl;

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
        // 根据机械臂实际情况进行旋转
        // 绕Z轴旋转270度
        sigema7_pos_mapping = rotatePose(sigema7_pos_mapping);
        // 计算机械臂的目标位置
        std::array<double, 6> pos_mapping = {0, 0, 0, 0, 0, 0};
        pos_mapping[0] = 0.150131 + sigema7_pos_mapping[0] * mapping_ratio;
        pos_mapping[1] = 0.360185 + sigema7_pos_mapping[1] * mapping_ratio;
        pos_mapping[2] = 0.659604 + sigema7_pos_mapping[2] * mapping_ratio;
        pos_mapping[3] = M_PI + sigema7_pos_mapping[3] * mapping_rad_ratio;
        pos_mapping[4] = 0 + sigema7_pos_mapping[4] * mapping_rad_ratio;
        pos_mapping[5] = -M_PI/2 + sigema7_pos_mapping[5] * mapping_rad_ratio;

        // 手柄速度映射到机械臂速度
        std::array<double, 3> vel_mapping = {0, 0, 0};
        vel_mapping[0] = sigema7_vel_linear[0] * mapping_ratio * 3;
        vel_mapping[1] = sigema7_vel_linear[1] * mapping_ratio * 3;
        vel_mapping[2] = sigema7_vel_linear[2] * mapping_ratio * 3;

        // 调整末端执行器的速度
        double tool_speed = std::sqrt(std::pow(vel_mapping[0], 2) + std::pow(vel_mapping[1], 2) + std::pow(vel_mapping[2], 2)) * 6;
        tool_speed = tool_speed*1000; // 单位为mm/s
        // if (tool_speed > 800)tool_speed = 800;
        speed_ratio = tool_speed/default_speed * 2;



        // CoordinateType ct = CoordinateType::flangeInBase;
        // CartesianPosition startPosition = robot.posture(ct,ec);
        // CartesianPosition targetPosition({pos_mapping[0], pos_mapping[1], pos_mapping[2], pos_mapping[3], pos_mapping[4], pos_mapping[5]});

        // std::cout << "运动速度系数： " << std::endl;
        // std::cout << speed_ratio << std::endl;
        // std::cout << "当前位置： " << std::endl;
        // std::cout << startPosition.trans[0] << ',' << startPosition.trans[1] << ',' << startPosition.trans[2] << ',' << startPosition.rpy[0] << ',' << startPosition.rpy[1] << ',' << startPosition.rpy[2] << std::endl;
        // std::cout << "目标位置： " << std::endl;
        // std::cout << targetPosition.trans[0] << ',' << targetPosition.trans[1] << ',' << targetPosition.trans[2] << ',' << targetPosition.rpy[0] << ',' << targetPosition.rpy[1] << ',' << targetPosition.rpy[2] << std::endl;


        const std::array<double, 6UL> pos_mapping_arr= {pos_mapping[0], pos_mapping[1], pos_mapping[2], pos_mapping[3], pos_mapping[4], pos_mapping[5]};
        std::array<double, 16UL> targetPositionMatrix;
        Utils::postureToTransArray(pos_mapping_arr,targetPositionMatrix);

        std::array<double, 6> init_jointPos = robot.jointPos(ec);
        std::array<double, 6> target_jointPos = {};
        auto model = robot.model();
        model.getJointPos(targetPositionMatrix,0,init_jointPos,target_jointPos);
        // // 打印出关节角度
        // std::cout << "法兰笛卡尔空间位姿： " << std::endl;
        // for (size_t i = 0; i < targetPositionMatrix.size(); ++i)
        // {
        //     std::cout << targetPositionMatrix[i] << ',';
        // }
        std::cout << std::endl;
        std::cout << "当前关节位置： " << std::endl;
        for (size_t i = 0; i < init_jointPos.size(); ++i)
        {
            std::cout << init_jointPos[i]/M_PI*180 << ',';
        }
        std::cout << std::endl;
        std::cout << "目标关节位置： " << std::endl;
        for (size_t i = 0; i < target_jointPos.size(); ++i)
        {
            std::cout << target_jointPos[i]/M_PI*180 << ',';
        }
        std::cout << std::endl;

        // 点位跟随目标点更改
        target_jnt = target_jointPos;

        
        std::cout << "完成发送 " << std::endl;
        std::cout << "———————————————————————— " << std::endl;

    }

    // /**
    //  * @brief 跟随关节点位示例
    //  */
    // void example_followPosition(rokae::xMateRobot &robot) {
    // std::thread updater;
    // error_code ec;
    // std::atomic_bool running = true; ///< running state flag
    // try {
    //     std::cout << "进入跟随关节点位函数" << std::endl;
    //     rtCon = robot.getRtMotionController().lock();
    //     std::array<double, 6> it = { M_PI/2, 0, -M_PI/2, 0, -M_PI/2, 0};

    //     auto model = robot.model();
    //     FollowPosition<6> follow_pose;
    //     follow_pose.init(robot, model);
    //     std::cout<< "开始跟随"<< std::endl;

    //     follow_pose.start(it);

    //     // 启动一个线程来更新目标位置
    //     updater = std::thread([&]() {
    //     follow_pose.setScale(2);
    //     double setscale = 1;
    //     double scale_change = 0.002*8;
    //     while(running) {
    //         // 模拟每20ms更新一次位置
    //         while (running) {
    //             if(setscale>=1.8)scale_change = -0.002*8;
    //             if(setscale<=1)scale_change = 0.002*8;
    //             setscale += scale_change;
    //             follow_pose.setScale(setscale);
    //             // std::cout << "正向，set scale: " << setscale << std::endl;
    //             it[4] += M_PI/100;
    //             follow_pose.update(it);
    //             std::this_thread::sleep_for(std::chrono::milliseconds (20));
    //             if (it[4] >= M_PI/2) {
    //                 break;
    //         }
    //         }
    //         while (running) {
    //             // follow_pose.setScale(setscale-=0.1);
    //             // std::cout << "逆向，set scale: " << setscale << std::endl;
    //             if(setscale>=1.8)scale_change = -0.002*8;
    //             if(setscale<=1)scale_change = 0.002*8;
    //             setscale += scale_change;
    //             follow_pose.setScale(setscale);
    //             it[4] -= M_PI/100;
    //             follow_pose.update(it);
    //             std::this_thread::sleep_for(std::chrono::milliseconds (20));
    //             if (it[4] <= -M_PI/2) {
    //                 break;
    //         }
    //         }
    //     }
    //     });

    //     // 等待用户输入
    //     std::thread consoleInput([&running]{
    //     while(getchar() != 'q'); // press 'q' to stop
    //     running = false;
    //     while(getchar() != 'p'); // press 'q' to stop
    //     running = true;
    //     });
        
    //     consoleInput.detach();
    //     while(running);

    //     follow_pose.stop();
    //     updater.join();
    // } 
    // catch (const std::exception &e) {
    //     std::cerr << "Exception: " << e.what() << std::endl;
    //     if(updater.joinable()) {
    //     running = false;
    //     updater.join();
    //     }
    // }
    // }
    
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleControlClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
