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
int count = 0;

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
        

    }
    ~TeleControlClient() {};

private:
    xMateRobot robot;
    std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> rtCon;
    std::error_code ec;
    std::string id;
    bool running;
    double mapping_ratio = 2;
    double mapping_rad_ratio = 0.9;
    double speed_ratio = 1;

    rclcpp::Subscription<sigema7_msg::msg::Sigema7Position>::SharedPtr tele_control_subscriber;

    double zone = 80;
    int count = 1;

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
        robot.setDefaultSpeed(400, ec);
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

    

        // // 测试实时；运动控制指令————
        // robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});        
        // double time = 0;

        // std::array<double, 6> jntPos{};
        // CoordinateType ct = CoordinateType::flangeInBase;   

        // std::function<CartesianPosition()> callback = [&, rtCon](){
        // time += 0.01;
        // double delta_angle = 0.01* time;
        // CartesianPosition cmd ({jntPos[0] , jntPos[1] ,
        //                         jntPos[2]-delta_angle ,
        //                         jntPos[3] , 
        //                         jntPos[4] ,
        //                         jntPos[5] });

        // std::cout<<time<<std::endl;
    
        // if(time > 10) {
        //     cmd.setFinished(); // 60秒后结束
        // }
        // return cmd;
        // };

        // // 设置回调函数
        // rtCon->setControlLoop(callback);
        // // 更新起始角度为当前角度
        // jntPos = robot.posture(ct,ec);
        // // 开始轴空间位置控制
        // rtCon->startMove(RtControllerMode::cartesianPosition);
        // // 阻塞loop
        // rtCon->startLoop(true);

    }


    void tele_control_callback(const sigema7_msg::msg::Sigema7Position::SharedPtr msg)
    {
        std::cout << "进入回调函数: " << std::endl;

        // 获取手柄的输入位置、速度和力
        std::vector<double> sigma7_pos(msg->sigema7_pos.size(), 0);
        std::vector<double> sigema7_vel_linear(msg->sigema7_vel_linear.size(), 0);
        std::vector<double> sigema7_force(msg->sigema7_force.size(), 0);

        for (size_t i = 0; i < msg->sigema7_pos.size(); ++i)
        {
            sigma7_pos[i] = msg->sigema7_pos[i];
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
        std::vector<double> pos_mapping(msg->sigema7_pos.size(), 0);
        pos_mapping[0] = 0.150 + (sigma7_pos[0] + 0.006798) * mapping_ratio;
        pos_mapping[1] = 0.360 + (sigma7_pos[1] + 0.000586) * mapping_ratio;
        pos_mapping[2] = 0.660 + (sigma7_pos[2] + 0.003852) * mapping_ratio;
        pos_mapping[3] = M_PI + (sigma7_pos[3] - 0.00278983) * mapping_rad_ratio;
        pos_mapping[4] = 0 + (sigma7_pos[4] + 0.00896451) * mapping_rad_ratio;
        pos_mapping[5] = -M_PI/2 + (sigma7_pos[5] - 0.0069999) * mapping_rad_ratio;

        std::vector<double> vel_mapping(msg->sigema7_vel_linear.size(), 0);
        vel_mapping[0] = sigema7_vel_linear[0] * mapping_ratio * 3;
        vel_mapping[1] = sigema7_vel_linear[1] * mapping_ratio * 3;
        vel_mapping[2] = sigema7_vel_linear[2] * mapping_ratio * 3;

        // 调整末端执行器的速度
        double tool_speed = std::sqrt(std::pow(vel_mapping[0], 2) + std::pow(vel_mapping[1], 2) + std::pow(vel_mapping[2], 2)) * 6;
        // if (tool_speed < 0.15)tool_speed = 0.15;
        tool_speed = tool_speed*1000; // 单位为mm/s
        if (tool_speed > 800)tool_speed = 800;
        speed_ratio = tool_speed/800;

        // 控制机械臂运动
        if (count == 1) {

            CoordinateType ct = CoordinateType::flangeInBase;
            CartesianPosition startPosition = robot.posture(ct,ec);
            CartesianPosition targetPosition({pos_mapping[0], pos_mapping[1], pos_mapping[2], pos_mapping[3], pos_mapping[4], pos_mapping[5]});

            std::cout << "运动速度系数： " << std::endl;
            std::cout << speed_ratio << std::endl;
            std::cout << "当前位置： " << std::endl;
            std::cout << startPosition.trans[0] << ',' << startPosition.trans[1] << ',' << startPosition.trans[2] << ',' << startPosition.rpy[0] << ',' << startPosition.rpy[1] << ',' << startPosition.rpy[2] << std::endl;
            std::cout << "目标位置： " << std::endl;
            std::cout << targetPosition.trans[0] << ',' << targetPosition.trans[1] << ',' << targetPosition.trans[2] << ',' << targetPosition.rpy[0] << ',' << targetPosition.rpy[1] << ',' << targetPosition.rpy[2] << std::endl;

            // CartesianPosition p1 = startPosition;
            // p1.trans[0] += 0.05;
            const std::array<double, 6UL> pos_mapping_arr= {pos_mapping[0], pos_mapping[1], pos_mapping[2], pos_mapping[3], pos_mapping[4], pos_mapping[5]};
            std::array<double, 16UL> targetPositionMatrix;
            Utils::postureToTransArray(pos_mapping_arr,targetPositionMatrix);
    
            std::array<double, 6> init_jointPos = robot.jointPos(ec);
            std::array<double, 6> target_jointPos = {};
            auto model = robot.model();
            model.getJointPos(targetPositionMatrix,0,init_jointPos,target_jointPos);
            // 打印出关节角度
            std::cout << "法兰笛卡尔空间位姿： " << std::endl;
            for (size_t i = 0; i < targetPositionMatrix.size(); ++i)
            {
                std::cout << targetPositionMatrix[i] << ',';
            }
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

            
            // 实时运动控制——————————————————
            rtCon->MoveJ(speed_ratio, init_jointPos, target_jointPos);
            
            std::cout << "完成发送 " << std::endl;
            std::cout << "———————————————————————— " << std::endl;

            count = 0;
        }

        count++;
    }
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleControlClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
