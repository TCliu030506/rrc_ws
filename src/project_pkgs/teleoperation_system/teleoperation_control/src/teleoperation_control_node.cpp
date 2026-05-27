#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include "x_core_sdk/robot.h"
#include "sigema7_msg/msg/sigema7_position.hpp"

const double PI = 3.14159265359;
int flag_begin = 1;
int count = 0;

using namespace std;
using namespace rokae;


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
    std::error_code ec;
    std::string id;
    bool running;
    double mapping_ratio = 2;
    double mapping_rad_ratio = 0.9;

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
        // 初始化机器人：设置操作模式、电源状态和运动重置
        // xMateRobot robot("192.168.2.160", "192.168.2.101");
        double scale = 1;
        double acc;
        double jerk;
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setPowerState(true, ec);
        robot.moveReset(ec);
        robot.adjustSpeedOnline(scale, ec);

        // 设置默认速度
        robot.setDefaultSpeed(200, ec);
        // 设置默认转弯区
        robot.setDefaultZone(5, ec);
        // 获取机器人的加速度
        robot.getAcceleration(acc, jerk, ec);

        // 检查是否有错误发生
        if (ec) {
            std::cerr << "获取加速度时发生错误: " << ec.message() << std::endl;
            return;
        }

        // 打印加速度信息
        std::cout << "机器人的加速度为: ";
        std::cout << acc << std::endl;
        std::cout << "机器人的加加速度为: ";
        std::cout << jerk << std::endl;

        robot.adjustAcceleration(1.5,2, ec);


        // // 机器人初始位置
        MoveAbsJCommand  p1({0, M_PI/6, M_PI/3, 0, M_PI_2, 0});
        MoveAbsJCommand  startPosjoint({M_PI/2, 0, -M_PI/2, 0, -M_PI/2, 0});
        // 创建MoveL指令
        CartesianPosition targetPosition({0.15, 0.37, 0.66, M_PI, 0, -M_PI/2});
        MoveLCommand moveL(targetPosition);

        // // 运动
        // robot.executeCommand({startPosjoint}, ec);
        // waitRobot(robot, running);

        // // 先运动到拖拽位姿
        // robot.executeCommand({p1}, ec);
        // waitRobot(robot, running);

        // // 执行MoveL运动
        // robot.executeCommand({moveL}, ec);
        // // waitRobot(robot, running);

        // // 运动回拖拽位姿
        // robot.executeCommand({p1}, ec);
        // // waitRobot(robot, running);

        // robot.moveAppend({moveL}, id, ec);
        // robot.moveAppend({p1}, id, ec);

        // robot.moveAppend({moveL}, id, ec);
        // robot.moveAppend({p1}, id, ec);

        robot.moveAppend({startPosjoint}, id, ec);
        robot.moveStart(ec);
        // robot.moveAppend({moveL}, id, ec);
        // robot.executeCommand({startPosjoint}, ec);
        // robot.executeCommand({moveL}, ec);
        // robot.moveReset(ec);

        
        waitRobot(robot, running);

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
        tool_speed = tool_speed*1000*1000; // 单位为mm/s

        // 控制机械臂运动
        if (count == 2) {

            CartesianPosition targetPosition({pos_mapping[0], pos_mapping[1], pos_mapping[2], pos_mapping[3], pos_mapping[4], pos_mapping[5]});
            std::cout << "目标位置： " << std::endl;
            std::cout << pos_mapping[0] << ',' << pos_mapping[1] << ',' << pos_mapping[2] << ',' << pos_mapping[3] << ',' << pos_mapping[4] << ',' << pos_mapping[5] << std::endl;
            std::cout << "运动速度： " << std::endl;
            std::cout << tool_speed << std::endl;
            MoveLCommand movel({targetPosition}, tool_speed, zone);
            // MoveLCommand p_test({0.2,0.5,0.6,-90/180*M_PI,90/180*M_PI,0/180*M_PI}, 100, zone);
            robot.moveReset(ec);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // robot.moveAppend({movel}, id, ec);
            // robot.moveStart(ec);
            robot.executeCommand({movel}, ec);
            // 检查是否有错误发生
            if (ec) {
                std::cerr << "运行发生错误: " << ec.message() << std::endl;
                return;
            }

            count = 0;
            // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            std::cout << "完成发送 " << std::endl;
            std::cout << "———————————————————————— " << std::endl;
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
