#include "xmate_cr7_script/cr7_script.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cr7_script::Cr7ScriptClient>();
    std::string result;

    // // // 示例1：movej
    // std::vector<double> joint_angles1 = {M_PI/2, 0, M_PI/2, 0, M_PI/2, 0};
    // node->movej(joint_angles1, 100, 30, result);
    // RCLCPP_INFO(node->get_logger(), "movej result: %s", result.c_str());
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // std::vector<double> joint_angles2 = {M_PI/2, 0, M_PI/2, M_PI/6, M_PI/2, 0};
    // node->movej(joint_angles2, 100, 30, result);
    // RCLCPP_INFO(node->get_logger(), "movej result: %s", result.c_str());

    // // 示例2：movep
    // std::array<double, 6> pose1 = {0.200, -0.356, 0.724, -150.0*M_PI/180.0, 0.0, -85.0*M_PI/180.0};
    // node->movep(pose1, 100, 30, result);
    // RCLCPP_INFO(node->get_logger(), "movep result: %s", result.c_str());

    // std::array<double, 6> pose2 = {0.150, -0.360, 0.660, -M_PI, 0.0, -M_PI/2};
    // node->movep(pose2, 100, 30, result);
    // RCLCPP_INFO(node->get_logger(), "movep result: %s", result.c_str());

    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // // 示例3：stop
    // node->stop(result);
    // RCLCPP_INFO(node->get_logger(), "stop result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // // 示例4：movel
    // std::array<double, 6> posel1 = {0.200, -0.356, 0.724, -150.0*M_PI/180.0, 0.0, -85.0*M_PI/180.0};
    // node->movel(posel1, 100, 30, result);
    // RCLCPP_INFO(node->get_logger(), "movel result: %s", result.c_str());

    // std::array<double, 6> posel2 = {0.150, -0.360, 0.660, -M_PI, 0.0, -M_PI/2};
    // node->movel(posel2, 100, 30, result);
    // RCLCPP_INFO(node->get_logger(), "movel result: %s", result.c_str());

    // // 示例5：follow_pos_start
    // node->follow_pos_start(result);
    // RCLCPP_INFO(node->get_logger(), "follow_pos_start result: %s", result.c_str());

    // // 每隔 10ms 更新点位
    // std::array<double, 6> joint_angles_update = {M_PI/2, 0, M_PI/2, 0, M_PI/2, 0};
    // for (int i = 0; i < 1000; ++i) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     joint_angles_update[4] += M_PI*6/10000;
    //     node->follow_pos_update(joint_angles_update, 400, result);
    // }
    // for (int i = 0; i < 1000; ++i) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     joint_angles_update[4] -= M_PI*6/10000;
    //     node->follow_pos_update(joint_angles_update, 400, result);
    // }
    // node->follow_pos_stop(result);
    // RCLCPP_INFO(node->get_logger(), "follow_pos_stop result: %s", result.c_str());

    // // 示例6：open_rtControl_loop
    // node->open_rtControl_loop("jointPosition", result);
    // RCLCPP_INFO(node->get_logger(), "open_rtControl_loop result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // // 每隔 1ms 更新点位
    // std::array<double, 6> joint_angles_update = {M_PI/2, 0, M_PI/2, 0, M_PI/2, 0};
    // for (int i = 0; i < 10000; ++i) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     joint_angles_update[4] += M_PI*6/100000;
    //     node->rtControl_jointpos_update(joint_angles_update, result);
    // }
    // for (int i = 0; i < 10000; ++i) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     joint_angles_update[4] -= M_PI*6/100000;
    //     node->rtControl_jointpos_update(joint_angles_update, result);
    // }

    // // 示例7：open_rtControl_loop（cartesianPosition）
    // node->open_rtControl_loop("cartesianPosition", result);
    // RCLCPP_INFO(node->get_logger(), "open_rtControl_loop (cartesianPosition) result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // std::array<double, 6> posel_update = {0.150, -0.360, 0.660, -M_PI, 0.0, -M_PI/2};
    // for (int i = 0; i < 10000; ++i) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     posel_update[1] += 0.2/10000;
    //     node->rtControl_cartesianpos_update(posel_update, result);
    // }
    // for (int i = 0; i < 10000; ++i) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     posel_update[1] -= 0.2/10000;
    //     node->rtControl_cartesianpos_update(posel_update, result);
    // }

    // // 示例8：readp
    // node->readp(result);
    // RCLCPP_INFO(node->get_logger(), "readp result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例9：readj
    // node->readj(result);
    // RCLCPP_INFO(node->get_logger(), "readj result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例10：get_vel
    // node->get_vel(result);
    // RCLCPP_INFO(node->get_logger(), "get_vel result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例11：get_joint_vel
    // node->get_joint_vel(result);
    // RCLCPP_INFO(node->get_logger(), "get_joint_vel result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例12：calibrateForceSensor
    // node->calibrateForceSensor(true, 0, result);
    // RCLCPP_INFO(node->get_logger(), "calibrateForceSensor result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例13：getEndTorque
    // node->getEndTorque("world", result);
    // RCLCPP_INFO(node->get_logger(), "getEndTorque result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例14：get_joint_torque
    // node->get_joint_torque(result);
    // RCLCPP_INFO(node->get_logger(), "get_joint_torque result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例15：get_acc
    // node->get_acc(result);
    // RCLCPP_INFO(node->get_logger(), "get_acc result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例16：get_jerk
    // node->get_jerk(result);
    // RCLCPP_INFO(node->get_logger(), "get_jerk result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例17：inv_kinematics
    // std::array<double, 6> inv_pos = {0.15, -0.35, 0.66, M_PI, 0.0, -M_PI/2};
    // node->inv_kinematics(inv_pos, result);
    // RCLCPP_INFO(node->get_logger(), "inv_kinematics result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例18：forward_kinematics
    // std::array<double, 6> joint_angles1 = {M_PI/2, 0, M_PI/2, 0, M_PI/2, 0};
    // node->forward_kinematics(joint_angles1, result);
    // RCLCPP_INFO(node->get_logger(), "forward_kinematics result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 示例19：开启阻抗控制模式
    // std::array<double, 6> pose2 = {0.150, -0.360, 0.660, -M_PI, 0.0, -M_PI/2};
    // node->movep(pose2, 100, 30, result); // 运动至初始位置
    // node->fcInit("world", result);
    // node->setControlType(1, result); // 笛卡尔阻抗
    // node->setCartesianStiffness({1500, 0, 1500, 100, 100, 100}, result);
    // node->setCartesianNullspaceStiffness(4.0, result);
    // node->fcStart(result);
    // RCLCPP_INFO(node->get_logger(), "阻抗模式：fcStart result: %s", result.c_str());
    // std::this_thread::sleep_for(std::chrono::milliseconds(100000)); 
    // node->fcStop(result);
    // RCLCPP_INFO(node->get_logger(), "阻抗模式：fcStop result: %s", result.c_str());

    // 示例20：开始恒力运动模式
    std::array<double, 6> pose2 = {0.150, -0.360, 0.660, -M_PI, 0.0, -M_PI/2};
    node->movep(pose2, 100, 30, result); // 运动至初始位置

    node->fcInit("world", result);
    RCLCPP_INFO(node->get_logger(), "fcInit result: %s", result.c_str());
    node->setControlType(1, result); // 笛卡尔阻抗
    RCLCPP_INFO(node->get_logger(), "setControlType result: %s", result.c_str());
    node->setCartesianStiffness({500, 500, 0, 100, 100, 100}, result);
    RCLCPP_INFO(node->get_logger(), "setCartesianStiffness result: %s", result.c_str());
    node->setCartesianNullspaceStiffness(2.0, result);
    RCLCPP_INFO(node->get_logger(), "setCartesianNullspaceStiffness result: %s", result.c_str());
    node->setLoad(0,{0,0,0},{0,0,0},result);
    RCLCPP_INFO(node->get_logger(), "setLoad result: %s", result.c_str());
    node->fcStart(result);
    RCLCPP_INFO(node->get_logger(), "fcStart result: %s", result.c_str());
    node->setCartesianDesiredForce({0,0,-15,0,0,0},result);
    RCLCPP_INFO(node->get_logger(), "setCartesianDesiredForce result: %s", result.c_str());
    node->setForceCondition({-100,100,-100,100,-100,10},true,20.0,result);
    RCLCPP_INFO(node->get_logger(), "setForceCondition result: %s", result.c_str());
    node->waitCondition(result);
    RCLCPP_INFO(node->get_logger(), "waitCondition result: %s", result.c_str());

    std::array<double, 6> pose3 = pose2;
    pose3[1] = pose3[1]+0.20;
    node->movep(pose3, 10, 30, result); // 运动至目标位置
    RCLCPP_INFO(node->get_logger(), "movep result: %s", result.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(30000));

    node->fcStop(result);
    RCLCPP_INFO(node->get_logger(), "fcStop result: %s", result.c_str());


    rclcpp::shutdown();
    return 0;
}