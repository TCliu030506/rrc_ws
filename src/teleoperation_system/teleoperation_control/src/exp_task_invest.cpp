#define XMATEMODEL_LIB_SUPPORTED
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include "x_core_sdk/robot.h"
#include "x_core_sdk/utility.h"
#include "x_core_sdk/model.h"
#include "pf_control/pf_control.h" // 添加pf_control头文件

using namespace std;
using namespace rokae;

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
// 将 std::array<double, 16> 转为 Eigen::Matrix4d
inline Eigen::Matrix4d arrayToMatrix4d(const std::array<double, 16>& arr) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 16; ++i)
        mat(i / 4, i % 4) = arr[i];
    return mat;
}

// 将 Eigen::Matrix4d 转为 std::array<double, 16>
inline std::array<double, 16> matrix4dToArray(const Eigen::Matrix4d& mat) {
    std::array<double, 16> arr;
    for (int i = 0; i < 16; ++i)
        arr[i] = mat(i / 4, i % 4);
    return arr;
}

// 4x4矩阵乘法
inline void matrixMultiply(const std::array<double, 16>& a, const std::array<double, 16>& b, std::array<double, 16>& result) {
    Eigen::Matrix4d mat_a = arrayToMatrix4d(a);
    Eigen::Matrix4d mat_b = arrayToMatrix4d(b);
    Eigen::Matrix4d mat_result = mat_a * mat_b;
    result = matrix4dToArray(mat_result);
}

std::array<double, 6> trans_p_to_world(std::array<double, 6>&  base_in_world, std::array<double, 6>& current_pos)
{
    // 将当前位姿从基坐标系转换到世界坐标系
    // 读取当基坐标系相对于世界坐标系的位姿[x,y,z,rx,ry,rz]
    // 计算转换矩阵[4x4]
    std::array<double, 16> base_in_world_matrix;
    Utils::postureToTransArray(base_in_world, base_in_world_matrix);
    // 当前位姿转换为矩阵形式[4x4]
    std::array<double, 16> current_pos_matrix;
    Utils::postureToTransArray(current_pos, current_pos_matrix);
    // 转换当前位置到世界坐标系[4x4]
    // 计算当前位置在世界坐标系下的变换矩阵
    std::array<double, 16> current_pos_in_world_matrix;
    // 末端在世界坐标系下的变换 = 基坐标系在世界坐标系下的变换 * 末端在基坐标系下的变换
    matrixMultiply(base_in_world_matrix, current_pos_matrix, current_pos_in_world_matrix);
    // 转换回6维位姿[x,y,z,rx,ry,rz]
    Utils::transArrayToPosture(current_pos_in_world_matrix, current_pos);

    return current_pos;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // 1. 读取CSV文件
    std::string filename = "record.csv";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return 1;
    }

    // 新增：两个vector分别存储sigema7_pos和pf_pos
    std::vector<std::array<double, 6>> sigema7_pos_list;
    std::vector<std::array<double, 3>> pf_pos_list;

    std::string line;
    bool running;
    // 跳过表头
    std::getline(file, line);

    // 4. 初始化pf_control和机器人
    pf_control pf_controller("/dev/ttyUSB0"); // 参考system_control_node_ui

    std::error_code ec;
    xMateRobot robot("192.168.2.160", "192.168.2.101");
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    robot.moveReset(ec);
    robot.adjustSpeedOnline(1.0, ec);
    robot.setDefaultSpeed(50, ec);
    robot.setDefaultZone(10, ec);
    robot.adjustAcceleration(0.5,0.5, ec);

    


    // 读取当基坐标系相对于世界坐标系的位姿[x,y,z,rx,ry,rz]
    std::array<double, 6>  base_in_world = robot.baseFrame(ec);

    // 2. 读取所有点位并分类存储
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::array<double, 6> sigema7_pos;
        std::array<double, 3> pf_pos;
        int idx = 0;
        // 以逗号分割
        while (std::getline(iss, token, ',') && idx < 10) {
            if (idx >= 1 && idx <= 6) { // x,y,z,rx,ry,rz
                sigema7_pos[idx-1] = std::stod(token);
            }
            if (idx >= 7 && idx <= 9) { // theta1,theta2,theta3
                pf_pos[idx-7] = std::stod(token);
            }
            idx++;
        }
        if (idx >= 10) {
            // 坐标系转换
            std::array<double, 6> sigema7_pos_world = trans_p_to_world(base_in_world, sigema7_pos);
            sigema7_pos_list.push_back(sigema7_pos_world);
            pf_pos_list.push_back(pf_pos);
        }
    }
    file.close();

    // 3. 打印所有点位（已转换坐标系）
    std::cout << "读取到的sigema7_pos点位(世界坐标系)如下：" << std::endl;
    int seq = 1;
    for (const auto& pt : sigema7_pos_list) {
        std::cout << seq++ << ": ";
        for (int i = 0; i < 6; ++i) {
            std::cout << pt[i] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "读取到的pf_pos点位如下：" << std::endl;
    seq = 1;
    for (const auto& pt : pf_pos_list) {
        std::cout << seq++ << ": ";
        for (int i = 0; i < 3; ++i) {
            std::cout << pt[i] << " ";
        }
        std::cout << std::endl;
    }

    
    // 5. 依次执行点位，包括机械臂和pf两部分
    seq = 1;
    for (size_t i = 0; i < sigema7_pos_list.size(); ++i) {
        std::cout << "是否执行第" << seq << "个点位？输入1执行，其他跳过: ";
        int user_input = 0;
        std::cin >> user_input;
        if (user_input == 1) {
            std::cout << "正在执行第" << seq << "个点位..." << std::endl;
            // 先执行机械臂
            MoveJCommand cmd({
                sigema7_pos_list[i][0],
                sigema7_pos_list[i][1],
                sigema7_pos_list[i][2],
                sigema7_pos_list[i][3],
                sigema7_pos_list[i][4],
                sigema7_pos_list[i][5]
            });
            robot.executeCommand({cmd}, ec);
            // 等待机械臂运动结束
            waitRobot(robot, running);
            
            // 再执行pf
            pf_controller.pf_movep(
                pf_pos_list[i][0],
                pf_pos_list[i][1],
                pf_pos_list[i][2],
                3.0 // 可根据实际情况调整
            );
            // 先用打印代替pf_movep
            std::cout << "pf_movep: " << pf_pos_list[i][0] << ", " << pf_pos_list[i][1] << ", " << pf_pos_list[i][2] << std::endl;
            std::cout << "第" << seq << "个点位执行完成。" << std::endl;
        }
        ++seq;
    }

    // 6. 关闭机器人
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setPowerState(false, ec);
    robot.stopReceiveRobotState();

    rclcpp::shutdown();
    return 0;
}
