/**
 * @file move_example.cpp
 * @brief 非实时运动指令. 根据机型和坐标系的不同, 各示例中的点位不一定可达, 仅供接口使用方法的参考
 *
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include <cmath>
#include "x_core_sdk/robot.h"
#include "x_core_sdk/utility.h"
#include "print_helper.hpp"

using namespace std;
using namespace rokae;
std::ostream &os = std::cout; ///< print to console

namespace Predefines {
 // ******   拖拽位姿   ******
 const std::vector<double> ErDragPosture = {0, M_PI/6, M_PI/3, 0, M_PI_2, 0}; ///< xMateEr3, xMateEr7
 const std::vector<double> ErProDragPosture = {0, M_PI/6, 0, M_PI/3, 0, M_PI_2, 0}; ///< xMateEr3 Pro, xMateEr7 Pro
 const std::vector<double> CrDragPosture {0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0}; ///< xMateCR
 const std::vector<double> Cr5DragPostre = { 0, M_PI / 6, -M_PI_2, -M_PI / 3, 0}; ///< CR5轴构型

 Toolset defaultToolset; ///< 默认工具工件
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

/**
 * @brief 演示如何控制机器人进行MoveL运动
 */
void moveLExample(xMateRobot &robot) {
  error_code ec;
  bool running;

  // 设置默认工具工件
  robot.setToolset(Predefines::defaultToolset, ec);
  // 设置默认速度
  robot.setDefaultSpeed(500, ec);
  // 设置默认转弯区
  robot.setDefaultZone(5, ec);

  // 定义目标笛卡尔位置
  CartesianPosition targetPosition({0.786, 0, 0.431, M_PI, 0.6, M_PI});

  // 创建MoveL指令
  MoveLCommand moveL(targetPosition);

  // 先运动到拖拽位姿
  print(std::cout, "运动到拖拽位姿");
  robot.executeCommand({MoveAbsJCommand(Predefines::ErDragPosture)}, ec);
  waitRobot(robot, running);

  // 执行MoveL运动
  print(std::cout, "开始MoveL运动");
  robot.executeCommand({moveL}, ec);
  waitRobot(robot, running);

  // 运动回拖拽位姿
  print(std::cout, "运动回拖拽位姿");
  robot.executeCommand({MoveAbsJCommand(Predefines::ErDragPosture)}, ec);
  waitRobot(robot, running);
}

/**
 * @brief main program
 */
int main() {
  try {
    using namespace rokae;

    // *** 1. 连接机器人 ***
    std::error_code ec;
    xMateRobot robot("192.168.2.160", "192.168.2.101");

    // *** 2. 切换到自动模式并上电 ***
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // *** 3. 执行MoveL运动示例 ***
    moveLExample(robot);

    // *** 4. 关闭电源并断开连接 ***
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}
