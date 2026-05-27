/**
 * @file get_keypad_state.cpp
 * @brief 读取末端按键状态
 *
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "x_core_sdk/robot.h"
#include "print_helper.hpp"

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;
  try {
    std::string ip = "192.168.2.160";
    std::error_code ec;
    // 本示例用到了实时状态数据，所以需要设置本机地址
    rokae::xMateRobot robot(ip,"192.168.2.101");

    KeyPadState state = robot.getKeypadState(ec);
    std::cout << "当前末端按键的状态,key1: " << state.key1_state << ",key2:"<<state.key2_state
              << ",key3:" << state.key3_state << ",key4:" << state.key4_state << ",key5:" << state.key5_state
              << ",key6:" << state.key6_state << ",key7:" << state.key7_state << std::endl;


    // 设置要接收数据。其中keypads是本示例程序会用到的
    robot.startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::keypads });

    std::array<bool, 7> keypad{};
    robot.getStateData(RtSupportedFields::keypads, keypad);

    // 运行50次
    int count = 50;

    std::thread readKeyPad([&] {
      while (count--) {
        // 每隔1秒读取一次末端按键状态
        robot.updateRobotState(std::chrono::milliseconds(1));
        robot.getStateData(RtSupportedFields::keypads, keypad);
        std::cout << "当前末端按键的状态,key1: " << keypad[0] << ",key2:" << keypad[1]
                  << ",key3:" << keypad[2] << ",key4:" << keypad[3] << ",key5:" << keypad[4]
                  << ",key6:" << keypad[5] << ",key7:" << keypad[6] << std::endl;
      }
    });

    readKeyPad.join();
  }
  catch (const std::exception& e) {
    std::cout << e.what();
  }
  return 0;
}