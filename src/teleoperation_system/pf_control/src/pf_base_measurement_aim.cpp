#include <iostream>
#include <string>
#include "rapidcsv.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "aimooe_sdk/msg/aim_coord.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pf_base_measurement_aim");

    // 创建文件保存订阅数据
    std::string filename = "pf_base_measurement.csv";
    filename = "/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_system/pf_control/save/" + filename;
    std::ofstream logfile(filename);
    logfile << "frame_id,position.x,position.y,position.z,orientation.x,orientation.y,orientation.z\n";

    // 创建临时订阅者
    std::vector<aimooe_sdk::msg::AimCoord> messages;
    auto sub = node->create_subscription<aimooe_sdk::msg::AimCoord>(
        "aimooe_tracker",
        10,
        [&](const typename aimooe_sdk::msg::AimCoord::SharedPtr msg) {
            messages.push_back(*msg);
        });

    // 订阅5秒
    rclcpp::Rate rate(10);
    auto start = node->now();
    while ((node->now() - start).seconds() < 5.0) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    // 保存数据到同一个文件
    for (const auto& msg : messages) {
        logfile << msg.header.frame_id << ","
                << msg.position.x << "," << msg.position.y << "," << msg.position.z << ","
                << msg.orientation.x << "," << msg.orientation.y << "," << msg.orientation.z << "\n";
    }
    std::cout << "已追加" << messages.size() << "条数据到pf_base_measurement.csv" << std::endl;

    rclcpp::shutdown();
    return 0;
}