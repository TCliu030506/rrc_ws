#include "../include/pf_control/pf_control.h"
#include <iostream>
#include <string>
#include "rapidcsv.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "aimooe_sdk/msg/aim_coord.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>

typedef Eigen::Matrix3d           EM3d;
typedef Eigen::Vector3d           EV3d;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("save_file_demo_node");
    
    pf_control controller("/dev/ttyUSB1");
    std::string mode;
    std::string input;

    // 从CSV文件读取坐标点
    std::vector<EV3d> points;
    try
    {
        // 获取资源文件路径
        std::string package_path = ament_index_cpp::get_package_share_directory("pf_control");
        std::string csv_path = package_path + "/resource/1105_input_forfitting.csv";
        rapidcsv::Document doc(csv_path);

        // 遍历CSV中的行
        for (size_t i = 0; i < doc.GetRowCount(); ++i)
        {
            double x = doc.GetCell<double>(0, i);
            double y = doc.GetCell<double>(1, i);
            double z = doc.GetCell<double>(2, i);
            points.push_back(EV3d(x, y, z));

            std::cout << "已读取点位: (" << x << ", " << y << ", " << z << ")" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Excel文件读取失败: " << e.what() << std::endl;
        return 1;
    }

    // 示例：创建文件保存订阅数据
    std::string filename = "1105_points_measurement.csv";
    filename = "/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_system/pf_control/save/" + filename;
    std::ofstream logfile(filename);
    logfile << "index,x,y,z\n";

    // 示例：循环运动至每个目标点，并订阅aimooe_tracker话题
    for(size_t i=0; i<points.size(); i++) {
        // 移动至目标点(不需考虑)
        controller.pf_movej(points[i][0], points[i][1], points[i][2], 5);
        sleep(1); // 等待1秒，确保移动完成

        // 示例：创建临时订阅者
        std::vector<aimooe_sdk::msg::AimCoord> messages;
        auto sub = node->create_subscription<aimooe_sdk::msg::AimCoord>("aimooe_coord",10,
            [&](const typename aimooe_sdk::msg::AimCoord::SharedPtr msg) {
                messages.push_back(*msg);});

        // 示例：订阅2秒
        rclcpp::Rate rate(10);
        auto start = node->now();
        while ((node->now() - start).seconds() < 2.0) {
            rclcpp::spin_some(node);
            rate.sleep();
        }

        // 示例：保存数据到同一个文件
        for (const auto& msg : messages) {
            logfile << i+1 << ","<< msg.position.x << "," << msg.position.y << "," << msg.position.z << "\n";
        }
        std::cout << "已追加" << messages.size() << "条数据到目标文件 " << filename << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}