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
    auto node = std::make_shared<rclcpp::Node>("pf_move_test_node");
    
    pf_control controller("/dev/ttyUSB0");
    std::string mode;
    std::string input;
    std::cout << "开始定位精度测试实验！" << std::endl;

    // 从CSV文件读取坐标点
    std::vector<EV3d> points;
    try
    {
        // 获取资源文件路径
        std::string package_path = ament_index_cpp::get_package_share_directory("pf_control");
        std::string csv_path = package_path + "/resource/01 定位精度测试实验 input-2.csv";
        rapidcsv::Document doc(csv_path, rapidcsv::LabelParams(-1, -1));

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

    // 创建文件保存订阅数据
    std::string filename = "01 定位精度测试实验 测量数据-2(8).csv";
    filename = "/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_system/pf_control/save/" + filename;
    std::ofstream logfile(filename);
    // 保存订阅数据到CSV文件
    logfile << "index,x,y,z\n";

    bool target_point = false;

    // 循环运动至每个目标点，并订阅aimooe_tracker话题
    for(size_t i=0; i<points.size(); i++) {
        // 移动至目标点
        std::cout << "移动至点位 " << (i+1) << ": (" 
                  << points[i][0] << ", " 
                  << points[i][1] << ", " 
                  << points[i][2] << ")" << std::endl;
        controller.pf_movej(points[i][0], points[i][1], points[i][2], 5);
        sleep(2); // 等待2秒，确保移动完成
        std::cout << "已移动至点位 " << (i+1) << std::endl;

        // 判断是不是目标点
        if(target_point) {
            std::cout << "开始订阅点位 " << (i+1) << " 的数据..." << std::endl;
            // 创建临时订阅者
            std::vector<aimooe_sdk::msg::AimCoord> messages;
            auto sub = node->create_subscription<aimooe_sdk::msg::AimCoord>(
                "aimooe_coord",
                10,
                [&](const typename aimooe_sdk::msg::AimCoord::SharedPtr msg) {
                    messages.push_back(*msg);
                });
            // 订阅2秒
            rclcpp::Rate rate(10);
            auto start = node->now();
            while ((node->now() - start).seconds() < 2.0) {
                rclcpp::spin_some(node);
                rate.sleep();
            }
            // 保存数据到同一个文件
            for (const auto& msg : messages) {
                logfile << i+1 << ","
                        << msg.position.x << "," << msg.position.y << "," << msg.position.z << "\n";
            }
            std::cout << "已追加" << messages.size() << "条数据到目标文件 " << filename << std::endl;
        }
        // 切换目标点标志
        target_point = !target_point;
       
    }

    rclcpp::shutdown();
    return 0;
}