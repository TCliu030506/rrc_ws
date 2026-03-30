// C++ RTDE通信性能测试程序
// 依赖：ur_rtde C++接口（需提前准备好），可用chrono计时
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <numeric>
#include <algorithm>
// #include <ur_rtde/rtde_control_interface.h>
// #include <ur_rtde/rtde_receive_interface.h>

int main(int argc, char **argv) {
    // 参数
    std::string robot_ip = "192.168.1.102";
    std::vector<int> frequencies = {10, 50, 100, 125, 200, 250, 500, 750, 1000};
    int test_duration = 10; // 秒

    // std::cout << "连接到UR机器人..." << robot_ip << std::endl;
    // ur_rtde::RTDEControlInterface rtde_c(robot_ip);
    // ur_rtde::RTDEReceiveInterface rtde_r(robot_ip);
    // std::cout << "连接成功！" << std::endl;

    for (int freq : frequencies) {
        std::cout << "\n测试频率: " << freq << " Hz" << std::endl;
        double period = 1.0 / freq;
        std::vector<double> delays;
        int lost = 0;
        int count = 0;
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < test_duration) {
            auto t0 = std::chrono::steady_clock::now();
            try {
                // auto q = rtde_r.getActualQ(); // 需替换为C++接口
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 模拟调用
                auto t1 = std::chrono::steady_clock::now();
                double delay = std::chrono::duration<double>(t1 - t0).count();
                delays.push_back(delay);
            } catch (...) {
                std::cout << "通信异常" << std::endl;
                lost++;
            }
            count++;
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (elapsed < period) {
                std::this_thread::sleep_for(std::chrono::duration<double>(period - elapsed));
            }
        }
        if (!delays.empty()) {
            double max_delay = *std::max_element(delays.begin(), delays.end());
            double min_delay = *std::min_element(delays.begin(), delays.end());
            double avg_delay = std::accumulate(delays.begin(), delays.end(), 0.0) / delays.size();
            double std_delay = 0.0;
            for (auto d : delays) std_delay += (d - avg_delay) * (d - avg_delay);
            std_delay = std::sqrt(std_delay / delays.size());
            std::cout << "频率: " << freq << "Hz, 总包数: " << count << ", 丢包: " << lost
                      << ", 最大延时: " << max_delay << "s, 最小延时: " << min_delay
                      << "s, 平均延时: " << avg_delay << "s" << std::endl;
        }
    }
    // rtde_c.disconnect();
    // rtde_r.disconnect();
    return 0;
}
