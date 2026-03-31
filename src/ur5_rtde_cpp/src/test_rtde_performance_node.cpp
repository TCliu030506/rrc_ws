#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <numeric>
#include <algorithm>
#include <sstream>
#include <cmath>
// #include <ur_rtde/rtde_control_interface.h>
// #include <ur_rtde/rtde_receive_interface.h>

class RTDEPerformanceNode : public rclcpp::Node {
public:
    RTDEPerformanceNode() : Node("rtde_performance_node") {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.102");
        this->declare_parameter<int>("test_duration", 10);
        this->declare_parameter<std::vector<int>>("frequencies", {10, 50, 100, 125, 200, 250, 500, 750, 1000});

        robot_ip_ = this->get_parameter("robot_ip").as_string();
        test_duration_ = this->get_parameter("test_duration").as_int();
        frequencies_ = this->get_parameter("frequencies").as_integer_array();

        result_pub_ = this->create_publisher<std_msgs::msg::String>("rtde_performance_result", 1);

        test_thread_ = std::thread(&RTDEPerformanceNode::run_test, this);
    }

    ~RTDEPerformanceNode() override {
        if (test_thread_.joinable()) test_thread_.join();
    }

private:
    void run_test() {
        // ur_rtde::RTDEControlInterface rtde_c(robot_ip_);
        // ur_rtde::RTDEReceiveInterface rtde_r(robot_ip_);
        for (int freq : frequencies_) {
            double period = 1.0 / freq;
            std::vector<double> delays;
            int lost = 0, count = 0;
            auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < test_duration_) {
                auto t0 = std::chrono::steady_clock::now();
                try {
                    // auto q = rtde_r.getActualQ();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 模拟
                    auto t1 = std::chrono::steady_clock::now();
                    double delay = std::chrono::duration<double>(t1 - t0).count();
                    delays.push_back(delay);
                } catch (...) {
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
                std::ostringstream oss;
                oss << "freq: " << freq << "Hz, count: " << count << ", lost: " << lost
                    << ", max: " << max_delay << "s, min: " << min_delay
                    << "s, avg: " << avg_delay << "s, std: " << std_delay << "s";
                auto msg = std_msgs::msg::String();
                msg.data = oss.str();
                result_pub_->publish(msg);
            }
        }
        // rtde_c.disconnect();
        // rtde_r.disconnect();
    }

    std::string robot_ip_;
    int test_duration_;
    std::vector<int64_t> frequencies_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
    std::thread test_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RTDEPerformanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
