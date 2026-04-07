#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <numeric>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <locale>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

class RTDEPerformanceNode : public rclcpp::Node {
public:
    RTDEPerformanceNode() : Node("rtde_performance_node") {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.102");
        this->declare_parameter<int>("test_duration", 10);
        this->declare_parameter<std::vector<int>>("frequencies", {10, 50, 100, 125, 200, 250, 500, 750, 1000});
        this->declare_parameter<double>("simulated_work_ms", 0.0);

        robot_ip_ = this->get_parameter("robot_ip").as_string();
        test_duration_ = this->get_parameter("test_duration").as_int();
        frequencies_ = this->get_parameter("frequencies").as_integer_array();
        simulated_work_ms_ = this->get_parameter("simulated_work_ms").as_double();

        result_pub_ = this->create_publisher<std_msgs::msg::String>("rtde_performance_result", 1);
        RCLCPP_INFO(this->get_logger(), "RTDE性能测试节点启动");

        test_thread_ = std::thread(&RTDEPerformanceNode::run_test, this);
    }

    ~RTDEPerformanceNode() override {
        if (test_thread_.joinable()) test_thread_.join();
    }

private:
    void run_test() {
        ur_rtde::RTDEControlInterface rtde_control(robot_ip_);
        ur_rtde::RTDEReceiveInterface rtde_receive(robot_ip_);
        for (const auto freq_i64 : frequencies_) {
            const int freq = static_cast<int>(freq_i64);
            double period = 1.0 / freq;
            std::vector<double> read_times;
            std::vector<double> loop_times;
            int lost = 0, count = 0;
            int deadline_miss = 0;
            int fresh_count = 0;
            int stale_count = 0;
            double last_ts = 0.0;
            bool has_last_ts = false;
            auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < test_duration_) {
                auto t0 = std::chrono::steady_clock::now();
                try {
                    auto q = rtde_receive.getActualQ();
                    (void)q;
                    const auto t_read = std::chrono::steady_clock::now();
                    const double read_cost = std::chrono::duration<double>(t_read - t0).count();
                    read_times.push_back(read_cost);

                    if (simulated_work_ms_ > 0.0) {
                        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(simulated_work_ms_));
                    }

                    try {
                        const double curr_ts = rtde_receive.getTimestamp();
                        if (has_last_ts) {
                            if (curr_ts > last_ts) {
                                fresh_count++;
                            } else {
                                stale_count++;
                            }
                        }
                        last_ts = curr_ts;
                        has_last_ts = true;
                    } catch (...) {
                        // 某些 RTDE 固件或接口版本可能不支持时间戳读取。
                    }
                } catch (...) {
                    lost++;
                }

                const double loop_cost = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                loop_times.push_back(loop_cost);
                if (loop_cost > period) {
                    deadline_miss++;
                }

                count++;

                if (loop_cost < period) {
                    std::this_thread::sleep_for(std::chrono::duration<double>(period - loop_cost));
                }
            }

            if (!read_times.empty()) {
                const double read_max = *std::max_element(read_times.begin(), read_times.end());
                const double read_min = *std::min_element(read_times.begin(), read_times.end());
                const double read_avg = std::accumulate(read_times.begin(), read_times.end(), 0.0) / read_times.size();
                double read_std = 0.0;
                for (const auto d : read_times) {
                    read_std += (d - read_avg) * (d - read_avg);
                }
                read_std = std::sqrt(read_std / read_times.size());

                const double loop_max = *std::max_element(loop_times.begin(), loop_times.end());
                const double loop_avg = std::accumulate(loop_times.begin(), loop_times.end(), 0.0) / loop_times.size();
                const double actual_hz = static_cast<double>(count) / std::max(static_cast<double>(test_duration_), 1e-9);
                const double miss_rate = (count > 0) ? (static_cast<double>(deadline_miss) * 100.0 / static_cast<double>(count)) : 0.0;
                const int freshness_total = fresh_count + stale_count;
                const double freshness_rate = (freshness_total > 0)
                    ? (static_cast<double>(fresh_count) * 100.0 / static_cast<double>(freshness_total))
                    : -1.0;

                std::ostringstream oss;
                oss.imbue(std::locale::classic());
                oss << std::fixed;
                oss << "freq: " << freq << "Hz, count: " << count << ", lost: " << lost
                    << ", actual: " << std::setprecision(1) << actual_hz << "Hz"
                    << ", miss: " << deadline_miss << " (" << std::setprecision(2) << miss_rate << "%)"
                    << ", read_max: " << std::setprecision(6) << read_max << "s"
                    << ", read_min: " << std::setprecision(6) << read_min << "s"
                    << ", read_avg: " << std::setprecision(6) << read_avg << "s"
                    << ", read_std: " << std::setprecision(6) << read_std << "s"
                    << ", loop_max: " << std::setprecision(6) << loop_max << "s"
                    << ", loop_avg: " << std::setprecision(6) << loop_avg << "s"
                    << ", fresh_rate: " << std::setprecision(2) << freshness_rate << "%";
                auto msg = std_msgs::msg::String();
                msg.data = oss.str();
                result_pub_->publish(msg);
            }
        }
        rtde_control.disconnect();
        rtde_receive.disconnect();
    }

    std::string robot_ip_;
    int test_duration_;
    std::vector<int64_t> frequencies_;
    double simulated_work_ms_;
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
