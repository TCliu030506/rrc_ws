#include <rclcpp/rclcpp.hpp>
// 移除原先的 std_msgs MultiArray 相关头
#include <memory>
#include <chrono>
#include <sstream>
#include <vector>
#include <string>
#include <array>
#include "xmate_cr7_script/cr7_script.h"
#include "xmate_cr7_msg/msg/robot_state.hpp"

using namespace std::chrono_literals;

class Cr7StateSeneor : public rclcpp::Node {
public:
  Cr7StateSeneor() : Node("cr7_state_seneor") {
    // 创建脚本客户端（用于请求服务器）
    cr7_client_ = std::make_shared<cr7_script::Cr7ScriptClient>();

    // 使用 RobotState 消息（话题名：robotstate）
    robotstate_pub_ = this->create_publisher<xmate_cr7_msg::msg::RobotState>("robotstate", 10);

    // 频率参数
    this->declare_parameter<double>("state_poll_hz", 100.0);
    double hz = this->get_parameter("state_poll_hz").as_double();
    if (hz <= 0.0) hz = 50.0;
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));

    timer_ = this->create_wall_timer(period, std::bind(&Cr7StateSeneor::tick, this));

    RCLCPP_INFO(this->get_logger(), "cr7_state_seneor 已启动，轮询频率：%.2f Hz", hz);
  }

private:
  std::shared_ptr<cr7_script::Cr7ScriptClient> cr7_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<xmate_cr7_msg::msg::RobotState>::SharedPtr robotstate_pub_;

  // 解析形如 "prefix:val1,val2,..." 的 6 个 double 到 std::array
  static bool parse_array_after_prefix(const std::string &src, const std::string &prefix, std::array<double, 6> &out) {
    auto pos = src.find(prefix);
    if (pos == std::string::npos) return false;
    std::string list = src.substr(pos + prefix.size());
    // 截断到分号（若存在）
    auto semi = list.find(';');
    if (semi != std::string::npos) list = list.substr(0, semi);
    std::stringstream ss(list);
    std::string item;
    std::vector<double> vals;
    try {
      while (std::getline(ss, item, ',')) {
        if (item.empty()) continue;
        vals.push_back(std::stod(item));
      }
    } catch (...) {
      return false;
    }
    if (vals.size() < 6) return false;
    for (size_t i = 0; i < 6; ++i) out[i] = vals[i];
    return true;
  }

  void tick() {
    std::string res;
    std::array<double, 6> joints{};
    std::array<double, 6> pose{};
    std::array<double, 6> jvel{};

    bool ok_joint = false, ok_pose = false, ok_jvel = false;

    // joint
    if (cr7_client_->readj(res)) {
      ok_joint = parse_array_after_prefix(res, "joint:", joints);
      if (!ok_joint) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to parse joint from '%s'", res.c_str());
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "readj call failed");
    }

    // pose (rpy)
    if (cr7_client_->readp("world", res)) {
      ok_pose = parse_array_after_prefix(res, "pose:", pose);
      if (!ok_pose) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to parse pose from '%s'", res.c_str());
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "readp call failed");
    }

    // joint velocity
    if (cr7_client_->get_joint_vel(res)) {
      ok_jvel = parse_array_after_prefix(res, "joint_vel:", jvel);
      if (!ok_jvel) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to parse joint_vel from '%s'", res.c_str());
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "get_joint_vel call failed");
    }

    if (ok_joint && ok_pose && ok_jvel) {
      xmate_cr7_msg::msg::RobotState msg;
      msg.header.stamp = this->now();
      for (size_t i = 0; i < 6; ++i) {
        msg.joint_pos[i] = joints[i];
        msg.carte_pos[i] = pose[i];
        msg.joint_vel[i] = jvel[i];
      }
      robotstate_pub_->publish(msg);
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Cr7StateSeneor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
