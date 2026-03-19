#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

namespace force_sen
{

class ForceSensorAxis6Node : public rclcpp::Node
{
public:
  ForceSensorAxis6Node()
  : Node("force_sensor_axis_6")
  {
    declare_parameter<std::string>("forcesensorport", "/dev/ttyUSB0");
    declare_parameter<int>("forcesensor_rate", 100);
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<std::string>("frame_id", "base_frame");
    declare_parameter<std::string>("topic_name", "wrench");
    declare_parameter<bool>("auto_zero", false);

    frame_id_ = get_parameter("frame_id").as_string();
    topic_name_ = get_parameter("topic_name").as_string();
    forcesensor_rate_ = get_parameter("forcesensor_rate").as_int();

    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(topic_name_, 10);

    if (!open_serial()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize serial port.");
      throw std::runtime_error("serial initialization failed");
    }

    if (get_parameter("auto_zero").as_bool()) {
      set_adjzf('1');
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      flush_serial();
    }

    if (forcesensor_rate_ <= 0) {
      forcesensor_rate_ = 100;
    }
    auto period = std::chrono::milliseconds(1000 / forcesensor_rate_);
    timer_ = create_wall_timer(period, std::bind(&ForceSensorAxis6Node::poll_once, this));

    RCLCPP_INFO(
      get_logger(),
      "Started six-axis force sensor publisher. port=%s rate=%d topic=%s frame_id=%s",
      serial_port_.c_str(), forcesensor_rate_, topic_name_.c_str(), frame_id_.c_str());
  }

private:
  bool open_serial()
  {
    serial_port_ = get_parameter("forcesensorport").as_string();
    const int baudrate = get_parameter("baudrate").as_int();

    try {
      ser_.setPort(serial_port_);
      ser_.setBaudrate(static_cast<uint32_t>(baudrate));
      serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
      ser_.setTimeout(timeout);
      ser_.open();
    } catch (const serial::IOException & e) {
      RCLCPP_ERROR(get_logger(), "Unable to open serial port %s: %s", serial_port_.c_str(), e.what());
      return false;
    }

    if (!ser_.isOpen()) {
      RCLCPP_ERROR(get_logger(), "Serial port is not open: %s", serial_port_.c_str());
      return false;
    }

    return true;
  }

  void poll_once()
  {
    if (!ser_.isOpen()) {
      return;
    }

    send_god();

    std::vector<uint8_t> packet;
    while (read_packet(packet)) {
      geometry_msgs::msg::WrenchStamped msg;
      if (!parse_wrench_packet(packet, msg)) {
        continue;
      }
      msg.header.stamp = now();
      msg.header.frame_id = frame_id_;
      wrench_pub_->publish(msg);
    }
  }

  void send_god()
  {
    static const uint8_t cmd[] = {0x41, 0x54, 0x2b, 0x47, 0x4f, 0x44, 0x0d, 0x0a};
    ser_.write(cmd, sizeof(cmd));
  }

  void set_adjzf(char adjzf)
  {
    uint8_t cmd[23] = {0};
    cmd[0] = 0x41;
    cmd[1] = 0x54;
    cmd[2] = 0x2b;
    cmd[3] = 0x41;
    cmd[4] = 0x44;
    cmd[5] = 0x4a;
    cmd[6] = 0x5a;
    cmd[7] = 0x46;
    cmd[8] = 0x3d;
    cmd[9] = static_cast<uint8_t>(adjzf);
    cmd[10] = 0x3b;
    cmd[11] = static_cast<uint8_t>(adjzf);
    cmd[12] = 0x3b;
    cmd[13] = static_cast<uint8_t>(adjzf);
    cmd[14] = 0x3b;
    cmd[15] = static_cast<uint8_t>(adjzf);
    cmd[16] = 0x3b;
    cmd[17] = static_cast<uint8_t>(adjzf);
    cmd[18] = 0x3b;
    cmd[19] = static_cast<uint8_t>(adjzf);
    cmd[20] = 0x3b;
    cmd[21] = 0x0d;
    cmd[22] = 0x0a;
    ser_.write(cmd, sizeof(cmd));
  }

  void flush_serial()
  {
    if (ser_.available() > 0) {
      const size_t n = ser_.available();
      (void)ser_.read(n);
    }
    rx_buffer_.clear();
  }

  bool read_packet(std::vector<uint8_t> & packet)
  {
    const size_t available = ser_.available();
    if (available > 0) {
      const std::string raw = ser_.read(available);
      rx_buffer_.insert(rx_buffer_.end(), raw.begin(), raw.end());
    }

    while (rx_buffer_.size() >= 2 && !(rx_buffer_[0] == 0xaa && rx_buffer_[1] == 0x55)) {
      rx_buffer_.erase(rx_buffer_.begin());
    }

    if (rx_buffer_.size() < 4) {
      return false;
    }

    const uint16_t payload_len = static_cast<uint16_t>(rx_buffer_[2] << 8) | rx_buffer_[3];
    const size_t total_len = static_cast<size_t>(payload_len) + 4;
    if (rx_buffer_.size() < total_len) {
      return false;
    }

    packet.assign(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(total_len));
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(total_len));
    return true;
  }

  bool parse_wrench_packet(
    const std::vector<uint8_t> & packet,
    geometry_msgs::msg::WrenchStamped & msg)
  {
    constexpr size_t kDataOffset = 6;
    constexpr size_t kChannelCount = 6;
    constexpr size_t kBytesPerChannel = 4;
    const size_t needed = kDataOffset + kChannelCount * kBytesPerChannel;

    if (packet.size() < needed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Packet too short for 6-axis wrench data: %zu bytes", packet.size());
      return false;
    }

    float value[6] = {0.0F};
    for (size_t i = 0; i < kChannelCount; ++i) {
      std::memcpy(&value[i], &packet[kDataOffset + i * kBytesPerChannel], kBytesPerChannel);
    }

    msg.wrench.force.x = static_cast<double>(value[0]);
    msg.wrench.force.y = static_cast<double>(value[1]);
    msg.wrench.force.z = static_cast<double>(value[2]);
    msg.wrench.torque.x = static_cast<double>(value[3]);
    msg.wrench.torque.y = static_cast<double>(value[4]);
    msg.wrench.torque.z = static_cast<double>(value[5]);

    return true;
  }

private:
  serial::Serial ser_;
  std::string serial_port_;
  std::string frame_id_;
  std::string topic_name_;
  int forcesensor_rate_{100};
  std::vector<uint8_t> rx_buffer_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace force_sen

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<force_sen::ForceSensorAxis6Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
