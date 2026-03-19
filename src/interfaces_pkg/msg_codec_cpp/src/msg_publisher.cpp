#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "../include/msg_codec_cpp/msg_coder.hpp"
#include <cctype>
#include <iomanip>

using MType = MsgCoder::ValType;

class MsgPublisher : public rclcpp::Node {
public:
    MsgPublisher() : Node("msg_publisher") {
        MsgCoder::registerCommand(std::string("MV"), {{"cmd", MType::CHAR2}, {"id", MType::INT8}, {"pos", MType::INT64}, {"vel", MType::DOUBLE}});
        MsgCoder::registerCommands({
            {std::array<char, 2>{'C', 'F'}, {{"cmd", MType::CHAR2}, {"id", MType::INT8}, {"pos", MType::INT64}, {"vel", MType::DOUBLE}}},
            {std::array<char, 2>{'R', 'S'}, {{"cmd", MType::CHAR2}, {"id", MType::INT8}, {"status", MType::INT8}}}
        });

        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("encoded_msg", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MsgPublisher::publish_message, this)
        );
        RCLCPP_INFO(this->get_logger(), "MsgPublisher has been started.");
    }

private:
    MsgCoder::KeyValues data_to_encode = {
        {"cmd", std::array<char, 2>{'M', 'V'}},
        {"id",  static_cast<int8_t>(6)},
        {"pos", static_cast<int64_t>(1000)},
        {"vel", static_cast<double>(3.14159)}
    };
    void publish_message() {
        int64_t val = std::get<int64_t>(data_to_encode["pos"]);
        data_to_encode["pos"] = val + 1;

        try {
            auto encoded_msg = MsgCoder::msg_encode(data_to_encode);
            std_msgs::msg::UInt8MultiArray msg;
            msg.data = std::vector<uint8_t>(encoded_msg.begin(), encoded_msg.end());
            publisher_->publish(msg);

            std::ostringstream oss;
            for (const auto& byte : encoded_msg) {
                if (std::isprint(byte)) {
                    oss << byte;
                } else {
                    oss << "\\x" << std::hex << std::setw(2) << std::setfill('0') << (static_cast<int>(byte) & 0xFF);
                }
            }
            RCLCPP_INFO(this->get_logger(), "Published encoded message: %s", oss.str().c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error encoding message: %s", e.what());
        }
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MsgPublisher>());
    rclcpp::shutdown();
    return 0;
}