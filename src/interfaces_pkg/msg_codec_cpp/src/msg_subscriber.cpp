#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "../include/msg_codec_cpp/msg_coder.hpp"

using MType = MsgCoder::ValType;

class MsgSubscriber : public rclcpp::Node {
public:
    MsgSubscriber() : Node("msg_subscriber") {
        MsgCoder::registerCommand(std::string("MV"), {{"cmd", MType::CHAR2}, {"id", MType::INT8}, {"pos", MType::INT64}, {"vel", MType::DOUBLE}});
        MsgCoder::registerCommands({
            {std::array<char, 2>{'C', 'F'}, {{"cmd", MType::CHAR2}, {"id", MType::INT8}, {"pos", MType::INT64}, {"vel", MType::DOUBLE}}},
            {std::array<char, 2>{'R', 'S'}, {{"cmd", MType::CHAR2}, {"id", MType::INT8}, {"status", MType::INT8}}}
        });
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "encoded_msg",
            10,
            std::bind(&MsgSubscriber::decode_message, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "MsgSubscriber has been started.");
    }

private:
    void decode_message(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        try {
            std::vector<char> encoded_msg(msg->data.begin(), msg->data.end());
            printf("Received encoded message of size: %zu\n", encoded_msg.size());
            auto decoded_msg = MsgCoder::msg_decode(encoded_msg);

            RCLCPP_INFO(this->get_logger(), "Decoded message:");
            for (const auto &pair : decoded_msg) {
                if (std::holds_alternative<std::array<char, 2>>(pair.second)) {
                    auto value = std::get<std::array<char, 2>>(pair.second);
                    RCLCPP_INFO(this->get_logger(), "  %s: %c%c", pair.first.c_str(), value[0], value[1]);
                } else if (std::holds_alternative<int8_t>(pair.second)) {
                    RCLCPP_INFO(this->get_logger(), "  %s: %d", pair.first.c_str(), std::get<int8_t>(pair.second));
                } else if (std::holds_alternative<int64_t>(pair.second)) {
                    RCLCPP_INFO(this->get_logger(), "  %s: %ld", pair.first.c_str(), std::get<int64_t>(pair.second));
                } else if (std::holds_alternative<double>(pair.second)) {
                    RCLCPP_INFO(this->get_logger(), "  %s: %f", pair.first.c_str(), std::get<double>(pair.second));
                } else {
                    RCLCPP_WARN(this->get_logger(), "  %s: [unknown type]", pair.first.c_str());
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error decoding message: %s", e.what());
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MsgSubscriber>());
    rclcpp::shutdown();
    return 0;
}