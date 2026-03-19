#pragma once

#include "rclcpp/rclcpp.hpp"
#include "inspire_msg/srv/inspire_script.hpp"

namespace zzr_kinematics {

class zzr_motion : public rclcpp::Node
{
private:
  rclcpp::Client<inspire_msg::srv::InspireScript>::SharedPtr inspire_client;

public:
  zzr_motion(/* args */);
  ~zzr_motion();

  void cmd_movp(uint8_t id, int16_t pos);

  void cmd_movs(uint8_t id, int16_t pos);

  void cmd_movj(uint8_t id, int16_t pos, int16_t speed);

};

/*******************************************************************************/
//class zzr_motion
zzr_motion::zzr_motion() : Node("zzr_robot")
{
  inspire_client = create_client<inspire_msg::srv::InspireScript>("insactuator_srv");
};

zzr_motion::~zzr_motion(){};

void zzr_motion::cmd_movp(uint8_t id, int16_t pos)
{
  auto shared_this = shared_from_this();
  auto srv = std::make_shared<inspire_msg::srv::InspireScript::Request>();

  srv->command = "P";
  srv->id = id;
  srv->data.clear();
  srv->data.push_back(pos);

  auto result_future = inspire_client->async_send_request(srv);

  if (rclcpp::spin_until_future_complete(shared_this, result_future) == rclcpp::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(get_logger(), "Result: %s", result_future.get()->result.c_str());
  else
    RCLCPP_ERROR(get_logger(), "Failed to call service: %s", inspire_client->get_service_name());
};

void zzr_motion::cmd_movs(uint8_t id, int16_t pos)
{
  auto shared_this = shared_from_this();
  auto srv = std::make_shared<inspire_msg::srv::InspireScript::Request>();

  srv->command = "X";
  srv->id = id;
  srv->data.clear();
  srv->data.push_back(pos);

  auto result_future = inspire_client->async_send_request(srv);

  if (rclcpp::spin_until_future_complete(shared_this, result_future) == rclcpp::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(get_logger(), "Result: %s", result_future.get()->result.c_str());
  else
    RCLCPP_ERROR(get_logger(), "Failed to call service: %s", inspire_client->get_service_name());
};

void zzr_motion::cmd_movj(uint8_t id, int16_t pos, int16_t speed)
{
  auto shared_this = shared_from_this();
  auto srv = std::make_shared<inspire_msg::srv::InspireScript::Request>();

  srv->command = "V";
  srv->id = id;
  srv->data.clear();
  srv->data.push_back(pos);
  srv->data.push_back(speed);

  auto result_future = inspire_client->async_send_request(srv);

  if (rclcpp::spin_until_future_complete(shared_this, result_future) == rclcpp::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(get_logger(), "Result: %s", result_future.get()->result.c_str());
  else
    RCLCPP_ERROR(get_logger(), "Failed to call service: %s", inspire_client->get_service_name());
};

}