#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "omni_msgs/msg/omni_state.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
using namespace std::chrono_literals;

class Omni_Broadcaster
{
public:
    Omni_Broadcaster(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
    }
    void init()
    {
        framename_ = node_->declare_parameter<std::string>("framename", "frame");

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

        sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/phantom/pose", 10,
            std::bind(&Omni_Broadcaster::stateCallback, this, std::placeholders::_1));
        sub_button_ = node_->create_subscription<omni_msgs::msg::OmniButtonEvent>("/phantom/button", 10,
            std::bind(&Omni_Broadcaster::buttonCallback, this, std::placeholders::_1));
        broadcast_timer = node_->create_wall_timer(4ms, std::bind(&Omni_Broadcaster::positionBroadcaster, this));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "test init");
    }
    
private:
    void stateCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> state_msg)
    {
        actTouchPose = *state_msg; 
    }
    void buttonCallback(const std::shared_ptr<omni_msgs::msg::OmniButtonEvent> button_msg)
    {
        actbutton = *button_msg;  
    }
    void positionBroadcaster()
    {
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = node_->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = framename_;
        
        t.transform.translation.x = actTouchPose.pose.position.x;
        t.transform.translation.y = actTouchPose.pose.position.y;
        t.transform.translation.z = actTouchPose.pose.position.z;

        t.transform.rotation.x = actTouchPose.pose.orientation.x;
        t.transform.rotation.y = actTouchPose.pose.orientation.y;
        t.transform.rotation.z = actTouchPose.pose.orientation.z;
        t.transform.rotation.w = actTouchPose.pose.orientation.w;

        if((actbutton.grey_button == 1) && (actbutton.white_button == 1))
        {
            tf_broadcaster_->sendTransform(t);
        }
    }

    geometry_msgs::msg::PoseStamped actTouchPose;
    omni_msgs::msg::OmniButtonEvent actbutton;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr sub_button_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr broadcast_timer;
    std::string framename_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("omni_broadcaster");
    Omni_Broadcaster broadcaster(node);
    broadcaster.init();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "test main");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
