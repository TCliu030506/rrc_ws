#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <ur5_msg/msg/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "coordinate/srv/string_script.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#define D1 0.08935245198864346
#define D4 0.1108629663680774
#define D5 0.09481752424615827
#define D6 0.08250304262136304
#define A2 0.4255142946511841
#define A3 0.3922769788

struct pose_quan
{
    double position[3];
    double orientation[4];
};


class hand_eye: public rclcpp::Node
{
private:
    
    rclcpp::Subscription<ur5_msg::msg::RobotState>::SharedPtr robot_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub;
    rclcpp::Service<coordinate::srv::StringScript>::SharedPtr command_call;

    double robotstate[6];
    pose_quan aruco_pose;

    std::ofstream file_ur;
    std::ofstream file_aru;

    void record(); //record current pose of UR5 end joint and aruco
    


public:
    hand_eye(/* args */);
    ~hand_eye();

    Eigen::Matrix4d transform_handeye;
    double pose_eulerxyz_handeye[6];
    double pose_angleaxis_handeye[6];

    std::vector<cv::Mat> TargetToCameraMats;
    std::vector<cv::Mat> TargetToCameraVecs;
    std::vector<cv::Mat> UR5EndToBaseMats;
    std::vector<cv::Mat> UR5EndToBaseVecs;
    void calculate(); //calculate hand_eye transform


    void RobotstateCallback(const ur5_msg::msg::RobotState::SharedPtr msg);
    void ArucoposeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void CommandCallback(const coordinate::srv::StringScript::Request::SharedPtr req, const coordinate::srv::StringScript::Response::SharedPtr res);

};

