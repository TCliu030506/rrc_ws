#include <rclcpp/rclcpp.hpp>
#include "../include/ur5_control_cpp/urscript.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("ur_script_demo");

    ur_script::script_pub demo;
    double pose_angleaxis[6];
    for(int i=0;i<6;i++)
    {
        if(i<3)
        {
            std::cout<<"Please input the pose["<< i <<"]/mm\n";
            std::cin>>pose_angleaxis[i];
            std::cin.ignore();
            pose_angleaxis[i]/=1000;
        }
        else
        {
            std::cout<<"Please input the pose["<< i <<"]/rad\n";
            std::cin>>pose_angleaxis[i];
            std::cin.ignore();
        }
    }
    printf("Please check the input: \n[%lf,%lf,%lf,%lf,%lf,%lf]\n",pose_angleaxis[0],pose_angleaxis[1],pose_angleaxis[2],pose_angleaxis[3],pose_angleaxis[4],pose_angleaxis[5]);
    printf("Input 1 to continue, others for quit\n");
    int flag;
    std::cin>>flag;
    std::cin.ignore();
    if(flag==1) demo.urscript_publish_movej_pose(pose_angleaxis, 0.2, 0.2, 0, 0);

    rclcpp::spin(node);

    return 0;
}