
#include "hand_eye.h"

#define FILE_PATH "/Lab_WS/rrc_ws/src/project_pkgs/robot_control_pkg/hand-eye/data/"

std::string write_filename(std::string filename)
{
    std::string path;
    std::time_t now = time(nullptr);
    path = std::string(std::getenv("HOME")) + FILE_PATH + filename + "-" + ctime(&now);
    path.pop_back();
    path = path+".txt";
    std::ofstream createFile(path);
    createFile.close();
    return path;
}

hand_eye::hand_eye(): Node("hand_eye_cali")
{
    robot_sub = create_subscription<ur5_msg::msg::RobotState>("robotstate",10,std::bind(&hand_eye::RobotstateCallback,this,std::placeholders::_1));
    aruco_sub = create_subscription<geometry_msgs::msg::PoseStamped>("aruco_single/pose",10,std::bind(&hand_eye::ArucoposeCallback,this,std::placeholders::_1));
    command_call = create_service<coordinate::srv::StringScript>("hand_eye_command",std::bind(&hand_eye::CommandCallback, this, std::placeholders::_1, std::placeholders::_2));
    transform_handeye = Eigen::Matrix4d::Identity();

    TargetToCameraMats.clear();
    UR5EndToBaseMats.clear();   
    std::string path_ur = write_filename("pose_ur");
    std::string path_aru = write_filename("pose_aru");
    file_ur.open(path_ur);
    file_aru.open(path_aru);
    if (!file_ur) std::cerr << "无法打开文件: " << path_ur << std::endl;
    if (!file_aru) std::cerr << "无法打开文件: " << path_aru << std::endl;

}   

hand_eye::~hand_eye()
{
    if (file_ur.is_open()) file_ur.close();
    if (file_aru.is_open()) file_aru.close();
    
}

void hand_eye::record()
{
    cv::Mat cvmat_aruco,cvmat_ur5;
    Eigen::Matrix4d transform_aruco,transform_ur5;
    cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);

    //aruco
    file_aru <<aruco_pose.position[0]<<","<<aruco_pose.position[1]<<","<<aruco_pose.position[2]<<","<<aruco_pose.orientation[0]<<","
            <<aruco_pose.orientation[1]<<","<<aruco_pose.orientation[2]<<","<<aruco_pose.orientation[3]<<std::endl;

    Eigen::Quaterniond q(aruco_pose.orientation);
    transform_aruco.block<3,3>(0,0) = q.normalized().toRotationMatrix();
    for(int i=0;i<3;i++) transform_aruco(i,3) = aruco_pose.position[i];
    transform_aruco.row(3)<<0.0,0.0,0.0,1.0;
    std::cout << "Transform_aruco: " << std::endl;
    std::cout << transform_aruco << std::endl;
    cv::eigen2cv(transform_aruco,cvmat_aruco);
    TargetToCameraMats.push_back(cvmat_aruco(R_rect).clone());
    TargetToCameraVecs.push_back(cvmat_aruco(T_rect).clone());
    

    //ur5
    file_ur <<robotstate[0]<<","<<robotstate[1]<<","<<robotstate[2]<<","
            <<robotstate[3]<<","<<robotstate[4]<<","<<robotstate[5]<<std::endl;
    file_aru.flush();
    file_ur.flush();

    Eigen::Vector3d orientation;
    for(int i=0;i<3;i++) orientation[i] = robotstate[i+3];
    Eigen::AngleAxisd angleaxis(orientation.norm(),orientation.normalized());
    transform_ur5.block<3,3>(0,0) = angleaxis.toRotationMatrix();
    for(int i=0;i<3;i++) transform_ur5(i,3) = robotstate[i];
    transform_ur5.row(3)<<0.0,0.0,0.0,1.0;
    std::cout << "Transform_ur5: " << std::endl;
    std::cout << transform_ur5 << std::endl;
    cv::eigen2cv(transform_ur5,cvmat_ur5);
    UR5EndToBaseMats.push_back(cvmat_ur5(R_rect).clone());
    UR5EndToBaseVecs.push_back(cvmat_ur5(T_rect).clone());
}

void hand_eye::calculate()
{
    cv::Mat R_temp = (cv::Mat_<double>(3, 3));
    cv::Mat T_temp = (cv::Mat_<double>(3, 1));
    cv::calibrateHandEye(UR5EndToBaseMats,UR5EndToBaseVecs,TargetToCameraMats,TargetToCameraVecs,R_temp,T_temp,cv::CALIB_HAND_EYE_TSAI);
    Eigen::Matrix3d matrix_temp;
    cv::cv2eigen(R_temp,matrix_temp);
    transform_handeye.block<3,3>(0,0) = matrix_temp;
    for(int i=0;i<3;i++) transform_handeye(i,3) = T_temp.at<double>(i,0);
    Eigen::Vector3d eulerAngle = matrix_temp.eulerAngles(0,1,2);
    Eigen::AngleAxisd axisAngle(matrix_temp);
    for(int i=0;i<3;i++)
    {
        pose_angleaxis_handeye[i] = T_temp.at<double>(i,0); 
        pose_angleaxis_handeye[i+3] = axisAngle.angle() * axisAngle.axis()[i];
        pose_eulerxyz_handeye[i] = T_temp.at<double>(i,0);
        pose_eulerxyz_handeye[i+3] = eulerAngle[i];
    }
    file_ur << "Transform_handeye: " << std::endl;
    file_ur << transform_handeye << std::endl;
    file_ur.flush();
    std::cout << "Transform_handeye: " << std::endl;
    std::cout << transform_handeye << "\n";
    //std::cout << "R_temp: " << std::endl;
    //std::cout << R_temp << "\n";
}

void hand_eye::RobotstateCallback(const ur5_msg::msg::RobotState::SharedPtr msg)
{
    for(int i=0;i<6;i++) robotstate[i] = msg->carte_pos[i];
}

void hand_eye::ArucoposeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    aruco_pose.position[0] = msg->pose.position.x;
    aruco_pose.position[1] = msg->pose.position.y;
    aruco_pose.position[2] = msg->pose.position.z;
    aruco_pose.orientation[0] = msg->pose.orientation.x;
    aruco_pose.orientation[1] = msg->pose.orientation.y;
    aruco_pose.orientation[2] = msg->pose.orientation.z;
    aruco_pose.orientation[3] = msg->pose.orientation.w;
}

void hand_eye::CommandCallback(const coordinate::srv::StringScript::Request::SharedPtr req, const coordinate::srv::StringScript::Response::SharedPtr res)
{
    if(req->command == "Rec")
    {
        record();
        res->result = "Record Call OK!";
    }
    else if(req->command == "Cal")
    {
        calculate();
        res->result = "Calculate Call OK!";
    }
}
