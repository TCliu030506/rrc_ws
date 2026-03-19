
#include "rclcpp/rclcpp.hpp"
#include <fstream>

//#include "ft_motor_msg/msg/ft_motor.hpp"
#include "ft_motor_msg/srv/ft_command.hpp"
#include "aimooe_sdk/msg/aim_coord.hpp"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

struct Point
{
    double pose[6];
    double mean_error;
};


class rcm_calibration : public rclcpp::Node
{
private:
    
    //rclcpp::Subscription<ft_motor_msg::msg::FTMotor>::SharedPtr ftmotor_sub;
    rclcpp::Subscription<aimooe_sdk::msg::AimCoord>::SharedPtr aimooe_sub;
    rclcpp::Client<ft_motor_msg::srv::FTCommand>::SharedPtr ftmotor_client;
    aimooe_sdk::msg::AimCoord aimooe_msg;

    std::vector<Point> cali_point_list;
    
    std::vector<double> pos_1;
    std::vector<double> pos_2;

    void record();

public:
    rcm_calibration(/* args */);
    ~rcm_calibration();

    void initialization();
    
    void collection();

    void least_square_cal(); //least square method for calculation

    //void ftmotorCallback(const puncture_path_msg::msg::PuncPathPosition::SharedPtr msg);
    void aimooeCallback(const aimooe_sdk::msg::AimCoord::SharedPtr msg);
};

rcm_calibration::rcm_calibration(/* args */): Node("RCM_calibration")
{
    //ftmotor_sub = create_subscription<puncture_path_msg::msg::PuncPathPosition>("punc_path_msg", 10, std::bind(&rcm_calibration::ftmotorCallback, this, std::placeholders::_1));
    aimooe_sub = create_subscription<aimooe_sdk::msg::AimCoord>("aimooe_tracker",10,std::bind(&rcm_calibration::aimooeCallback,this,std::placeholders::_1));
    ftmotor_client = create_client<ft_motor_msg::srv::FTCommand>("punc_path_adjust_srv");
    while(!ftmotor_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(get_logger(),"/punc_path_adjust_srv: waiting...");
    }
    cali_point_list.clear();
    pos_1.clear();
    pos_2.clear();
    double pos_1_range_min = 0.0;
    double pos_1_range_max = 36.0;
    double pos_2_range_min = -56.0;
    double pos_2_range_max = 0.0;

    double step_1 = 6.0;
    double step_2 = 8.0;
    double pos_1_temp = pos_1_range_min;
    double pos_2_temp = pos_2_range_min;
    while (pos_1_temp <= pos_1_range_max)
    {
        while (pos_2_temp <= pos_2_range_max)
        {
            pos_1.push_back(pos_1_temp);
            pos_2.push_back(pos_2_temp);
            pos_2_temp += step_2;
        }
        pos_2_temp = pos_2_range_min;
        pos_1_temp += step_1;
    }
    RCLCPP_INFO(get_logger(),"The pose number is %ld",pos_1.size());
}

rcm_calibration::~rcm_calibration()
{
}


void rcm_calibration::aimooeCallback(const aimooe_sdk::msg::AimCoord::SharedPtr msg)
{
    if (msg->header.frame_id == "Disk30")
    {
        aimooe_msg.header.frame_id = msg->header.frame_id;
        aimooe_msg.position = msg->position;
        aimooe_msg.orientation = msg->orientation;
        aimooe_msg.mean_error = msg->mean_error;
    }
}



void rcm_calibration::initialization()
{
    auto request = std::make_shared<ft_motor_msg::srv::FTCommand::Request>();
    request->command = "FIX";
    request->motor_pos[0] = 0.0;
    auto result = ftmotor_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result)==rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(get_logger(),"calling initialization success");
    }else
    {
        RCLCPP_ERROR(get_logger(),"calling initialization failed");
    }
}


void rcm_calibration::record()
{
    Point point_temp;
    point_temp.pose[0]=aimooe_msg.position.x;
    point_temp.pose[1]=aimooe_msg.position.y;
    point_temp.pose[2]=aimooe_msg.position.z;
    point_temp.pose[3]=aimooe_msg.orientation.x;
    point_temp.pose[4]=aimooe_msg.orientation.y;
    point_temp.pose[5]=aimooe_msg.orientation.z;
    point_temp.mean_error=aimooe_msg.mean_error;
    
    cali_point_list.push_back(point_temp);
    printf("%lf, %lf, %lf, %lf, %lf, %lf\n",point_temp.pose[0],point_temp.pose[1],point_temp.pose[2],point_temp.pose[3],point_temp.pose[4],point_temp.pose[5]);
}



void rcm_calibration::collection()
{
    auto request = std::make_shared<ft_motor_msg::srv::FTCommand::Request>();
    request->command = "MOV";
    RCLCPP_INFO(get_logger(),"Start point collection...");
    for(long unsigned int i=0;i<pos_1.size();i++)
    {
        request->motor_pos[0] = pos_1[i];
        request->motor_pos[1] = pos_2[i];
        request->motor_vel[0] = 2.0;
        request->motor_vel[1] = 2.0;
        request->motor_tim[0] = 0.0;
        request->motor_tim[1] = 0.0;
        auto result = ftmotor_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->shared_from_this(), result);
        sleep(5);

        record();
    }
    cali_point_list.erase(cali_point_list.begin());
    RCLCPP_INFO(get_logger(),"The collection is complete, and %ld points are saved",cali_point_list.size());
}

void rcm_calibration::least_square_cal()
{
    auto n = cali_point_list.size();

    Eigen::MatrixXd Amat(3*(n-1), 3);
    Eigen::VectorXd Bvec(3*(n-1));
    Eigen::Vector3d rotate_vec_i,rotate_vec_i_1;
    Eigen::Vector3d translate_vec_i;
    Eigen::Matrix3d matrix_error;

    for (long unsigned int i = 0; i<n-1;i++)
    {
        for (int j = 0; j<3;j++)
        {
            rotate_vec_i[j] = cali_point_list[i].pose[j+3];
            rotate_vec_i_1[j] = cali_point_list[i+1].pose[j+3];
            Bvec[3*i+j] = cali_point_list[i+1].pose[j]-cali_point_list[i].pose[j];
        }
        Eigen::AngleAxisd angleaxis_i(rotate_vec_i.norm(),rotate_vec_i/rotate_vec_i.norm());
        Eigen::AngleAxisd angleaxis_i_1(rotate_vec_i_1.norm(),rotate_vec_i_1/rotate_vec_i_1.norm());
        matrix_error = angleaxis_i_1.matrix() - angleaxis_i.matrix();
        for(int j=0;j<3;j++) Amat.row(i*3+j) = matrix_error.row(j);
    }
    //-(a_mat'*a_mat)\(a_mat'*b_mat)
    Eigen::VectorXd x = - (Amat.transpose()*Amat).lu().solve(Amat.transpose()*Bvec);

    //std::cout << "Solution x:\n" << x << std::endl;

    Eigen::Vector3d rcmToAim = {0.0,0.0,0.0};

    for (long unsigned int i = 0; i<n;i++)
    {
        for (int j = 0; j<3;j++)
        {
            rotate_vec_i[j] = cali_point_list[i].pose[j+3];
            translate_vec_i[j] = cali_point_list[i].pose[j];
        }
        Eigen::AngleAxisd angleaxis_i(rotate_vec_i.norm(),rotate_vec_i/rotate_vec_i.norm());
        rcmToAim += (angleaxis_i.matrix()*x + translate_vec_i);
    }
    rcmToAim /= n;
    std::cout << "rcmToAim:\n" << rcmToAim << std::endl;

    //RMSE
    Eigen::Vector3d error;
    double RMSE = 0.0;
    for (long unsigned int i = 0; i<n;i++)
    {
        for (int j = 0; j<3;j++)
        {
            rotate_vec_i[j] = cali_point_list[i].pose[j+3];
            translate_vec_i[j] = cali_point_list[i].pose[j];
        }
        Eigen::AngleAxisd angleaxis_i(rotate_vec_i.norm(),rotate_vec_i/rotate_vec_i.norm());
        error = (angleaxis_i.matrix()*x + translate_vec_i)-rcmToAim;
        RMSE += error.norm();
    }
    RMSE = sqrt(RMSE/n);
    std::cout << "RMSE: " << RMSE << std::endl;

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rcm_calibration>();
    
    //node->initialization();
    node->collection();
    node->least_square_cal();

    rclcpp::spin(node);

    return 0;
}