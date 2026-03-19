#include "../../ur5_kinematics/include/ur5_kinematics.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ur5_msg/msg/robot_state.hpp"
#include "ur5_kinematics/msg/joint_target.hpp"

	//关节表示
	//-10 -80 -100 -150 -100 -100
	//-10 -80 -100 -10 90 90
class CustomNode : public rclcpp::Node {
    public:
        CustomNode() : Node("custom_node") {
            
         

            joint_target_msg_ = std::make_shared<ur5_kinematics::msg::JointTarget>();// 新增关节目标消息

            // 创建发布者，发布 ur5_kinematics::msg::JointTarget 消息到 "joint_target"
            publisher_ = this->create_publisher<ur5_kinematics::msg::JointTarget>("joint_target", 10);

           

            // 创建订阅者，订阅 "robotstate" 上的 ur5_msg::msg::RobotState消息，回调函数为 handle_message
           subscriber_ = this->create_subscription<ur5_msg::msg::RobotState>(
                "robotstate", 10, std::bind(&CustomNode::handle_message, this, std::placeholders::_1));

            
            
        }

        double joint_base[6];

        void update_and_publish_joint_target(double* joint_target){
                for (int i = 0; i < 1000; i++) {
                    joint_target_msg_->joint_target[i] = joint_target[i];
                }
                publisher_->publish(*joint_target_msg_); // 发布消息
            }

        void update_joint_base(rclcpp::Logger logger);

            

    private:
        rclcpp::Publisher<ur5_kinematics::msg::JointTarget>::SharedPtr publisher_;
        rclcpp::Subscription<ur5_msg::msg::RobotState>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        ur5_kinematics::msg::JointTarget::SharedPtr joint_target_msg_;
        double joint[6];    
        // 发布消息的回调函数
        void publish_message();

        
        // 订阅消息的回调函数
        void handle_message(const ur5_msg::msg::RobotState::SharedPtr msg);
    


};


void CustomNode::handle_message(const ur5_msg::msg::RobotState::SharedPtr msg){
    //RCLCPP_INFO(logger, "Received message:");
    //RCLCPP_INFO(logger, "Header:");
    //RCLCPP_INFO(logger, "  Stamp: %d.%09d", msg->header.stamp.sec, msg->header.stamp.nanosec);
    //RCLCPP_INFO(logger, "  Frame ID: %s", msg->header.frame_id.c_str());
    //RCLCPP_INFO(logger, "Joint Positions:");
    for (size_t i = 0; i < 6; ++i) {
        //RCLCPP_INFO(logger, "  Position of joint %zu: %f", i, msg->joint_pos[i]);
        joint[i] = msg->joint_pos[i];
    }
}

void CustomNode::update_joint_base(rclcpp::Logger logger){
    RCLCPP_INFO(logger, "joint_base UPDATE!");
    for (size_t i = 0; i < 6; ++i) {
        joint_base[i] = joint[i];
    }
}


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    // 创建一个节点
    auto node = std::make_shared<rclcpp::Node>("my_node");

    // 创建一个 logger 对象
    auto logger = node->get_logger();


    auto custom_node = std::make_shared<CustomNode>();

    //定义初始量
    double target_p[3];
    double initial_p[3];
    double middle_p[3];
    double joint_target[6];
    double joint_initial[6];
    double joint_middle[6];
    double joint_target_storage[1000];

    for(int i = 0; i < 1000; ++i) {
        joint_target_storage[i] = 0;
    }
    //double joint_base[6];

    int count = 0;
    int type;
    int continue_flag = 0;

    //URposition类的实例化
    URposition ur_position;

    // 创建一个 Eigen::Isometry3d 对象
    EIso3d mat_initial = Eigen::Isometry3d::Identity();
    EIso3d mat_middle = Eigen::Isometry3d::Identity();
    EIso3d mat_target = Eigen::Isometry3d::Identity();
    // 将变换矩阵设置为单位矩阵（无旋量）
    mat_initial.setIdentity();
    mat_middle.setIdentity();

    //设置运动之初六个关节的位置
    //joint_base[0]=1.57;
    //joint_base[1]=1.57;
    //joint_base[2]=0;
    //joint_base[3]=-1.57;
    //joint_base[4]=-1.57;
    //joint_base[5]=0;

    //用户输入获取直线运动的起始点坐标

    std::cout << "输入直线运动起点位姿"  << std::endl;
    std::cout << "请选择输入运动起点位姿的方式:角轴表示(1),欧拉角表示(2),关节角度表示(3),四元数表示(4)";
    std::cin >> type;
    ur_position.select_mat(mat_initial, type);

    std::cout << "输入直线运动终点位姿"  << std::endl;
    std::cout << "请选择输入运动终点位姿的方式:角轴表示(1),欧拉角表示(2),关节角度表示(3),四元数表示(4)";
    std::cin >> type;
    ur_position.select_mat(mat_target, type);

    mat_middle = mat_initial;

    //将机械臂运动至初始位置
    ur_position.dhinvkinematics(mat_initial, joint_initial, custom_node->joint_base);
    
    //发布关节角度使机械臂运动至直线起点-----------此处用打印出关节角度代替
    std::cout << "直线运动初始位置关节角度依次为" << std::endl;
    for (int i = 0; i < 6; ++i) {
        std::cout << joint_initial[i]*180/PI << std::endl;
    }

    //判断是否继续执行
    std::cout << "请确定是否需要继续执行运动: (继续运动请输入1,终止运动请输入0) ";
    std::cin >> continue_flag;

    if (continue_flag == 0) return 0;

    custom_node->update_joint_base(logger);// 订阅关节目标消息
        
    //将中间位置初始值赋值为运动起始点
    middle_p[0] = mat_initial(0,3);
    middle_p[1] = mat_initial(1,3);
    middle_p[2] = mat_initial(2,3);

    initial_p[0] = mat_initial(0,3);
    initial_p[1] = mat_initial(1,3);
    initial_p[2] = mat_initial(2,3);

    target_p[0] = mat_target(0,3);
    target_p[1] = mat_target(1,3);
    target_p[2] = mat_target(2,3);
    
    
    //每次运动的起点关节角度为上一次运动的目标关节角度，初始值为直线起点关节角度
    for (int i = 0; i < 6; ++i) {
        joint_middle[i] = joint_initial[i];
    }

    //将运动分为100段运动
    int segments = 100;
    double stepX = (target_p[0] - initial_p[0] ) / segments;
    double stepY = (target_p[1] - initial_p[1]) / segments;
    double stepZ = (target_p[2] - initial_p[2]) / segments;
    
    for (int i = 0; i < segments; ++i) {
        //求出第i段直线终点坐标
        target_p[0] =  middle_p[0] + stepX;
        target_p[1] =  middle_p[1] + stepY;
        target_p[2] =  middle_p[2] + stepZ;
        //求出第i段运动终点的关节角度
        Eigen::Vector3d middle_translation(target_p[0],target_p[1],target_p[2]);      //起点的(X,Y,Z)
        mat_middle.translation() = middle_translation;
        ur_position.dhinvkinematics(mat_middle, joint_target, joint_middle);
        std::cout << "直线运动第" << i+1 << "个目标点对应的关节角度依次为：" <<std::endl;
        for (int j = 0; j < 6; ++j) {
            std::cout << joint_target[j]*180/PI << std::endl;
        }

        for (int j = 0; j < 6; ++j) {
            joint_target_storage[count+j] = joint_target[j];
        }
        count = count + 6;

        //custom_node->update_and_publish_joint_target(joint_target);


        //将终点关节角度作为下一段运动的初始关节角度
        for (int j = 0; j < 6; ++j) {
            joint_middle[j] = joint_target[j];
        }
        
        middle_p[0] = target_p[0];
        middle_p[1] = target_p[1];
        middle_p[2] = target_p[2];
    }

    std::cout << "直线运动完成" << std::endl;

    while(true){
        custom_node->update_and_publish_joint_target(joint_target_storage);
    }

    rclcpp::shutdown();
	return 0;
}





 
