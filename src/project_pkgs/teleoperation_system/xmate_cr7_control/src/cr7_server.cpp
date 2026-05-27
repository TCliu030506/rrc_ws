#include "../include/xmate_cr7_control/cr7_server.h"

Cr7Server::Cr7Server():Node("cr7_server"), robot("192.168.2.160", "192.168.2.101")
{
    // 初始化机械臂
    init();

    // 创建服务器
    cr7_service = this->create_service<xmate_cr7_msg::srv::Cr7Script>(
        "xmate_cr7_server", 
        std::bind(&Cr7Server::execute_cmd_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
    
}

Cr7Server::~Cr7Server()
{
    if (followThread.joinable()) {
        running = false;
        followThread.join();
    }
    if (rtConThread.joinable()) {
        rtConThread.join();
    }
}

void Cr7Server::init()
{
    // 设置操作模式
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    if (ec) {
        std::cerr << "Failed to set operate mode: " << ec.message() << std::endl;
        return;
    }

    // 设置电源状态
    robot.setPowerState(true, ec);
    if (ec) {
        std::cerr << "Failed to set power state: " << ec.message() << std::endl;
        return;
    }

    // 消除未完成的运动
    robot.moveReset(ec);
    if (ec) {
        std::cerr << "Failed to move reset: " << ec.message() << std::endl;
        return;
    }

    // 调整速度比例
    robot.adjustSpeedOnline(1, ec);
    if (ec) {
        std::cerr << "Failed to adjust speed online: " << ec.message() << std::endl;
        return;
    }

    // 设置默认速度
    robot.setDefaultSpeed(default_speed, ec);
    if (ec) {
        std::cerr << "Failed to set default speed: " << ec.message() << std::endl;
        return;
    }

    // 设置默认转弯区域
    robot.setDefaultZone(30, ec);
    if (ec) {
        std::cerr << "Failed to set default zone: " << ec.message() << std::endl;
        return;
    }

    // 调整加速度为最大
    robot.adjustAcceleration(1.5, 2, ec);
    if (ec) {
        std::cerr << "Failed to adjust acceleration: " << ec.message() << std::endl;
        return;
    }
}

bool Cr7Server::execute_cmd_callback(
    const std::shared_ptr<xmate_cr7_msg::srv::Cr7Script::Request> req,
    const std::shared_ptr<xmate_cr7_msg::srv::Cr7Script::Response> res)
{
    // 清空之前的服务请求
    res->result = "OK";
    // 根据指令执行相应的动作
    if(req->command == "movej"){
        // 解析指令
        std::vector<double> joint_angles = req->pose;
        int speed = req->speed;
        int zone = req->zone;
        // 执行动作
        movej(joint_angles, speed, zone);

    }else if(req->command == "movep"){
        // 解析指令
        std::array<double, 6> target_pos;
        std::copy(req->pose.begin(), req->pose.begin() + std::min(req->pose.size(), target_pos.size()), target_pos.begin());
        int speed = req->speed;
        int zone = req->zone;
        // 执行动作
        movep(target_pos, speed, zone);

    }else if(req->command == "open_rtControl_loop"){
        // 开启实时控制
        open_rtControl_loop();

    }else if(req->command == "rtControl_pos_update"){
        std::array<double, 6> joint_angles;
        std::copy(req->pose.begin(), req->pose.begin() + std::min(req->pose.size(), joint_angles.size()), joint_angles.begin());
        // 更新目标位置
        rtControl_pos_update(joint_angles);
    }
    else if(req->command == "follow_pos_start"){
        // 开始跟随位置
        follow_pos_start();

    }else if(req->command == "follow_pos_stop"){
        // 停止跟随位置
        follow_pos_stop();

    }else if(req->command == "follow_pos_update"){
        std::array<double, 6> joint_angles;
        std::copy(req->pose.begin(), req->pose.begin() + std::min(req->pose.size(), joint_angles.size()), joint_angles.begin());
        int speed = req->speed;
        // 更新跟随位置
        follow_pos_update(joint_angles, speed);

    }else if(req->command == "set_follow_speed"){
        // 设置跟随速度
        int speed = req->speed;
        set_follow_speed(speed);

    }else if(req->command == "readp"){
        std::array<double, 6> current_pos = readp();
        res->pose.assign(current_pos.begin(), current_pos.end());

    }else if(req->command == "readj"){
        std::array<double, 6> current_joint = readj();
        res->joint.assign(current_joint.begin(), current_joint.end());

    }else if(req->command == "get_vel"){
        std::array<double, 6> current_vel = get_vel();
        res->vel.assign(current_vel.begin(), current_vel.end());

    }else if(req->command == "get_joint_vel"){
        std::array<double, 6> current_joint_vel = get_joint_vel();
        res->vel.assign(current_joint_vel.begin(), current_joint_vel.end());

    }else if(req->command == "get_joint_torque"){
        std::array<double, 6> current_joint_torque = get_joint_torque();
        res->torque.assign(current_joint_torque.begin(), current_joint_torque.end());

    }else if(req->command == "get_acc"){
        res->acc = get_acc();
        res->jerk = get_jerk();

    }else if(req->command == "inv_kinematics"){
        std::array<double, 6> pos;
        std::array<double, 6>  jointPos;
        std::copy(req->pose.begin(), req->pose.begin() + std::min(req->pose.size(), pos.size()), pos.begin());
        inv_kinematics(pos, jointPos);
        res->joint.assign(jointPos.begin(), jointPos.end());
        // 测试
        std::cout << res->result << std::endl;

    }else if(req->command == "forward_kinematics"){
        std::array<double, 6> joint_angles;
        std::array<double, 6> pos;
        std::copy(req->pose.begin(), req->pose.begin() + std::min(req->pose.size(), joint_angles.size()), joint_angles.begin());
        forward_kinematics(joint_angles, pos);
        res->pose.assign(pos.begin(), pos.end());
    }

    return true;
}


void Cr7Server::movej(std::vector<double> joint_angles, int speed, int zone)
{
    JointPosition target_joints(joint_angles);
    MoveAbsJCommand movej_cmd(target_joints, speed, zone);
    robot.executeCommand({movej_cmd}, ec);
}

void Cr7Server::movep(std::array<double, 6> target_pos, int speed, int zone)
{
    CartesianPosition target_position(target_pos);
    MoveLCommand movel_cmd(target_position, speed, zone);
    robot.executeCommand({movel_cmd}, ec);
}

void Cr7Server::rtControl_loop()
{
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    rtCon = robot.getRtMotionController().lock();
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
    // 定义回调函数
    std::function<JointPosition()> rtC_loop_callback = [&](){
      JointPosition cmd = {{target_joint[0], target_joint[1], target_joint[2], target_joint[3], target_joint[4], target_joint[5] }};
    //   // 测试
    //   std::cout << "rtControl_loop" << std::endl;
    //   std::cout << "target_joint: " << std::endl;
    //   for (size_t i = 0; i < target_joint.size(); ++i)
    //   {
    //       std::cout << target_joint[i] << ',';
    //   }
    //   std::cout << std::endl;

      return cmd;
    };
    // 设置回调函数
    rtCon->setControlLoop(rtC_loop_callback);
    // 开始轴空间位置控制
    rtCon->startMove(RtControllerMode::jointPosition);
    // 阻塞loop
    rtCon->startLoop(true);
}

void Cr7Server::open_rtControl_loop()
{
    target_joint = robot.jointPos(ec);
    rtConThread= std::thread(&Cr7Server::rtControl_loop, this);
}

void Cr7Server::rtControl_pos_update(std::array<double, 6> joint_angles)
{
    // 更新目标位置
    target_joint = joint_angles;
    // 测试
    std::cout << "rtControl_pos_update" << std::endl;
    std::cout << "target_joint: " << std::endl;
    for (size_t i = 0; i < target_joint.size(); ++i)
    {
        std::cout << target_joint[i] << ',';
    }
    std::cout << std::endl;
}

void Cr7Server::followPosition(xMateRobot &robot, std::array<double, 6> &target_joint, double speed_ratio)
{
    // 切换到实施控制模式
    robot.setRtNetworkTolerance(20, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    rtCon = robot.getRtMotionController().lock();
    if (!rtCon) {
        std::cerr << "获取实时运动控制器失败" << std::endl;
        return;
    }
    // 设置zone=0, 使机械臂不进行转弯
    robot.setDefaultZone(0, ec);

    // 开启跟随位置
    std::thread updater;
    running = true; ///< running state flag
    auto model = robot.model();
    follow_pose.init(robot, model);
    follow_pose.start(target_joint);
    // 测试
    std::cout << "followPositionOpen" << std::endl;

    // 启动一个线程来更新目标位置
    updater = std::thread([&]() 
    {
        while(running) {
            // 更新位置
            follow_pose.setScale(speed_ratio);
            follow_pose.update(target_joint);
            // 等待一段时间
            std::this_thread::sleep_for(std::chrono::milliseconds (20));
            // 测试
            std::cout << "followPositionUpdate" << std::endl;
        }
    });

    while(running);

    follow_pose.stop();
    updater.join();
}

void Cr7Server::follow_pos_start()
{   
    target_joint = robot.jointPos(ec);
    // 创建并启动新线程来运行 followPosition 函数
    followThread = std::thread(&Cr7Server::followPosition, this, std::ref(robot), std::ref(target_joint), speed_ratio);
    // 测试
    std::cout << "follow_pos_start" << std::endl;
}

void Cr7Server::follow_pos_stop()
{
    // 停止跟随位置
    running = false;
}

void Cr7Server::follow_pos_update(std::array<double, 6> joint_angles, int speed)
{
    // 设置速度比例
    set_follow_speed(speed);
    // 更新跟随位置
    target_joint = joint_angles;
    // 测试
    std::cout << "follow_pos_update" << std::endl;
    // std::cout << "target_joint: " << std::endl;
    // for (size_t i = 0; i < target_joint.size(); ++i)
    // {
    //     std::cout << target_joint[i] << ',';
    // }
    // std::cout << std::endl;
    // std::cout << "speed_ratio: " << speed_ratio << std::endl;

}

void Cr7Server::set_follow_speed(int speed)
{
    // 设置跟随速度
    speed_ratio = speed/default_speed;
}

std::array<double, 6> Cr7Server::readp()
{
    // 读取当前位置
    CartesianPosition Position = robot.posture(ct,ec);
    std::array<double, 6> current_pos = {Position.trans[0], Position.trans[1], Position.trans[2], Position.rpy[0], Position.rpy[1], Position.rpy[2]};
    return current_pos;
}

std::array<double, 6> Cr7Server::readj()
{
    // 读取当前位置
    std::array<double, 6> current_joint = robot.jointPos(ec);
    return current_joint;
}

std::array<double, 6> Cr7Server::get_vel()
{
    std::array<double, 6> current_vel ;
    auto model = robot.model();
    current_vel = model.getCartVel(readj(),get_joint_vel());
    return current_vel;
    
}
std::array<double, 6> Cr7Server::get_joint_vel()
{
    // 读取当前关节速度
    std::array<double, 6> current_joint_vel = robot.jointVel(ec);
    return current_joint_vel;
}

std::array<double, 6> Cr7Server::get_joint_torque()
{
    // 读取当前关节力矩
    std::array<double, 6> current_joint_torque = robot.jointTorque(ec);
    return current_joint_torque;
}

double Cr7Server::get_acc()
{
    double current_acc;
    double current_jerk;
    robot.getAcceleration(current_acc, current_jerk, ec);
    return current_acc;
}
double Cr7Server::get_jerk()
{
    double current_acc;
    double current_jerk;
    robot.getAcceleration(current_acc, current_jerk, ec);
    return current_jerk;
}

void Cr7Server::inv_kinematics(std::array<double, 6> &Pos, std::array<double, 6> &jntPos)
{
    std::array<double, 16UL> targetPositionMatrix;
    Utils::postureToTransArray(Pos,targetPositionMatrix);
    auto model = robot.model();
    model.getJointPos(targetPositionMatrix,0,readj(),jntPos);
}

void Cr7Server::forward_kinematics(std::array<double, 6> &jntPos, std::array<double, 6> &Pos)
{
    std::array<double, 16UL> targetPositionMatrix;
    auto model = robot.model();
    targetPositionMatrix = model.getCartPose(jntPos);
    Utils::transArrayToPosture(targetPositionMatrix,Pos);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Cr7Server>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}





