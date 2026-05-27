#include "../include/xmate_cr7_script/cr7_server.h"
#include <regex>
#include <sstream>

// 辅助：解析参数字符串为map
std::map<std::string, std::string> Cr7Server::parse_params(const std::string& param_str) {
    std::map<std::string, std::string> param_map;
    std::stringstream ss(param_str);
    std::string item;
    while (std::getline(ss, item, ',')) {
        auto pos = item.find(':');
        if (pos != std::string::npos) {
            std::string key = item.substr(0, pos);
            std::string value = item.substr(pos + 1);
            // 去除首尾空格
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            param_map[key] = value;
        }
    }
    return param_map;
}

// 新增：字符串指令解析与分发
bool Cr7Server::parse_and_execute(const std::string& command, std::string& result) {
    std::smatch match;
    // 匹配格式：cmd(param1:val1,param2:val2,...)
    if (!std::regex_match(command, match, std::regex(R"((\w+)\((.*)\))"))) {
        result = "Parse command failed";
        return false;
    }
    std::string cmd_name = match[1];
    std::string params = match[2];
    auto param_map = parse_params(params);
    // //测试是否正确解析
    // for (const auto& p : param_map) {
    //     RCLCPP_INFO(this->get_logger(), "Param: %s = %s", p.first.c_str(), p.second.c_str());
    // }

    try {
        if (cmd_name == "movej") {
            std::vector<double> joints(6, 0.0);
            for (int i = 0; i < 6; ++i)
                joints[i] = std::stod(param_map["joint" + std::to_string(i+1)]);
            int speed = std::stoi(param_map["speed"]);
            int zone = std::stoi(param_map["zone"]);
            movej(joints, speed, zone, result);
        } else if (cmd_name == "movep") {
            std::array<double, 6> pos{};
            const char* names[6] = {"x","y","z","rx","ry","rz"};
            for (int i = 0; i < 6; ++i)
                pos[i] = std::stod(param_map[names[i]]);
            int speed = std::stoi(param_map["speed"]);
            int zone = std::stoi(param_map["zone"]);
            movep(pos, speed, zone, result);
        } else if (cmd_name == "movel") {
            std::array<double, 6> pos{};
            const char* names[6] = {"x","y","z","rx","ry","rz"};
            for (int i = 0; i < 6; ++i)
                pos[i] = std::stod(param_map[names[i]]);
            int speed = std::stoi(param_map["speed"]);
            int zone = std::stoi(param_map["zone"]);
            movel(pos, speed, zone, result);
        } else if (cmd_name == "stop") {
            stop(result);
        } else if (cmd_name == "rtControl_jointpos_update") {
            std::array<double, 6> joints{};
            for (int i = 0; i < 6; ++i)
                joints[i] = std::stod(param_map["joint" + std::to_string(i+1)]);
            rtControl_jointpos_update(joints, result);
        } else if (cmd_name == "rtControl_cartesianpos_update") {
            std::array<double, 6> pos{};
            const char* names[6] = {"x","y","z","rx","ry","rz"};
            for (int i = 0; i < 6; ++i)
                pos[i] = std::stod(param_map[names[i]]);
            rtControl_cartesianpos_update(pos, result);
        } else if (cmd_name == "follow_pos_update") {
            std::array<double, 6> joints{};
            for (int i = 0; i < 6; ++i)
                joints[i] = std::stod(param_map["joint" + std::to_string(i+1)]);
            int speed = std::stoi(param_map["speed"]);
            follow_pos_update(joints, speed, result);
        } else if (cmd_name == "set_follow_speed") {
            int speed = std::stoi(param_map["speed"]);
            set_follow_speed(speed, result);
        } else if (cmd_name == "calibrateForceSensor") {
            bool all_axes = (param_map["all_axes"] == "true");
            int axis_index = std::stoi(param_map["axis_index"]);
            calibrateForceSensor(all_axes, axis_index, result);
        } else if (cmd_name == "getEndTorque") {
            std::string ref_type_str = param_map["ref_type"];
            std::array<double, 6> joint_torque_measured;
            std::array<double, 6> external_torque_measured;
            std::array<double, 3> cart_torque;
            std::array<double, 3> cart_force;
            getEndTorque(ref_type_str, joint_torque_measured, external_torque_measured, cart_torque, cart_force);
            std::ostringstream oss;
            oss << "joint_torque:";
            for (int i = 0; i < 6; ++i) oss << joint_torque_measured[i] << (i<5?",":"");
            oss << ";external_torque:";
            for (int i = 0; i < 6; ++i) oss << external_torque_measured[i] << (i<5?",":"");
            oss << ";cart_torque:";
            for (int i = 0; i < 3; ++i) oss << cart_torque[i] << (i<2?",":"");
            oss << ";cart_force:";
            for (int i = 0; i < 3; ++i) oss << cart_force[i] << (i<2?",":"");
            result = oss.str();
        } else if (cmd_name == "inv_kinematics") {
            std::array<double, 6> pos{};
            const char* names[6] = {"x","y","z","rx","ry","rz"};
            for (int i = 0; i < 6; ++i)
                pos[i] = std::stod(param_map[names[i]]);
            std::array<double, 6> jointPos{};
            inv_kinematics(pos, jointPos);
            std::ostringstream oss;
            oss << "joint:";
            for (int i = 0; i < 6; ++i) oss << jointPos[i] << (i<5?",":"");
            result = oss.str();
        } else if (cmd_name == "forward_kinematics") {
            std::array<double, 6> joints{};
            for (int i = 0; i < 6; ++i)
                joints[i] = std::stod(param_map["joint" + std::to_string(i+1)]);
            std::array<double, 6> pos{};
            forward_kinematics(joints, pos);
            std::ostringstream oss;
            oss << "pose:";
            for (int i = 0; i < 6; ++i) oss << pos[i] << (i<5?",":"");
            result = oss.str();
        } else if (cmd_name == "readp") {
            std::string ref_type_str = param_map["ref_type"];
            std::array<double, 6> pos = readp(ref_type_str);
            std::ostringstream oss;
            oss << "pose:";
            for (int i = 0; i < 6; ++i) oss << pos[i] << (i<5?",":"");
            result = oss.str();
        } else if (cmd_name == "readj") {
            std::array<double, 6> joints = readj();
            std::ostringstream oss;
            oss << "joint:";
            for (int i = 0; i < 6; ++i) oss << joints[i] << (i<5?",":"");
            result = oss.str();
        } else if (cmd_name == "get_vel") {
            std::array<double, 6> vel = get_vel();
            std::ostringstream oss;
            oss << "vel:";
            for (int i = 0; i < 6; ++i) oss << vel[i] << (i<5?",":"");
            result = oss.str();
        } else if (cmd_name == "get_joint_vel") {
            std::array<double, 6> joint_vel = get_joint_vel();
            std::ostringstream oss;
            oss << "joint_vel:";
            for (int i = 0; i < 6; ++i) oss << joint_vel[i] << (i<5?",":"");
            result = oss.str();
        } else if (cmd_name == "get_acc") {
            double acc = get_acc();
            result = "acc:" + std::to_string(acc);
        } else if (cmd_name == "get_jerk") {
            double jerk = get_jerk();
            result = "jerk:" + std::to_string(jerk);
        } else if (cmd_name == "get_joint_torque") {
            std::array<double, 6> joint_torque = get_joint_torque();
            std::ostringstream oss;
            oss << "joint_torque:";
            for (int i = 0; i < 6; ++i) oss << joint_torque[i] << (i<5?",":"");
            result = oss.str();
        } else if (cmd_name == "follow_pos_start") {
            follow_pos_start();
            result = "OK";
        } else if (cmd_name == "follow_pos_stop") {
            follow_pos_stop();
            result = "OK";
        } else if (cmd_name == "open_rtControl_loop") {
            std::string rt_type = param_map["rt_type"];
            open_rtControl_loop(rt_type);
            result = "OK";
        } else if (cmd_name == "fcInit") {
            std::string frame_type_str = param_map["frame_type"];
            FrameType frame_type = FrameType::world;
            if (frame_type_str == "world") frame_type = FrameType::world;
            else if (frame_type_str == "wobj") frame_type = FrameType::wobj;
            else if (frame_type_str == "tool") frame_type = FrameType::tool;
            else if (frame_type_str == "base") frame_type = FrameType::base;
            else if (frame_type_str == "flange") frame_type = FrameType::flange;
            else result = "Unknown frame type: " + frame_type_str;
            fcInit(frame_type, ec);
            result = "OK";
        } else if (cmd_name == "fcStart") {
            fcStart(ec);
            result = "OK";
        } else if (cmd_name == "fcStop") {
            fcStop(ec);
            result = "OK";
        } else if (cmd_name == "setControlType") {
            int control_type = std::stoi(param_map["control_type"]);
            setControlType(control_type, ec);
            result = "OK";
        } else if (cmd_name == "setLoad") {
            double mass = std::stod(param_map["mass"]);
            std::array<double, 3> cog = {
                std::stod(param_map["cog_x"]),
                std::stod(param_map["cog_y"]),
                std::stod(param_map["cog_z"])
            };
            std::array<double, 3> inertia = {
                std::stod(param_map["inertia_x"]),
                std::stod(param_map["inertia_y"]),
                std::stod(param_map["inertia_z"])
            };
            setLoad(mass, cog, inertia, ec);
            result = "OK";
        } else if (cmd_name == "setCartesianStiffness") {
            std::array<double, 6> stiffness = {
                std::stod(param_map["kx"]),
                std::stod(param_map["ky"]),
                std::stod(param_map["kz"]),
                std::stod(param_map["krx"]),
                std::stod(param_map["kry"]),
                std::stod(param_map["krz"])
            };
            setCartesianStiffness(stiffness, ec);
            result = "OK";
        } else if (cmd_name == "setCartesianNullspaceStiffness")
        {
            double stiffness = std::stod(param_map["stiffness"]);
            setCartesianNullspaceStiffness(stiffness, ec);
            result = "OK";  
        } else if (cmd_name == "setCartesianDesiredForce") {
            std::array<double, 6> force = {
                std::stod(param_map["fx"]),
                std::stod(param_map["fy"]),
                std::stod(param_map["fz"]),
                std::stod(param_map["frx"]),
                std::stod(param_map["fry"]),
                std::stod(param_map["frz"])
            };
            setCartesianDesiredForce(force, ec);
            result = "OK";
        } else if (cmd_name == "setSineOverlay") {
            int line_dir = std::stoi(param_map["line_dir"]);
            double amplify = std::stod(param_map["amplify"]);
            double frequency = std::stod(param_map["frequency"]);
            double bias = std::stod(param_map["bias"]);
            double phase = std::stod(param_map["phase"]);
            setSineOverlay(line_dir, amplify, frequency, bias, phase, ec);
            result = "OK";
        } else if (cmd_name == "setLissajousOverlay")
        {
            int plane = std::stoi(param_map["plane"]);
            double amplify_one = std::stod(param_map["amplify_one"]);
            double frequency_one = std::stod(param_map["frequency_one"]);
            double amplify_two = std::stod(param_map["amplify_two"]);
            double frequency_two = std::stod(param_map["frequency_two"]);
            double phase_diff = std::stod(param_map["phase_diff"]);
            setLissajousOverlay(plane, amplify_one, frequency_one, amplify_two, frequency_two, phase_diff, ec);
            result = "OK";
        } else if (cmd_name == "startOverlay")
        {
            startOverlay(ec);
            result = "OK";
        } else if (cmd_name == "stopOverlay")
        {
            stopOverlay(ec);
            result = "OK";
        } else if (cmd_name == "pauseOverlay")
        {
            restartOverlay(ec);
            result = "OK";
        } else if (cmd_name == "restartOverlay")
        {
            restartOverlay(ec);
            result = "OK";
        } else if (cmd_name == "setForceCondition")
        {
            std::array<double, 6> range = {
                std::stod(param_map["X_min"]),
                std::stod(param_map["X_max"]),
                std::stod(param_map["Y_min"]),
                std::stod(param_map["Y_max"]),
                std::stod(param_map["Z_min"]),
                std::stod(param_map["Z_max"])
            };
            bool isInside = (param_map["isInside"] == "true");
            double timeout = std::stod(param_map["timeout"]);
            setForceCondition(range, isInside, timeout, ec);
            result = "OK";
        } else if (cmd_name == "setTorqueCondition")
        {
            std::array<double, 6> range = {
                std::stod(param_map["X_min"]),
                std::stod(param_map["X_max"]),
                std::stod(param_map["Y_min"]),
                std::stod(param_map["Y_max"]),
                std::stod(param_map["Z_min"]),
                std::stod(param_map["Z_max"])
            };
            bool isInside = (param_map["isInside"] == "true");
            double timeout = std::stod(param_map["timeout"]);
            setTorqueCondition(range, isInside, timeout, ec);
            result = "OK";
        } else if (cmd_name == "setPoseBoxCondition")
        {
            std::array<double, 6> frame = {
                std::stod(param_map["X"]),
                std::stod(param_map["Y"]),
                std::stod(param_map["Z"]),
                std::stod(param_map["Rx"]),
                std::stod(param_map["Ry"]),
                std::stod(param_map["Rz"])
            };
            std::array<double, 6> box = {
                std::stod(param_map["X_start"]),
                std::stod(param_map["X_end"]),
                std::stod(param_map["Y_start"]),
                std::stod(param_map["Y_end"]),
                std::stod(param_map["Z_start"]),
                std::stod(param_map["Z_end"])
            };
            bool isInside = (param_map["isInside"] == "true");
            double timeout = std::stod(param_map["timeout"]);
            setPoseBoxCondition(frame, box, isInside, timeout, ec);
            result = "OK";
        } else if (cmd_name == "waitCondition")
        {
            waitCondition(ec);
            result = "OK";
        } else if (cmd_name == "fcMonitor")
        {
            bool enable = (param_map["enable"] == "true");
            fcMonitor(enable, ec);
            result = "OK";
        } else if (cmd_name == "setCartesianMaxVel")
        {
            std::array<double, 6> velocity = {
                std::stod(param_map["X"]),
                std::stod(param_map["Y"]),
                std::stod(param_map["Z"]),
                std::stod(param_map["Rx"]),
                std::stod(param_map["Ry"]),
                std::stod(param_map["Rz"])
            };
            setCartesianMaxVel(velocity, ec);
            result = "OK";
        } else {
            result = "Unknown command: " + cmd_name;
            return false;
        }
    } catch (const std::exception& e) {
        result = std::string("Exception: ") + e.what();
        return false;
    }

    if (result.empty()) result = "OK";

    return true;
}

Cr7Server::Cr7Server():Node("cr7_server"), robot("192.168.2.160", "192.168.2.101"),fc(robot.forceControl())
{
    // 初始化机械臂
    init();

    // 创建服务器
    cr7_service = this->create_service<coordinate::srv::StringScript>(
        "cr7_script", 
        std::bind(&Cr7Server::execute_cmd_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    RCLCPP_INFO(this->get_logger(), "Cr7 Server is ready to receive commands.");
    
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

    // 设置基坐标系为当前的真实世界坐标系
    const std::array<double, 6> world_frame = {0.0, 0.0, 0.0, 0.0, M_PI/6, 0.0};
    robot.setBaseFrame(world_frame, ec);
    if (ec) {
        std::cerr << "Failed to set base frame: " << ec.message() << std::endl;
        return;
    }
}

// 服务回调
void Cr7Server::execute_cmd_callback(
    const std::shared_ptr<coordinate::srv::StringScript::Request> req,
    std::shared_ptr<coordinate::srv::StringScript::Response> res)
{
    // 测试请求是否到达
    RCLCPP_INFO(this->get_logger(), "Received command: %s", req->command.c_str());

    // 统一用字符串指令解析与分发
    parse_and_execute(req->command, res->result);

    // 打印响应结果
    RCLCPP_INFO(this->get_logger(), "Response result: %s", res->result.c_str());
}

// 运动停止说明：规划停止机器人运动后不断电, 停在原始路径上
void Cr7Server::stop(std::string& result)
{
    robot.stop(ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (int i = 0; i < 200; ++i) {
        auto st = robot.operationState(ec);
        if (st == rokae::OperationState::idle || st == rokae::OperationState::unknown) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    robot.moveReset(ec);
    if (ec) {
        result = "Failed to stop and reset movement: " + ec.message();
    } else {
        result = "OK";
    }
}

void Cr7Server::movej(const std::vector<double>& joint_angles, int speed, int zone, std::string& result)
{
    JointPosition target_joints(joint_angles);
    MoveAbsJCommand movej_cmd(target_joints, speed, zone);
    robot.executeCommand({movej_cmd}, ec);
    if (ec) {
        result = "movej failed: " + ec.message();
    } else {
        result = "OK";
    }
}

void Cr7Server::movep(const std::array<double, 6>& target_pos, int speed, int zone, std::string& result)
{
    CartesianPosition target_position(target_pos);
    MoveJCommand movej_cmd(target_position, speed, zone);
    robot.executeCommand({movej_cmd}, ec);
    if (ec) {
        result = "movep failed: " + ec.message();
    } else {
        result = "OK";
    }
}

void Cr7Server::movel(const std::array<double, 6>& target_pos, int speed, int zone, std::string& result)
{
    CartesianPosition target_position(target_pos);
    MoveLCommand movel_cmd(target_position, speed, zone);
    robot.executeCommand({movel_cmd}, ec);
    if (ec) {
        result = "movel failed: " + ec.message();
    } else {
        result = "OK";
    }
}

void Cr7Server::rtControl_jointpos_update(const std::array<double, 6>& joint_angles, std::string& result)
{
    target_joint = joint_angles;
    result = "OK";
}

void Cr7Server::rtControl_cartesianpos_update(const std::array<double, 6>& cartesian_pos, std::string& result)
{
    target_pose = cartesian_pos;
    result = "OK";
}

void Cr7Server::follow_pos_update(const std::array<double, 6>& joint_angles, int speed, std::string& result)
{
    set_follow_speed(speed, result);
    target_joint = joint_angles;
    result = "OK";
}

void Cr7Server::set_follow_speed(int speed, std::string& result)
{
    speed_ratio = speed / default_speed;
    result = "OK";
}

void Cr7Server::rtControl_loop(string rt_type)
{
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    rtCon = robot.getRtMotionController().lock();
    robot.stopReceiveRobotState();
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
    // robot.startReceiveRobotState(std::chrono::milliseconds(1),
    //                              {tcpPoseAbc_m, tauExt_inBase, tcpVel_m});
    // 根据选择的实时控制类型，设置相应的回调函数
    if(rt_type == "jointPosition"){
        std::function<JointPosition()> rtC_loop_callback = [&](){
            JointPosition cmd = {{target_joint[0], target_joint[1], target_joint[2], target_joint[3], target_joint[4], target_joint[5] }};
            return cmd;
          };
        // 设置回调函数
        rtCon->setControlLoop(rtC_loop_callback);
        // 开始轴空间位置控制
        rtCon->startMove(RtControllerMode::jointPosition);
    }
    else if(rt_type == "cartesianPosition"){
        std::function<CartesianPosition()> rtC_loop_callback = [&](){
            // CartesianPosition cmd = {{target_pos[0], target_pos[1], target_pos[2], target_pos[3], target_pos[4], target_pos[5] }};
            CartesianPosition cmd ;
            std::array<double, 16> target_pose_16;
            Utils::postureToTransArray(target_pose,target_pose_16);
            cmd.pos = target_pose_16;
            return cmd;
          };
        // 设置回调函数
        rtCon->setControlLoop(rtC_loop_callback);
        // 开始笛卡尔空间位置控制
        rtCon->startMove(RtControllerMode::cartesianPosition);
        // 测试
        std::cout << "rtControl_loop_cartesianPosition START" << std::endl;
    }
    
    // 阻塞loop
    rtCon->startLoop(true);
}

void Cr7Server::open_rtControl_loop(string rt_type)
{
    target_joint = robot.jointPos(ec);
    target_pose = robot.posture(rokae::CoordinateType::flangeInBase,ec);
    rtConThread= std::thread(&Cr7Server::rtControl_loop, this, rt_type);
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
            std::this_thread::sleep_for(std::chrono::milliseconds (1));
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

std::array<double, 6> Cr7Server::readp(std::string& ref_type_str)
{
    // 读取当前位置
    CartesianPosition Position = robot.posture(ct,ec);
    std::array<double, 6> current_pos = {Position.trans[0], Position.trans[1], Position.trans[2], Position.rpy[0], Position.rpy[1], Position.rpy[2]};
        
    if (ref_type_str == "base") {
        return current_pos;
    } else if (ref_type_str == "world") {
        // 将当前位姿从基坐标系转换到世界坐标系
        // 读取当基坐标系相对于世界坐标系的位姿[x,y,z,rx,ry,rz]
        std::array<double, 6>  base_in_world = robot.baseFrame(ec);
        // 计算转换矩阵[4x4]
        std::array<double, 16> base_in_world_matrix;
        Utils::postureToTransArray(base_in_world, base_in_world_matrix);
        // 当前位姿转换为矩阵形式[4x4]
        std::array<double, 16> current_pos_matrix;
        Utils::postureToTransArray(current_pos, current_pos_matrix);
        // 转换当前位置到世界坐标系[4x4]
        // 计算当前位置在世界坐标系下的变换矩阵
        std::array<double, 16> current_pos_in_world_matrix;
        // 末端在世界坐标系下的变换 = 基坐标系在世界坐标系下的变换 * 末端在基坐标系下的变换
        matrixMultiply(base_in_world_matrix, current_pos_matrix, current_pos_in_world_matrix);
        // 转换回6维位姿[x,y,z,rx,ry,rz]
        Utils::transArrayToPosture(current_pos_in_world_matrix, current_pos);
        return current_pos;

    } else {
        return current_pos;
    }

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

// 校准力传感器（关节力传感器的零点校准）无力学辨识！
// all_axes: 是否校准所有轴(true:校准所有轴, false:校准指定轴)
// axis_index: 校准的轴索引，仅在单轴时有效(范围[0-5])
void Cr7Server::calibrateForceSensor(bool all_axes, int axis_index, std::string& result)
{
    robot.calibrateForceSensor(all_axes, axis_index, ec);
    if (ec) {
        result = "Calibration failed: " + ec.message();
    } else {
        result = "Calibration successful";
    }
}

// 读取当前关节力矩
std::array<double, 6> Cr7Server::get_joint_torque()
{
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

/**
* @brief 获取当前力矩信息
* @param[in] ref_type 力矩相对的参考系：
*     1) FrameType::world - 末端相对世界坐标系的力矩信息
*     2) FrameType::flange - 末端相对于法兰盘的力矩信息
*     3) FrameType::tool - 末端相对于TCP点的力矩信息
* @param[out] joint_torque_measured 轴空间测量力信息,力传感器测量到的各轴所受力矩, 单位Nm
* @param[out] external_torque_measured 轴空间外部力信息，控制器根据机器人模型和测量力计算出的各轴所受力矩信息, 单位Nm
* @param[out] cart_torque 笛卡尔空间各个方向[X, Y, Z]受到的力矩, 单位Nm
* @param[out] cart_force 笛卡尔空间各个方向[X, Y, Z]受到的力, 单位N
* @param[out] ec 错误码
*/
void Cr7Server::getEndTorque(string ref_type_str, std::array<double, 6> &joint_torque_measured, std::array<double, 6> &external_torque_measured,
        std::array<double, 3> &cart_torque, std::array<double, 3> &cart_force)
{
    FrameType ref_type;
    if (ref_type_str == "world") {
        ref_type = FrameType::world;
    } else if (ref_type_str == "flange") {
        ref_type = FrameType::flange;
    } else if (ref_type_str == "tool") {
        ref_type = FrameType::tool;
    } else {
        std::cerr << "Invalid reference type: " << ref_type_str << std::endl;
        return;
    }
    fc.getEndTorque(ref_type, joint_torque_measured, external_torque_measured, cart_torque, cart_force, ec);
    if (ec) {
        std::cerr << "Failed to get end torque: " << ec.message() << std::endl;
        return;
    }
    return;
}

void Cr7Server::fcInit(FrameType frame_type, std::error_code &ec)
{
    fc.fcInit(frame_type, ec);
}

void Cr7Server::fcStart(std::error_code &ec)
{
    fc.fcStart(ec);
}

void Cr7Server::fcStop(std::error_code &ec)
{
    fc.fcStop(ec);
}

void Cr7Server::setControlType(int type, std::error_code &ec)
{
    fc.setControlType(type, ec);
}

void Cr7Server::setLoad(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia, error_code &ec)
{
    Load load(m, cog, inertia);
    fc.setLoad(load, ec);
}

void Cr7Server::setCartesianStiffness(const std::array<double, 6> &stiffness, error_code &ec)
{
    fc.setCartesianStiffness(stiffness, ec);
}

void Cr7Server::setCartesianNullspaceStiffness(double stiffness, error_code &ec)
{
    fc.setCartesianNullspaceStiffness(stiffness, ec);
}

void Cr7Server::setCartesianDesiredForce(const std::array<double, 6> &value, error_code &ec)
{
    fc.setCartesianDesiredForce(value, ec);
}

void Cr7Server::setSineOverlay(int line_dir, double amplify, double frequency, double phase, double bias, error_code &ec)
{
    fc.setSineOverlay(line_dir, amplify, frequency, phase, bias, ec);
}

void Cr7Server::setLissajousOverlay(int plane, double amplify_one, double frequency_one,
    double amplify_two, double frequency_two, double phase_diff, error_code &ec)
{
    fc.setLissajousOverlay(plane, amplify_one, frequency_one,
        amplify_two, frequency_two, phase_diff, ec);
}

void Cr7Server::startOverlay(error_code &ec)
{
    fc.startOverlay(ec);
}

void Cr7Server::stopOverlay(error_code &ec)
{
    fc.stopOverlay(ec);
}

void Cr7Server::pauseOverlay(error_code &ec)
{
    fc.pauseOverlay(ec);
}

void Cr7Server::restartOverlay(error_code &ec)
{
    fc.restartOverlay(ec);
}

void Cr7Server::setForceCondition(const std::array<double, 6> &range, bool isInside, double timeout, error_code &ec)
{
    fc.setForceCondition(range, isInside, timeout, ec);
}

void Cr7Server::setTorqueCondition(const std::array<double, 6> &range, bool isInside, double timeout, error_code &ec)
{
    fc.setTorqueCondition(range, isInside, timeout, ec);
}

void Cr7Server::setPoseBoxCondition(const std::array<double, 6> &frame, const std::array<double, 6> &box, bool isInside, double timeout, error_code &ec)
{
    fc.setPoseBoxCondition(frame, box, isInside, timeout, ec);
}

void Cr7Server::waitCondition(error_code &ec)
{
    fc.waitCondition(ec);
}

void Cr7Server::fcMonitor(bool enable, error_code &ec)
{
    fc.fcMonitor(enable, ec);
}

void Cr7Server::setCartesianMaxVel(const std::array<double, 6> &velocity, error_code &ec)
{
    fc.setCartesianMaxVel(velocity, ec);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Cr7Server>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}





