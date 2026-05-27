#include "../include/xmate_cr7_script/cr7_script.h"
#include <sstream>

namespace cr7_script {

// 辅助：参数拼接
std::string ScriptCmd::join_params(const std::vector<std::pair<std::string, std::string>>& params) {
    std::ostringstream oss;
    for (size_t i = 0; i < params.size(); ++i) {
        oss << params[i].first << ":" << params[i].second;
        if (i < params.size() - 1) oss << ",";
    }
    return oss.str();
}

// ScriptCmd 构造与析构
ScriptCmd::ScriptCmd() {}
ScriptCmd::~ScriptCmd() {}

// movej(joint1:...,joint2:...,...,speed:...,zone:...)
std::string ScriptCmd::build_movej(const std::vector<double>& joint_angles, int speed, int zone) {
    std::vector<std::pair<std::string, std::string>> params;
    for (size_t i = 0; i < joint_angles.size(); ++i) {
        params.emplace_back("joint" + std::to_string(i+1), std::to_string(joint_angles[i]));
    }
    params.emplace_back("speed", std::to_string(speed));
    params.emplace_back("zone", std::to_string(zone));
    return "movej(" + join_params(params) + ")";
}

std::string ScriptCmd::build_movep(const std::array<double, 6>& target_pos, int speed, int zone) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"x","y","z","rx","ry","rz"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(target_pos[i]));
    }
    params.emplace_back("speed", std::to_string(speed));
    params.emplace_back("zone", std::to_string(zone));
    return "movep(" + join_params(params) + ")";
}

std::string ScriptCmd::build_movel(const std::array<double, 6>& target_pos, int speed, int zone) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"x","y","z","rx","ry","rz"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(target_pos[i]));
    }
    params.emplace_back("speed", std::to_string(speed));
    params.emplace_back("zone", std::to_string(zone));
    return "movel(" + join_params(params) + ")";
}

std::string ScriptCmd::build_stop() {
    return "stop()";
}

std::string ScriptCmd::build_rtControl_jointpos_update(const std::array<double, 6>& joint_angles) {
    std::vector<std::pair<std::string, std::string>> params;
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back("joint" + std::to_string(i+1), std::to_string(joint_angles[i]));
    }
    params.emplace_back("mode", "joint");
    return "rtControl_jointpos_update(" + join_params(params) + ")";
}

std::string ScriptCmd::build_rtControl_cartesianpos_update(const std::array<double, 6>& cartesian_pos) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"x","y","z","rx","ry","rz"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(cartesian_pos[i]));
    }
    params.emplace_back("mode", "cartesian");
    return "rtControl_cartesianpos_update(" + join_params(params) + ")";
}

std::string ScriptCmd::build_follow_pos_update(const std::array<double, 6>& joint_angles, int speed) {
    std::vector<std::pair<std::string, std::string>> params;
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back("joint" + std::to_string(i+1), std::to_string(joint_angles[i]));
    }
    params.emplace_back("speed", std::to_string(speed));
    return "follow_pos_update(" + join_params(params) + ")";
}

std::string ScriptCmd::build_set_follow_speed(int speed) {
    return "set_follow_speed(speed:" + std::to_string(speed) + ")";
}

std::string ScriptCmd::build_calibrateForceSensor(bool all_axes, int axis_index) {
    std::string axes = all_axes ? "true" : "false";
    return "calibrateForceSensor(all_axes:" + axes + ",axis_index:" + std::to_string(axis_index) + ")";
}

std::string ScriptCmd::build_getEndTorque(const std::string& ref_type) {
    return "getEndTorque(ref_type:" + ref_type + ")";
}

std::string ScriptCmd::build_readp(const std::string& ref_type) {
    return "readp(ref_type:" + ref_type + ")";
}

std::string ScriptCmd::build_inv_kinematics(const std::array<double, 6>& Pos) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"x","y","z","rx","ry","rz"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(Pos[i]));
    }
    return "inv_kinematics(" + join_params(params) + ")";
}

std::string ScriptCmd::build_forward_kinematics(const std::array<double, 6>& jntPos) {
    std::vector<std::pair<std::string, std::string>> params;
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back("joint" + std::to_string(i+1), std::to_string(jntPos[i]));
    }
    return "forward_kinematics(" + join_params(params) + ")";
}

std::string ScriptCmd::build_open_rtControl_loop(const std::string& rt_type) {
    return "open_rtControl_loop(rt_type:" + rt_type + ")";
}

std::string ScriptCmd::build_fcInit(const std::string& frame_type) {
    return "fcInit(frame_type:" + frame_type + ")";
}

std::string ScriptCmd::build_fcStart() {
    return "fcStart()";
}

std::string ScriptCmd::build_fcStop() {
    return "fcStop()";
}

std::string ScriptCmd::build_setControlType(int type) {
    return "setControlType(control_type:" + std::to_string(type) + ")";
}

std::string ScriptCmd::build_setLoad(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia) {
    std::vector<std::pair<std::string, std::string>> params;
    params.emplace_back("mass", std::to_string(m));
    params.emplace_back("cog_x", std::to_string(cog[0]));
    params.emplace_back("cog_y", std::to_string(cog[1]));
    params.emplace_back("cog_z", std::to_string(cog[2]));
    params.emplace_back("inertia_x", std::to_string(inertia[0]));
    params.emplace_back("inertia_y", std::to_string(inertia[1]));
    params.emplace_back("inertia_z", std::to_string(inertia[2]));
    return "setLoad(" + join_params(params) + ")";
}

std::string ScriptCmd::build_setCartesianStiffness(const std::array<double, 6> &stiffness) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"kx","ky","kz","krx","kry","krz"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(stiffness[i]));
    }
    return "setCartesianStiffness(" + join_params(params) + ")";
}

std::string ScriptCmd::build_setCartesianNullspaceStiffness(double stiffness) {
    return "setCartesianNullspaceStiffness(stiffness:" + std::to_string(stiffness) + ")";
}

std::string ScriptCmd::build_setCartesianDesiredForce(const std::array<double, 6> &value) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"fx","fy","fz","frx","fry","frz"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(value[i]));
    }
    return "setCartesianDesiredForce(" + join_params(params) + ")";
}

std::string ScriptCmd::build_setSineOverlay(int line_dir, double amplify, double frequency, double phase, double bias) {
    std::vector<std::pair<std::string, std::string>> params;
    params.emplace_back("line_dir", std::to_string(line_dir));
    params.emplace_back("amplify", std::to_string(amplify));
    params.emplace_back("frequency", std::to_string(frequency));
    params.emplace_back("phase", std::to_string(phase));
    params.emplace_back("bias", std::to_string(bias));
    return "setSineOverlay(" + join_params(params) + ")";
}

std::string ScriptCmd::build_setLissajousOverlay(int plane, double amplify_one, double frequency_one, double amplify_two, double frequency_two, double phase_diff) {
    std::vector<std::pair<std::string, std::string>> params;
    params.emplace_back("plane", std::to_string(plane));
    params.emplace_back("amplify_one", std::to_string(amplify_one));
    params.emplace_back("frequency_one", std::to_string(frequency_one));
    params.emplace_back("amplify_two", std::to_string(amplify_two));
    params.emplace_back("frequency_two", std::to_string(frequency_two));
    params.emplace_back("phase_diff", std::to_string(phase_diff));
    return "setLissajousOverlay(" + join_params(params) + ")";
}   

std::string ScriptCmd::build_startOverlay() {
    return "startOverlay()";
}

std::string ScriptCmd::build_stopOverlay() {
    return "stopOverlay()";
}

std::string ScriptCmd::build_pauseOverlay() {
    return "pauseOverlay()";
}

std::string ScriptCmd::build_restartOverlay() {
    return "restartOverlay()";
}

std::string ScriptCmd::build_setForceCondition(const std::array<double, 6> &range, bool isInside, double timeout) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"X_min","Y_min","Z_min","X_max","Y_max","Z_max"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(range[i]));
    }
    params.emplace_back("isInside", isInside ? "true" : "false");
    params.emplace_back("timeout", std::to_string(timeout));
    return "setForceCondition(" + join_params(params) + ")";
}

std::string ScriptCmd::build_setTorqueCondition(const std::array<double, 6> &range, bool isInside, double timeout) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"X_min","Y_min","Z_min","X_max","Y_max","Z_max"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(range[i]));
    }
    params.emplace_back("isInside", isInside ? "true" : "false");
    params.emplace_back("timeout", std::to_string(timeout));
    return "setTorqueCondition(" + join_params(params) + ")";
}

std::string ScriptCmd::build_setPoseBoxCondition(const std::array<double, 6> &frame, const std::array<double, 6> &box, bool isInside, double timeout) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* frame_names[6] = {"X","Y","Z","Rx","Ry","Rz"};
    const char* box_names[6] = {"X_start","X_end","Y_start","Y_end","Z_start","Z_end"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(frame_names[i], std::to_string(frame[i]));
    }
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(box_names[i], std::to_string(box[i]));
    }
    params.emplace_back("isInside", isInside ? "true" : "false");
    params.emplace_back("timeout", std::to_string(timeout));
    return "setPoseBoxCondition(" + join_params(params) + ")";
}

std::string ScriptCmd::build_waitCondition() {
    return "waitCondition()";
}

std::string ScriptCmd::build_fcMonitor(bool enable) {
    return "fcMonitor(enable:" + std::string(enable ? "true" : "false") + ")";
}

std::string ScriptCmd::build_setCartesianMaxVel(const std::array<double, 6> &max_vel) {
    std::vector<std::pair<std::string, std::string>> params;
    const char* names[6] = {"X","Y","Z","Rx","Ry","Rz"};
    for (size_t i = 0; i < 6; ++i) {
        params.emplace_back(names[i], std::to_string(max_vel[i]));
    }
    return "setCartesianMaxVel(" + join_params(params) + ")";
} 


// Cr7ScriptClient 构造与析构
Cr7ScriptClient::Cr7ScriptClient() : Node("cr7_script_client") {
    script_client_ = this->create_client<coordinate::srv::StringScript>("cr7_script");
    // 等待服务可用
    while (!script_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for cr7_server service...");
    }
    // 提示服务已连接
    RCLCPP_INFO(this->get_logger(), "Connected to cr7_server service.");
}

Cr7ScriptClient::~Cr7ScriptClient() {}

bool Cr7ScriptClient::send_script(const std::string& command, std::string& result) {
    auto req = std::make_shared<coordinate::srv::StringScript::Request>();
    req->command = command;
    auto future = script_client_->async_send_request(req);
    // 等待结果
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
    rclcpp::FutureReturnCode::SUCCESS) {
        result = future.get()->result;
        return true;
    } else {
        result = "Service call failed";
        return false;
    }
}

// 快捷接口实现
bool Cr7ScriptClient::movej(const std::vector<double>& joint_angles, int speed, int zone, std::string& result) {
    std::string cmd = ScriptCmd::build_movej(joint_angles, speed, zone);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::movep(const std::array<double, 6>& target_pos, int speed, int zone, std::string& result) {
    std::string cmd = ScriptCmd::build_movep(target_pos, speed, zone);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::movel(const std::array<double, 6>& target_pos, int speed, int zone, std::string& result) {
    std::string cmd = ScriptCmd::build_movel(target_pos, speed, zone);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::stop(std::string& result) {
    std::string cmd = ScriptCmd::build_stop();
    return send_script(cmd, result);
}
bool Cr7ScriptClient::rtControl_jointpos_update(const std::array<double, 6>& joint_angles, std::string& result) {
    std::string cmd = ScriptCmd::build_rtControl_jointpos_update(joint_angles);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::rtControl_cartesianpos_update(const std::array<double, 6>& cartesian_pos, std::string& result) {
    std::string cmd = ScriptCmd::build_rtControl_cartesianpos_update(cartesian_pos);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::follow_pos_update(const std::array<double, 6>& joint_angles, int speed, std::string& result) {
    std::string cmd = ScriptCmd::build_follow_pos_update(joint_angles, speed);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::set_follow_speed(int speed, std::string& result) {
    std::string cmd = ScriptCmd::build_set_follow_speed(speed);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::calibrateForceSensor(bool all_axes, int axis_index, std::string& result) {
    std::string cmd = ScriptCmd::build_calibrateForceSensor(all_axes, axis_index);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::getEndTorque(const std::string& ref_type, std::string& result) {
    std::string cmd = ScriptCmd::build_getEndTorque(ref_type);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::inv_kinematics(const std::array<double, 6>& Pos, std::string& result) {
    std::string cmd = ScriptCmd::build_inv_kinematics(Pos);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::forward_kinematics(const std::array<double, 6>& jntPos, std::string& result) {
    std::string cmd = ScriptCmd::build_forward_kinematics(jntPos);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::readp(const std::string& ref_type, std::string& result) {
    std::string cmd = ScriptCmd::build_readp(ref_type);
    return send_script(cmd, result);
}
bool Cr7ScriptClient::readj(std::string& result) {
    std::string cmd = "readj()";
    return send_script(cmd, result);
}
bool Cr7ScriptClient::get_vel(std::string& result) {
    std::string cmd = "get_vel()";
    return send_script(cmd, result);
}
bool Cr7ScriptClient::get_joint_vel(std::string& result) {
    std::string cmd = "get_joint_vel()";
    return send_script(cmd, result);
}
bool Cr7ScriptClient::get_acc(std::string& result) {
    std::string cmd = "get_acc()";
    return send_script(cmd, result);
}
bool Cr7ScriptClient::get_jerk(std::string& result) {
    std::string cmd = "get_jerk()";
    return send_script(cmd, result);
}
bool Cr7ScriptClient::get_joint_torque(std::string& result) {
    std::string cmd = "get_joint_torque()";
    return send_script(cmd, result);
}

bool Cr7ScriptClient::follow_pos_start(std::string& result) {
    std::string cmd = "follow_pos_start()";
    return send_script(cmd, result);
}

bool Cr7ScriptClient::follow_pos_stop(std::string& result) {
    std::string cmd = "follow_pos_stop()";
    return send_script(cmd, result);
}

bool Cr7ScriptClient::open_rtControl_loop(const std::string& rt_type, std::string& result) {
    std::string cmd = ScriptCmd::build_open_rtControl_loop(rt_type);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::fcInit(const std::string& frame_type, std::string& result) {
    std::string cmd = ScriptCmd::build_fcInit(frame_type);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::fcStart(std::string& result) {
    std::string cmd = ScriptCmd::build_fcStart();
    return send_script(cmd, result);
}

bool Cr7ScriptClient::fcStop(std::string& result) {
    std::string cmd = ScriptCmd::build_fcStop();
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setControlType(int type, std::string& result) {
    std::string cmd = ScriptCmd::build_setControlType(type);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setLoad(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia, std::string& result) {
    std::string cmd = ScriptCmd::build_setLoad(m, cog, inertia);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setCartesianStiffness(const std::array<double, 6> &stiffness, std::string& result) {
    std::string cmd = ScriptCmd::build_setCartesianStiffness(stiffness);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setCartesianNullspaceStiffness(double stiffness, std::string& result) {
    std::string cmd = ScriptCmd::build_setCartesianNullspaceStiffness(stiffness);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setCartesianDesiredForce(const std::array<double, 6> &value, std::string& result) {
    std::string cmd = ScriptCmd::build_setCartesianDesiredForce(value);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setSineOverlay(int line_dir, double amplify, double frequency, double phase, double bias, std::string& result) {
    std::string cmd = ScriptCmd::build_setSineOverlay(line_dir, amplify, frequency, phase, bias);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setLissajousOverlay(int plane, double amplify_one, double frequency_one, double amplify_two, double frequency_two, double phase_diff, std::string& result) {
    std::string cmd = ScriptCmd::build_setLissajousOverlay(plane, amplify_one, frequency_one, amplify_two, frequency_two, phase_diff);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::startOverlay(std::string& result) {
    std::string cmd = ScriptCmd::build_startOverlay();
    return send_script(cmd, result);
}

bool Cr7ScriptClient::stopOverlay(std::string& result) {
    std::string cmd = ScriptCmd::build_stopOverlay();
    return send_script(cmd, result);
}

bool Cr7ScriptClient::pauseOverlay(std::string& result) {
    std::string cmd = ScriptCmd::build_pauseOverlay();
    return send_script(cmd, result);
}

bool Cr7ScriptClient::restartOverlay(std::string& result) {
    std::string cmd = ScriptCmd::build_restartOverlay();
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setForceCondition(const std::array<double, 6> &range, bool isInside, double timeout, std::string& result) {
    std::string cmd = ScriptCmd::build_setForceCondition(range, isInside, timeout);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setTorqueCondition(const std::array<double, 6> &range, bool isInside, double timeout, std::string& result) {
    std::string cmd = ScriptCmd::build_setTorqueCondition(range, isInside, timeout);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setPoseBoxCondition(const std::array<double, 6> &frame, const std::array<double, 6> &box, bool isInside, double timeout, std::string& result) {
    std::string cmd = ScriptCmd::build_setPoseBoxCondition(frame, box, isInside, timeout);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::waitCondition(std::string& result) {
    std::string cmd = ScriptCmd::build_waitCondition();
    return send_script(cmd, result);
}

bool Cr7ScriptClient::fcMonitor(bool enable, std::string& result) {
    std::string cmd = ScriptCmd::build_fcMonitor(enable);
    return send_script(cmd, result);
}

bool Cr7ScriptClient::setCartesianMaxVel(const std::array<double, 6> &max_vel, std::string& result) {
    std::string cmd = ScriptCmd::build_setCartesianMaxVel(max_vel);
    return send_script(cmd, result);
}

} // namespace cr7_script

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cr7_script::Cr7ScriptClient>();
    // 示例：构建并发送 movej 指令
    std::vector<double> joints = {M_PI/2, 0, M_PI/2, 0, M_PI/2, 0};
    std::string result;
    node->movej(joints, 100, 5, result);
    RCLCPP_INFO(node->get_logger(), "Service result: %s", result.c_str());
    std::vector<double> joints2 = {M_PI/2, 0, M_PI/2, 0, 0, 0};
    node->movej(joints2, 100, 5, result);
    RCLCPP_INFO(node->get_logger(), "Service result: %s", result.c_str());
    rclcpp::shutdown();
    return 0;
}