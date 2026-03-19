#ifndef CR7_SCRIPT_H
#define CR7_SCRIPT_H

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include "coordinate/srv/string_script.hpp"

namespace cr7_script {

class ScriptCmd
{
public:
    ScriptCmd();
    ~ScriptCmd();

    // 指令字符串构建接口(仅构建需要参数的指令)
    static std::string build_movej(const std::vector<double>& joint_angles, int speed, int zone);
    static std::string build_movep(const std::array<double, 6>& target_pos, int speed, int zone);
    static std::string build_movel(const std::array<double, 6>& target_pos, int speed, int zone);

    static std::string build_stop();

    static std::string build_rtControl_jointpos_update(const std::array<double, 6>& joint_angles);
    static std::string build_rtControl_cartesianpos_update(const std::array<double, 6>& cartesian_pos);

    static std::string build_follow_pos_update(const std::array<double, 6>& joint_angles, int speed);
    static std::string build_set_follow_speed(int speed);

    static std::string build_calibrateForceSensor(bool all_axes, int axis_index);

    static std::string build_getEndTorque(const std::string& ref_type);
    static std::string build_readp(const std::string& ref_type);

    static std::string build_inv_kinematics(const std::array<double, 6>& Pos);
    static std::string build_forward_kinematics(const std::array<double, 6>& jntPos);

    static std::string build_open_rtControl_loop(const std::string& rt_type);

    static std::string build_fcInit(const std::string& frame_type);
    static std::string build_fcStart();
    static std::string build_fcStop();
    static std::string build_setControlType(int type);
    static std::string build_setLoad(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia);
    static std::string build_setCartesianStiffness(const std::array<double, 6> &stiffness);
    static std::string build_setCartesianNullspaceStiffness(double stiffness);
    static std::string build_setCartesianDesiredForce(const std::array<double, 6> &value);
    static std::string build_setSineOverlay(int line_dir, double amplify, double frequency, double phase, double bias);
    static std::string build_setLissajousOverlay(int plane, double amplify_one, double frequency_one, double amplify_two, double frequency_two, double phase_diff);
    static std::string build_startOverlay();
    static std::string build_stopOverlay();
    static std::string build_pauseOverlay();
    static std::string build_restartOverlay();
    static std::string build_setForceCondition(const std::array<double, 6> &range, bool isInside, double timeout);
    static std::string build_setTorqueCondition(const std::array<double, 6> &range, bool isInside, double timeout);
    static std::string build_setPoseBoxCondition(const std::array<double, 6> &frame, const std::array<double, 6> &box, bool isInside, double timeout);
    static std::string build_waitCondition();
    static std::string build_fcMonitor(bool enable);
    static std::string build_setCartesianMaxVel(const std::array<double, 6> &max_vel);

    // 辅助：参数拼接
    static std::string join_params(const std::vector<std::pair<std::string, std::string>>& params);
};

class Cr7ScriptClient : public rclcpp::Node
{
public:
    Cr7ScriptClient();
    ~Cr7ScriptClient();

    // 发送服务请求
    bool send_script(const std::string& command, std::string& result);

    // 快捷接口：构造并发送指令
    bool movej(const std::vector<double>& joint_angles, int speed, int zone, std::string& result);
    bool movep(const std::array<double, 6>& target_pos, int speed, int zone, std::string& result);
    bool movel(const std::array<double, 6>& target_pos, int speed, int zone, std::string& result);
    bool stop(std::string& result);

    bool open_rtControl_loop(const std::string& rt_type, std::string& result);
    bool rtControl_jointpos_update(const std::array<double, 6>& joint_angles, std::string& result);
    bool rtControl_cartesianpos_update(const std::array<double, 6>& cartesian_pos, std::string& result);
    
    bool follow_pos_start(std::string& result);
    bool follow_pos_stop(std::string& result);
    bool follow_pos_update(const std::array<double, 6>& joint_angles, int speed, std::string& result);
    bool set_follow_speed(int speed, std::string& result);

    bool calibrateForceSensor(bool all_axes, int axis_index, std::string& result);

    bool readp(const std::string& ref_type, std::string& result);
    bool readj(std::string& result);
    bool get_vel(std::string& result);
    bool get_joint_vel(std::string& result);
    bool get_acc(std::string& result);
    bool get_jerk(std::string& result);
    bool get_joint_torque(std::string& result);
    bool getEndTorque(const std::string& ref_type, std::string& result);

    bool inv_kinematics(const std::array<double, 6>& Pos, std::string& result);
    bool forward_kinematics(const std::array<double, 6>& jntPos, std::string& result);

    bool fcInit(const std::string& frame_type, std::string& result);
    bool fcStart(std::string& result);
    bool fcStop(std::string& result);
    bool setControlType(int type, std::string& result);
    bool setLoad(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia, std::string& result);
    bool setCartesianStiffness(const std::array<double, 6> &stiffness, std::string& result);
    bool setCartesianNullspaceStiffness(double stiffness, std::string& result);
    bool setCartesianDesiredForce(const std::array<double, 6> &value, std::string& result);
    bool setSineOverlay(int line_dir, double amplify, double frequency, double phase, double bias, std::string& result);
    bool setLissajousOverlay(int plane, double amplify_one, double frequency_one, double amplify_two, double frequency_two, double phase_diff, std::string& result);
    bool startOverlay(std::string& result);
    bool stopOverlay(std::string& result);
    bool pauseOverlay(std::string& result);
    bool restartOverlay(std::string& result);
    bool setForceCondition(const std::array<double, 6> &range, bool isInside, double timeout, std::string& result);
    bool setTorqueCondition(const std::array<double, 6> &range, bool isInside, double timeout, std::string& result);
    bool setPoseBoxCondition(const std::array<double, 6> &frame, const std::array<double, 6> &box, bool isInside, double timeout, std::string& result);
    bool waitCondition(std::string& result);
    bool fcMonitor(bool enable, std::string& result);
    bool setCartesianMaxVel(const std::array<double, 6> &max_vel, std::string& result);

private:
    rclcpp::Client<coordinate::srv::StringScript>::SharedPtr script_client_;
};

} // namespace cr7_script

#endif

