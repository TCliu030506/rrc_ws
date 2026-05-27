#define XMATEMODEL_LIB_SUPPORTED
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <iostream>
#include <thread>
#include <atomic>
#include <map>
#include <string>
#include <mutex>
#include <Eigen/Dense>
#include <array>

#include "x_core_sdk/robot.h"
#include "x_core_sdk/utility.h"
#include "x_core_sdk/model.h"
#include "xmate_cr7_msg/action/cr7_script.hpp"
#include "xmate_cr7_msg/srv/cr7_script.hpp"
#include "coordinate/srv/string_script.hpp"


using namespace std;
using namespace rokae;
using namespace rokae::RtSupportedFields;

class Cr7Server : public rclcpp::Node
{
public:
    Cr7Server();
    ~Cr7Server();

private:
    rclcpp::Service<coordinate::srv::StringScript>::SharedPtr cr7_service;
    xMateRobot robot;
    FollowPosition<6> follow_pose;
    shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> rtCon;
    ForceControl_T<6> fc;
    thread followThread;
    thread rtConThread;
    std::mutex mtx; // 互斥锁
    bool running = true; // 用于控制线程的运行状态
    error_code ec;
    string id;
    CoordinateType ct = CoordinateType::flangeInBase;
    array<double, 6> target_joint; // 目标关节位置
    array<double, 6> target_pose; // 目标笛卡尔位置
    double default_speed = 400.0;
    double speed_ratio = 1.0;

    void init();

    void execute_cmd_callback(const std::shared_ptr<coordinate::srv::StringScript::Request> req,
                              std::shared_ptr<coordinate::srv::StringScript::Response> res);

    // 新增：字符串指令解析与分发
    bool parse_and_execute(const std::string& command, std::string& result);

    // 新增：参数字符串解析
    std::map<std::string, std::string> parse_params(const std::string& param_str);

    void stop(std::string& result);

    void movej(const std::vector<double>& joint_angles, int speed, int zone, std::string& result);
    void movep(const std::array<double, 6>& target_pos, int speed, int zone, std::string& result);
    void movel(const std::array<double, 6>& target_pos, int speed, int zone, std::string& result);

    void rtControl_loop(string rt_type);
    void open_rtControl_loop(string rt_type);
    void rtControl_jointpos_update(const std::array<double, 6>& joint_angles, std::string& result);
    void rtControl_cartesianpos_update(const std::array<double, 6>& cartesian_pos, std::string& result);

    void followPosition(xMateRobot &robot, std::array<double, 6> &target_jnt, double speed_ratio); 
    void follow_pos_start();
    void follow_pos_stop();
    void follow_pos_update(const std::array<double, 6>& joint_angles, int speed, std::string& result);

    void set_follow_speed(int speed, std::string& result);

    std::array<double, 6>  readp(std::string& ref_type_str);
    std::array<double, 6>  readj();

    std::array<double, 6>  get_vel();
    std::array<double, 6>  get_joint_vel();

    double  get_acc();
    double  get_jerk();

    void calibrateForceSensor(bool all_axes, int axis_index, std::string& result);

    std::array<double, 6>  get_joint_torque();

    void inv_kinematics(std::array<double, 6> &Pos, std::array<double, 6> &jntPos);
    void forward_kinematics(std::array<double, 6> &jntPos, std::array<double, 6> &Pos);

    void getEndTorque(string ref_type_str, std::array<double, 6> &joint_torque_measured, std::array<double, 6> &external_torque_measured,
        std::array<double, 3> &cart_torque, std::array<double, 3> &cart_force);
    

    //力控相关 

    /**
    * @brief 力控初始化
    * @param[in] frame_type 力控坐标系，支持world/wobj/tool/base/flange。工具工件坐标系使用setToolset()设置的坐标系
    * @param[out] ec 错误码
    */
   void fcInit(FrameType frame_type, error_code &ec) ;

   /**
    * @brief 开始力控，fcInit()之后调用。
    * 如需在力控模式下执行运动指令，fcStart()之后可执行。
    * 注意，如果在fcStart()之前通过moveAppend()下发了运动指令但未开始运动，fcStart之后就会执行这些运动指令。
    * @param[out] ec 错误码
    */
   void fcStart(error_code &ec) ;

   /**
    * @brief 停止力控
    * @param[out] ec 错误码
    */
   void fcStop(error_code &ec) ;

   /**
    * @brief 设置阻抗控制类型
    * @param[in] type 0 - 关节阻抗 | 1 - 笛卡尔阻抗
    * @param[out] ec 错误码
    */
   void setControlType(int type, error_code &ec) ;

   /**
    * @brief 设置力控模块使用的负载信息，fcStart()之后可调用。
    * @param[in] m 负载质量, 单位:千克
    * @param[in] cog 质心 [x, y, z], 单位:米
    * @param[in] inertia 惯量 [ix, iy, iz], 单位:千克·平方米
    * @param[out] ec 错误码
    */
   void setLoad(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia, error_code &ec) ;

   /**
    * @brief 设置笛卡尔阻抗刚度。fcInit()之后调用生效
    * 各机型的最大刚度不同，请参考《xCore控制系统手册》 SetCartCtrlStiffVec指令的说明
    * @param[in] stiffness 依次为：X Y Z方向阻抗力刚度[N/m], X Y Z方向阻抗力矩刚度[Nm/rad]
    * @param[out] ec 错误码
    */
   void setCartesianStiffness(const std::array<double, 6> &stiffness, error_code &ec) ;

   /**
    * @brief 设置笛卡尔零空间阻抗刚度。fcInit()之后调用生效
    * @param[in] stiffness 范围[0,4], 大于4会默认设置为4, 单位Nm/rad
    * @param[out] ec 错误码
    */
   void setCartesianNullspaceStiffness(double stiffness, error_code &ec) ;

   /**
    * @brief 设置笛卡尔期望力/力矩。fcStart()之后可调用
    * @param[in] value 依次为: X Y Z方向笛卡尔期望力, 范围[-60,60], 单位N; X Y Z方向笛卡尔期望力矩, 范围[-10,10], 单位Nm
    * @param[out] ec 错误码
    */
   void setCartesianDesiredForce(const std::array<double, 6> &value, error_code &ec) ;

   /**
    * @brief 设置绕单轴旋转的正弦搜索运动。
    * 设置阻抗控制类型为笛卡尔阻抗(即setControlType(1))之后, startOverlay()之前调用生效。
    * 各机型的搜索运动幅值上限和搜索运动频率上限不同，请参考《xCore控制系统手册》 SetSineOverlay指令的说明。
    * @param[in] line_dir 搜索运动参考轴: 0 - X | 1 - Y | 2 - Z
    * @param[in] amplify 搜索运动幅值, 单位Nm
    * @param[in] frequency 搜索运动频率, 单位Hz
    * @param[in] phase 搜索运动相位, 范围[0, PI], 单位弧度
    * @param[in] bias 搜索运动偏置, 范围[0, 10], 单位Nm
    * @param[out] ec 错误码
    */
   void setSineOverlay(int line_dir, double amplify, double frequency, double phase,
                       double bias, error_code &ec) ;

   /**
    * @brief 设置平面内的莉萨如搜索运动
    * 设置阻抗控制类型为笛卡尔阻抗(即setControlType(1))之后, startOverlay()之前调用生效。
    * @param[in] plane 搜索运动参考平面: 0 - XY | 1 - XZ | 2 - YZ
    * @param[in] amplify_one 搜索运动一方向幅值, 范围[0, 20], 单位Nm
    * @param[in] frequency_one 搜索运动一方向频率, 范围[0, 5], 单位Hz
    * @param[in] amplify_two 搜索运动二方向幅值, 范围[0, 20]单位Nm
    * @param[in] frequency_two 搜索运动二方向频率, 范围[0, 5], 单位Hz
    * @param[in] phase_diff 搜索运动两个方向相位偏差, 范围[0, PI], 单位弧度
    * @param[out] ec 错误码
    */
   void setLissajousOverlay(int plane, double amplify_one, double frequency_one, double amplify_two,
                            double frequency_two, double phase_diff, error_code &ec) ;

   /**
    * @brief 开启搜索运动。fcStart()之后调用生效
    * 搜索运动为前序设置的 setSineOverlay()或 setLissajousOverlay()的叠加
    * @param[out] ec 错误码
    */
   void startOverlay(error_code &ec) ;

   /**
    * @brief 停止搜索运动
    * @param[out] ec 错误码
    */
   void stopOverlay(error_code &ec) ;

   /**
    * @brief 暂停搜索运动。startOverlay()之后调用生效
    * @param[out] ec 错误码
    */
   void pauseOverlay(error_code &ec) ;

   /**
    * @brief 重新开启暂停的搜索运动。pauseOverlay()之后调用生效。
    * @param[out] ec 错误码
    */
   void restartOverlay(error_code &ec) ;

   /**
    * @brief 设置与接触力有关的终止条件
    * @param[in] range 各方向上的力限制 { X_min, X_max, Y_min, Y_max, Z_min, Z_max }, 单位N。
    * 设置下限时, 负值表示负方向上的最大值; 设置上限时, 负值表示负方向上的最小值。
    * @param[in] isInside true - 超出限制条件时停止等待; false - 符合限制条件时停止等待
    * @param[in] timeout 超时时间, 范围[1, 600], 单位秒
    * @param[out] ec 错误码
    */
   void setForceCondition(const std::array<double, 6> &range, bool isInside, double timeout, error_code &ec) ;

   /**
    * @brief 设置与接触力矩有关的终止条件
    * @param[in] range 各方向上的力矩限制 { X_min, X_max, Y_min, Y_max, Z_min, Z_max }, 单位Nm。
    * 设置下限时, 负值表示负方向上的最大值; 设置上限时, 负值表示负方向上的最小值。
    * @param[in] isInside true - 超出限制条件时停止等待; false - 符合限制条件时停止等待
    * @param[in] timeout 超时时间, 范围[1, 600], 单位秒
    * @param[out] ec 错误码
    */
   void setTorqueCondition(const std::array<double, 6> &range, bool isInside, double timeout, error_code &ec) ;

   /**
    * @brief 设置与接触位置有关的终止条件
    * @param[in] supervising_frame 长方体所在的参考坐标系, 相对于外部工件坐标系。
    *  改用：frame [X, Y, Z, Rx, Ry, Rz]
    * 外部工件坐标系是通过setToolset()设置的 (Toolset::ref)
    * @param[in] box 定义一个长方体 { X_start, X_end, Y_start, Y_end, Z_start, Z_end }, 单位米
    * @param[in] isInside true - 超出限制条件时停止等待; false - 符合限制条件时停止等待
    * @param[in] timeout 超时时间, 范围[1, 600], 单位秒
    * @param[out] ec 错误码
    */
   void setPoseBoxCondition(const std::array<double, 6> &frame, const std::array<double, 6> &box, bool isInside,
                           double timeout, error_code &ec) ;

   /**
    * @brief 激活前序设置的终止条件并等待，直到满足这些条件或者超时
    * @param[out] ec 错误码
    */
   void waitCondition(error_code &ec) ;

   /**
    * @brief 启动/关闭力控模块保护监控。
    * 设置监控参数后，不立即生效，调用fcMonitor(true)后开始生效，并且一直保持，直到调用fcMotion(false)后结束。
    * 结束后保护阈值恢复成默认值，即仍然会有保护效果，关闭监控后不再是用户设置的参数。
    * @param[in] enable true - 打开 | false - 关闭
    * @param[out] ec 错误码
    * @see setCartesianMaxVel() setJointMaxVel() setJointMaxMomentum() setJointMaxEnergy()
    */
   void fcMonitor(bool enable, error_code &ec) ;

   /**
    * @brief 设置力控模式下, 机械臂末端相对基坐标系的最大速度
    * @param[in] velocity 依次为：X Y Z [m/s], A B C [rad/s], 范围 >=0
    * @param[out] ec 错误码
    */
   void setCartesianMaxVel(const std::array<double, 6> &velocity, error_code &ec) ;

};

// 备注说明：
// 1. 所有运动命令（movej, movep, movel）在发送后立即返回，不等待运动完成。
// 2. 运动停止（stop）命令会立即停止机器人运动，不等待当前运动完成。
// 3. 运动速度（speed）参数为机器人末端最大线速度, 单位mm/s。
// 4. 运动区域（zone）参数为转弯区半径大小，单位mm。


// 将 std::array<double, 16> 转为 Eigen::Matrix4d
inline Eigen::Matrix4d arrayToMatrix4d(const std::array<double, 16>& arr) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 16; ++i)
        mat(i / 4, i % 4) = arr[i];
    return mat;
}

// 将 Eigen::Matrix4d 转为 std::array<double, 16>
inline std::array<double, 16> matrix4dToArray(const Eigen::Matrix4d& mat) {
    std::array<double, 16> arr;
    for (int i = 0; i < 16; ++i)
        arr[i] = mat(i / 4, i % 4);
    return arr;
}

// 4x4矩阵乘法
inline void matrixMultiply(const std::array<double, 16>& a, const std::array<double, 16>& b, std::array<double, 16>& result) {
    Eigen::Matrix4d mat_a = arrayToMatrix4d(a);
    Eigen::Matrix4d mat_b = arrayToMatrix4d(b);
    Eigen::Matrix4d mat_result = mat_a * mat_b;
    result = matrix4dToArray(mat_result);
}