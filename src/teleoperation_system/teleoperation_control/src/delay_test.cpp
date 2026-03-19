#include <iostream>
#include "sigema7_pkg/drdc.h"
#include "sigema7_pkg/dhdc.h"
#include "x_core_sdk/robot.h"
#include "x_core_sdk/utility.h"
#include "x_core_sdk/model.h"
#include <thread>
#include <chrono>

using namespace std;
using namespace rokae;
using namespace rokae::RtSupportedFields;

// 全局变量用于存储线程开始时间
std::chrono::time_point<std::chrono::high_resolution_clock> absTime;


// 定义一个函数用于记录时间
std::chrono::time_point<std::chrono::high_resolution_clock> recordTime() {
    return std::chrono::high_resolution_clock::now();
}

// 定义一个函数用于计算时间差
double calculateTimeDifference(const std::chrono::time_point<std::chrono::high_resolution_clock>& start) {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double>(end - start).count();
}


//主端设备初始化
void MasterDeviceInit()
{
    // 1 定义工作空间中心位置，开启专家模式
    double nullPose[DHD_MAX_DOF] = {0.0, 0.0, 0.0, // base  (translations)
                                    0.0, 0.0, 0.0, // wrist (rotations)
                                    0.0};          // gripper
    dhdEnableExpertMode();
    // 2 启动第一个手控器设备
    if (drdOpen() < 0)
    {
        printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return;
    }
    // 3 打印设备编码
    if (!drdIsSupported())
    {
        printf("unsupported device\n");
        printf("exiting...\n");
        dhdSleep(2.0);
        drdClose();
        return;
    }
    printf("%s haptic device detected\n", dhdGetSystemName());
    // 4 执行自动初始化
    if (!drdIsInitialized() && drdAutoInit() < 0)
    {
        printf("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return;
    }
    else if (drdStart() < 0)
    {
        printf("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return;
    }
    // 5 启动力控和激活手腕控制
    dhdEnableForce(DHD_ON);
    if (!dhdHasActiveWrist())
    {
        dhdClose();
        printf("error: device does not have an active wrist\n");
        dhdSleep(2.0);
        return;
    }
    // 6 移动到工作空间中心位置
    drdStart();
    drdMoveTo(nullPose);
    // // 7 停止初始化，开启操作
    // drdStop(true);

    // 读取频率
    double frequency = drdGetCtrlFreq();
    std::cout << "frequency: " << frequency << std::endl;

    std::cout << "主端设备初始化完成" << std::endl;

}

void open_sigema7_loop()
{   
    // 设置初始位置
    double sigema7_pos_default[6] = {-0.00544945, -0.000884445, -0.00275382, 0.00194075, 0.000535195, -0.00308807};
    // 定义目标位置
    double targetX = sigema7_pos_default[0];
    double targetY = sigema7_pos_default[1];
    double targetZ = sigema7_pos_default[2];

    // 沿着初始位置每隔10毫秒，移动500个单位，每次沿着x方向移动0.0001m，直到移动到目标位置
    int moveCount = 6;
    double moveStepX = 0.01;
    double moveStepY = 0.05;
    double moveStepZ = 0.05;

    auto startTime = recordTime();
    std::cout << "进入sigema7线程时间: " << calculateTimeDifference(absTime) << " 秒" << std::endl;

    for (int i = 0; i < moveCount; i++) {
        // 计算新的位置
        targetX = targetX + moveStepX;
        // 移动到新的位置
        int moveResult = drdMoveToPos(targetX, targetY, targetZ);
        if (moveResult!= 0) {
            std::cerr << "Failed to move to the new position!" << std::endl;
            return ;
        }
        // std::cout << "移动到新的位置: " << targetX << ", " << targetY << ", " << targetZ << std::endl;
        drdSleep(0.001);
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
    }
    for (int i = 0; i < moveCount; i++) {
        // 计算新的位置
        targetX = targetX - moveStepX;
        // double targetY = targetY - moveStepY;
        // double targetZ = targetZ - moveStepZ;
        // 移动到新的位置
        int moveResult = drdMoveToPos(targetX, targetY, targetZ);
        if (moveResult!= 0) {
            std::cerr << "Failed to move to the new position!" << std::endl;
            return ;
        }
        // std::cout << "移动到新的位置: " << targetX << ", " << targetY << ", " << targetZ << std::endl;
        drdSleep(0.001);
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
    }

    // 停止调节线程
    int stopResult = drdStop();
    if (stopResult != 0) {
        std::cerr << "Failed to stop the regulation thread!" << std::endl;
        return ;
    }
    std::cout << "sigema7线程完成时间: " << calculateTimeDifference(absTime) << " 秒" << std::endl;
}


//机器人初始化
void robot_initialize(xMateRobot &robot, std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> &rtCon)
    {   
        // 机器人参数
        bool running;
        double mapping_ratio = 1.0;
        double mapping_rad_ratio = 0.5;
        double speed_ratio = 0.7;
        double default_speed = 200;
        std::error_code ec;
        std::string id;

        // 打印出测试字符
        std::cout << "机器人初始化开始" << std::endl;

        // 初始化机器人：设置操作模式、电源状态和运动重置
        double scale = 1;
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setPowerState(true, ec);
        robot.moveReset(ec);
        robot.adjustSpeedOnline(scale, ec);
        robot.setDefaultSpeed(default_speed, ec);
        robot.setDefaultZone(10, ec);
        robot.adjustAcceleration(0.5,0.5, ec);

        // 运动至机器人初始位置
        MoveAbsJCommand  startPosjoint({M_PI/2, 0, -M_PI/2, 0, -M_PI/2, 0});
        robot.moveAppend({startPosjoint}, id, ec);
        robot.moveStart(ec);

        // 等待1s
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
        // 开启实时控制
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            std::cerr << "获取实时运动控制器失败" << std::endl;
            return;
        }

        std::cout << "机器人初始化完成" << std::endl;
    }

void rtControl_loop(xMateRobot &robot,  std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> &rtCon, std::array<double, 6> target_joint)
{   
    std::error_code ec;
    // 设置回调函数
    std::function<JointPosition()> rtC_loop_callback = [&](){
        target_joint[4] = target_joint[4] + 0.009/180*M_PI;
        JointPosition cmd = {{target_joint[0], target_joint[1], target_joint[2], target_joint[3], target_joint[4], target_joint[5] }};
        if(target_joint[4] > 0) {
            cmd.setFinished(); // 结束运动
        }
        return cmd;
        
    };
    // 设置滤波器参数，用于平滑？
    rtCon->setFilterFrequency(1, 10, 10, ec) ;

    // 设置回调函数
    rtCon->setControlLoop(rtC_loop_callback);

    std::cout << "进入从端线程时间: " << calculateTimeDifference(absTime) << " 秒" << std::endl;
    // 开始轴空间位置控制
    rtCon->startMove(RtControllerMode::jointPosition);
    // 阻塞loop
    rtCon->startLoop(true);

    std::cout << "从端线程完成时间: " << calculateTimeDifference(absTime) << " 秒" << std::endl;

}

int main() {

    absTime = recordTime();
    std::cout << "主程序开始，时间: " << calculateTimeDifference(absTime) << " 秒" << std::endl;

    // 初始化机械臂
    xMateRobot robot("192.168.2.160", "192.168.2.101");
    std::error_code ec;
    std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> rtCon;
    robot_initialize(robot,rtCon);
    // 初始化设备
    MasterDeviceInit();

    std::array<double, 6> target_joint = robot.jointPos(ec);
    // 打印错误
    if (ec) {
        std::cerr << "获取机器人状态失败: " << ec.message() << std::endl;
        return -1;
    }

    // 延时3s
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // 创建并启动机械臂线程
    std::thread robotThread(rtControl_loop, std::ref(robot), std::ref(rtCon), std::ref(target_joint));
    // 创建并启动手柄线程
    std::thread sigema7Thread(open_sigema7_loop);

    // 等待机械臂线程完成
    if (robotThread.joinable()) {
        robotThread.join();
    }   
    // 等待手柄线程完成
    if (sigema7Thread.joinable()) {
        sigema7Thread.join();
    }

    // 关闭手柄
    dhdClose();
    // 关闭机械臂
    robot.setPowerState(false, ec);
    robot.stopReceiveRobotState();

    std::cout << "TEST OVER "<< std::endl;
    std::cout << "TEST OVER，总耗时: " << calculateTimeDifference(absTime) << " 秒" << std::endl;

    return 0;
}