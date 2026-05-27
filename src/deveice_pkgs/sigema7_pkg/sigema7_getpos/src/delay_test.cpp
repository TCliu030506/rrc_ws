#include <iostream>
#include "sigema7_pkg/drdc.h"
#include "sigema7_pkg/dhdc.h"
#include <thread>

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
    // 7 停止初始化，开启操作
    // drdStop(true);
}


int main() {
    // 初始化设备
    MasterDeviceInit();
    double frequency = drdGetCtrlFreq();
    std::cout << "frequency: " << frequency << std::endl;
    // 等待1
    drdSleep(1.0);


    // 设置初始位置
    double sigema7_pos_default[6] = {-0.00544945, -0.000884445, -0.00275382, 0.00194075, 0.000535195, -0.00308807};
    // 定义目标位置
    double targetX = sigema7_pos_default[0];
    double targetY = sigema7_pos_default[1];
    double targetZ = sigema7_pos_default[2];

    // 沿着初始位置每隔10毫秒，移动500个单位，每次沿着x方向移动0.0001m，y方向移动0.0001m，z方向移动0.0001m，直到移动到目标位置
    int moveCount = 50;
    double moveStepX = 0.0005;
    double moveStepY = 0.0005;
    double moveStepZ = 0.0005;

    for (int i = 0; i < moveCount; i++) {
        // 计算新的位置
        targetX = targetX + moveStepX;
        // double targetY = targetY + moveStepY;
        // double targetZ = targetZ + moveStepZ;
        // 移动到新的位置
        int moveResult = drdMoveToPos(targetX, targetY, targetZ);
        if (moveResult!= 0) {
            std::cerr << "Failed to move to the new position!" << std::endl;
            return -1;
        }
        std::cout << "移动到新的位置: " << targetX << ", " << targetY << ", " << targetZ << std::endl;
        drdSleep(0.001);
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
            return -1;
        }
        std::cout << "移动到新的位置: " << targetX << ", " << targetY << ", " << targetZ << std::endl;
        drdSleep(0.001);
    }

    // 停止调节线程
    int stopResult = drdStop();
    if (stopResult != 0) {
        std::cerr << "Failed to stop the regulation thread!" << std::endl;
        return -1;
    }

    return 0;
}