///////////////////////////////////////////////////////////////////////////////
//
//  Copyright Copyright (C) 2001-2021 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  Force Dimension SDK 3.14.0
//
///////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "sigema7_pkg/dhdc.h"
#include "sigema7_pkg/drdc.h"
#include <unistd.h>
#include <fstream>
#include <sstream>
#include<iostream>


#define REFRESH_INTERVAL  0.1   // sec

using namespace std;

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
    drdStop(true);
}

//从设备中读取位姿信息，保存到函数输入数组中
//单位： x\y\z m,rx oa \ry ob \rz og rad
void read_pos(double *handler_pose_)
{
    dhdGetPositionAndOrientationRad(&handler_pose_[0], &handler_pose_[1], &handler_pose_[2],&handler_pose_[3], &handler_pose_[4], &handler_pose_[5]); 

}

int main (int  argc,char **argv)
{
 
    //初始量设置
    double handler_pose_[6];
    // 主端手控器初始化
    MasterDeviceInit(); 
    //
    std::cout << "这是一行测试语句"<< std::endl;

    //循环读取并打印数据
    while (1)
    {
        if (dhdSetForceAndWristJointTorquesAndGripperForce(0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0) == 0) printf("成功设定初始力、关节扭矩和夹爪力\n");

        read_pos(handler_pose_);
        std::cout << "x, y, z = " << handler_pose_[0] << ' ' << handler_pose_[1] << ' ' << handler_pose_[2] << std::endl;
	    std::cout << "rx, ry, rz = " << handler_pose_[3] << ' ' << handler_pose_[4] << ' ' << handler_pose_[5] << std::endl;

    /*		
        std::string global_pose_fileName0;
        stringstream ss_fileName0;
        ss_fileName0<<" /home/sun/haptic_handle/scan_ws/src/example_test/files/lab_5_2/oa/test.txt";
        ss_fileName0>>global_pose_fileName0;
	 fstream outFile_target0;
        outFile_target0.open(global_pose_fileName0,ios::out |ios::app);
        if(!outFile_target0)
        {
            std::cerr<<"---------------new file failed!"<<std::endl;
        }
			outFile_target0<< handler_pose_[0]*0.8<<" "<<handler_pose_[1]*0.8<<" "<<handler_pose_[2]*0.8<<" "<<handler_pose_[3]*0.5<<" "<<handler_pose_[4]*0.5<<" "<<handler_pose_[5]*0.5<<"\n";
		
		   
        outFile_target0.close();
 
        std::string global_pose_fileName1;
        stringstream ss_fileName1;
        ss_fileName1<<" /home/sun/haptic_handle/scan_ws/src/example_test/files/lab_5/2_d_lab1.txt";
        ss_fileName1>>global_pose_fileName1;
	    fstream outFile_target1;
        outFile_target1.open(global_pose_fileName1,ios::out |ios::app);
        if(!outFile_target1)
        {
            std::cerr<<"---------------new file failed!"<<std::endl;
        }
		outFile_target1<< handler_pose_[0]*1.5<<" "<<handler_pose_[1]*1.5<<" "<<handler_pose_[2]*1.5<<" "<<handler_pose_[3]*1.0<<" "<<handler_pose_[4]*1.2<<" "<<handler_pose_[5]*1.4<<"\n";
		
		   
        outFile_target1.close();
			
        std::string global_pose_fileName2;
        stringstream ss_fileName2;
        ss_fileName2<<" /home/sun/haptic_handle/scan_ws/src/example_test/files/lab_5/1_d_lab2.txt";
        ss_fileName2>>global_pose_fileName2;
	    fstream outFile_target2;
        outFile_target2.open(global_pose_fileName2,ios::out |ios::app);
        if(!outFile_target2)
        {
            std::cerr<<"---------------new file failed!"<<std::endl;
        }
			outFile_target2<< handler_pose_[0]*0.5<<" "<<handler_pose_[1]*0.5<<" "<<handler_pose_[2]*0.5<<" "<<handler_pose_[3]*0.2<<" "<<handler_pose_[4]*0.4<<" "<<handler_pose_[5]*0.6<<"\n";
		
		   
        outFile_target2.close();
	
        std::string global_pose_fileName3;
        stringstream ss_fileName3;
        ss_fileName3<<" /home/sun/haptic_handle/scan_ws/src/example_test/files/lab_5/1_d_lab3.txt";
        ss_fileName3>>global_pose_fileName3;
	    fstream outFile_target3;
        outFile_target3.open(global_pose_fileName3,ios::out |ios::app);
        if(!outFile_target3)
        {
            std::cerr<<"---------------new file failed!"<<std::endl;
        }
			outFile_target3<< handler_pose_[0]*2<<" "<<handler_pose_[1]*2<<" "<<handler_pose_[2]*2<<" "<<handler_pose_[3]*0.2<<" "<<handler_pose_[4]*0.4<<" "<<handler_pose_[5]*0.6<<"\n";
        outFile_target3.close();
	
    */

        usleep(50 * 1000);

   }
  return 0;
}


