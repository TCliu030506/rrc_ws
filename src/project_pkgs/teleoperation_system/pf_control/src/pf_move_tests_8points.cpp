
#include "../include/pf_control/pf_control.h"
#include <iostream>
#include <string>

typedef Eigen::Matrix3d           EM3d;
typedef Eigen::Vector3d           EV3d;

int main()
{
    pf_control controller("/dev/ttyUSB0");
    std::string mode;
    std::string input;
    std::cout << "欢迎使用摆杆控制测试程序！" << std::endl;

    // 定义存储八个点位的变量
    std::vector<EV3d> points = {
            EV3d(55, 55, 55),   // 点位1
            EV3d(85, 55, 30),   // 点位2
            EV3d(55, 30, 85),   // 点位3
            EV3d(30, 85, 55),   // 点位4
            EV3d(75, 60, 45),   // 点位5
            EV3d(60, 45, 75),   // 点位6
            EV3d(45, 75, 60),   // 点位7
            EV3d(55, 55, 55)    // 点位8
        };

    // 循环索引点位
    for(int i=0; i<8; i++){
        // 移动至下一个点位
        std::cout << "移动至点位 " << (i+1) << ": (" 
                  << points[i][0] << ", " 
                  << points[i][1] << ", " 
                  << points[i][2] << ")" << std::endl;
        // 执行 movej 函数
        controller.pf_movej(points[i][0], points[i][1], points[i][2], 5);
        // 等待5秒
        sleep(2);
    }

    // 程序结束
    std::cout << "程序结束！" << std::endl;

    return 0;
}
