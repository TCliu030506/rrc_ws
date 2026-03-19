
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

    while (true)
    {
        // 提示用户选择模式
        std::cout << "请输入控制模式 ( readj; readp; movej; movep; speedj; speedp; speed off ) ,或输入 END 结束程序: ";
        std::cin >> mode;

        if (mode == "END") {
            std::cout << "程序结束！" << std::endl;
            break;
        }
        else if (mode == "readj") {
            // 处理 readj 模式
            double theta1, theta2, theta3;
            controller.pf_readj(theta1, theta2, theta3);
            std::cout << "当前摆杆角度依次为: " << std::endl;
            std::cout<<"theta1 = "<<theta1<<std::endl;
            std::cout<<"theta2 = "<<theta2<<std::endl;
            std::cout<<"theta3 = "<<theta3<<std::endl;
        }
        else if (mode == "readp") {
            // 处理 readj 模式
            double x, y, z;
            controller.pf_readp(x, y, z);
            std::cout << "当前末端位置为(X,, Y, Z):" << std::endl;
            std::cout << "X: " << x << ", Y: " << y << ", Z: " << z << std::endl;

        }
        else if (mode == "movej") {
            // 处理 movej 模式
            double x, y, z, t;
            std::cout << "请输入目标角度 x, y, z 和运动时间 t (例如：30 30 30 1): ";
            std::cin >> x >> y >> z >> t;

            // 执行 movej 函数
            controller.pf_movej(x, y, z, t);
        }
        else if (mode == "movep") {
            // 处理 movep 模式
            double x, y, z, t;
            std::cout << "请输入目标位置 x, y, z 和运动时间 t(例如：8 -12 -65 2): ";
            std::cin >> x >> y >> z >> t;

            // 执行 movep 函数
            controller.pf_movep(x, y, z,t);
        }
        else if (mode == "speedj") {
            // 处理 movej 模式
            EV3d vel_theta;
            std::cout << "请输入关节速度 vel_theta1, vel_theta2, vel_theta3 (例如：0.05 0.1 0.15): ";
            std::cin >> vel_theta[0] >> vel_theta[1] >> vel_theta[2] ;

            // 执行 speedj 函数
            controller.pf_speedj(vel_theta);
            
            sleep(1);
            controller.pf_speed_off();

        }
        else if (mode == "speedp") {
            // 处理 speedp 模式
            EV3d vel_end;
            std::cout << "请输入末端速度 vel_x, vel_y, vel_z (例如：0.1 0.2 0.3): ";
            std::cin >> vel_end[0] >> vel_end[1] >> vel_end[2] ;

            // 执行 speedp 函数
            int count = 0;
            while(count++<100){
                controller.pf_speedp(vel_end);
                sleep(0.1);
            }
            controller.pf_speed_off();
        }
        else if(mode == "speed off"){
            // 关闭速度控制模式
            std::cout << "关闭速度控制模式 "<< std::endl;
            controller.pf_speed_off();
        }


        else {
            std::cout << "无效的模式，请重新选择。" << std::endl;
        }
    }

    return 0;
}
