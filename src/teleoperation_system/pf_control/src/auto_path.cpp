
#include "../include/pf_control/pf_control.h"
#include <iostream>
#include <string>

int main()
{
    pf_control controller("/dev/ttyUSB0");
    std::string mode;
    std::string input;

    while (true)
    {
        // 提示用户选择模式
        std::cout << "请输入控制模式: 直线运动（movel） 或 圆周运动（movec）)，或输入 END 结束程序: ";
        std::cin >> mode;

        if (mode == "END") {
            std::cout << "程序结束！" << std::endl;
            break;
        }
        else if (mode == "movel") {
            // 处理 movel 模式
            // 目前仍有一些问题！
            double start_x, start_y, start_z,end_x, end_y, end_z, step, t;
            std::cout << "请输入直线运动始末位置、运动步长、运动时间(例如： 8 -12 -65  -10 15 -75 1 5): ";
            std::cin >> start_x >> start_y >> start_z >> end_x >> end_y >> end_z >> step >> t;

            // 执行 movec 函数
            controller.pf_movel(start_x, start_y, start_z,end_x, end_y, end_z, step, t);
        }
        else if (mode == "movec") {
            std::cout << "暂未开发 ";
        }
        else {
            std::cout << "无效的模式，请重新选择。" << std::endl;
        }
    }

    return 0;
}
