//demo.cpp程序演示了正运动学的使用方法及实现功能
////程序效果：输入六个关节角度（弧度），输出传递矩阵
////调用正运动学的方法参考第19、20行，先实例化类，再调用类内函数 dhfwardkmatics

#include "../../ur5_kinematics/include/ur5_kinematics.h"

int main(){
    double joint[6];
    Eigen::Matrix4d tm_matrix;

    // 输入关节角度
    std::cout << "请输入 " << 6 << " 个关节角度（弧度）" << std::endl;
    for (int i = 0; i < 6; ++i) {
        std::cout << "请输入第 " << i + 1 << " 个关节角度（弧度）：";
        std::cin >> joint[i];
    }

    URposition ur_position; // 创建 URposition 类的对象
    tm_matrix = ur_position.dhfwardkmatics(joint); // 使用对象来调用非静态成员函数


    // 输出传递矩阵元素
    std::cout << "传递矩阵为:" << std::endl;
    std::cout << tm_matrix << std::endl;

    return 0;

}
