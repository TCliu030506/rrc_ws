//测试了逆运动学求解。
////输入目标位置的坐标，输出6个关节角度的一组解。
////注：此程序设置了虚拟的初始关节角度，且无旋量。

#include "../../ur5_kinematics/include/ur5_kinematics.h"


int main(int argc, char **argv)
{
    double p[3];
    double joint_target[6];
    double joint_base[6];

    joint_base[0] = 1.57;
    joint_base[1] = 1.57;
    joint_base[2] = 0;
    joint_base[3] = -1.57;
    joint_base[4] = -1.57;
    joint_base[5] = 0;

    std::cout << "请逐个输入目标位置的x,y,z坐标（单位：m）"  << std::endl;
	for (int i = 0; i < 3; ++i) {
        std::cin >> p[i];
    }

    // 创建一个 Eigen::Isometry3d 对象
    EIso3d mat = Eigen::Isometry3d::Identity();

    // 将变换矩阵设置为单位矩阵（无旋量）
    mat.setIdentity();

    // 设置平移向量
    Eigen::Vector3d translation(p[0], p[1], p[2]);
    mat.translation() = translation;

    URposition ur_position;
    ur_position.dhinvkinematics(mat, joint_target, joint_base);
    
    // 输出关节角度
    std::cout << "关节角度依次为" << std::endl;
    
    for (int i = 0; i < 6; ++i) {
        std::cout << joint_target[i] << std::endl;
    }
    
	return 0;
}