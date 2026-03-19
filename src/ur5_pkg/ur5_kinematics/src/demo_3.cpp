//利用逆运动学实现直线运动的数学求解
////输入初始位置和目标位置，输出直线运动中各个插值点（100个）处的6个关节角度解
////注：此程序设置了虚拟的初始关节角度，且无旋量。

#include "../../ur5_kinematics/include/ur5_kinematics.h"

int main(int argc, char **argv)
{
    //定义初始量
    double target_p[3];
    double initial_p[3];
    double middle_p[3];
    double joint_target[6];
    double joint_initial[6];
    double joint_middle[6];
    double joint_base[6];

    //URposition类的实例化
    URposition ur_position;

    // 创建一个 Eigen::Isometry3d 对象
    EIso3d mat_initial = Eigen::Isometry3d::Identity();
    EIso3d mat_middle = Eigen::Isometry3d::Identity();
    // 将变换矩阵设置为单位矩阵（无旋量）
    mat_initial.setIdentity();
    mat_middle.setIdentity();

    //设置运动之初六个关节的位置
    joint_base[0]=1.57;
    joint_base[1]=1.57;
    joint_base[2]=0;
    joint_base[3]=-1.57;
    joint_base[4]=-1.57;
    joint_base[5]=0;

    //用户输入获取直线运动的起始点坐标
    std::cout << "请逐个输入直线运动初始位置的x,y,z坐标（单位：m）"  << std::endl;
	for (int i = 0; i < 3; ++i) {
        std::cin >> initial_p[i];
    }

    std::cout << "请逐个输入直线运动目标位置的x,y,z坐标（单位：m）"  << std::endl;
	for (int i = 0; i < 3; ++i) {
        std::cin >> target_p[i];
    }

    //将机械臂运动至初始位置
    // 设置平移向量
    Eigen::Vector3d initial_translation(initial_p[0],initial_p[1],initial_p[2]);      //起点的(X,Y,Z)
    mat_initial.translation() = initial_translation;
    //逆运动学计算，得到直线初始位置关节角度
    ur_position.dhinvkinematics(mat_initial, joint_initial, joint_base);
    //发布关节角度使机械臂运动至直线起点-----------此处用打印出关节角度代替
    std::cout << "直线运动初始位置关节角度依次为" << std::endl;
    for (int i = 0; i < 6; ++i) {
        std::cout << joint_initial[i] << std::endl;
    }
    
    //将中间位置初始值赋值为运动起始点
    middle_p[0] = initial_p[0];
    middle_p[1] = initial_p[1];
    middle_p[2] = initial_p[2];

    //每次运动的起点关节角度为上一次运动的目标关节角度，初始值为直线起点关节角度
    for (int i = 0; i < 6; ++i) {
        joint_middle[i] = joint_initial[i];
    }

    //将运动分为100段运动
    int segments = 100;
    double stepX = (target_p[0] - initial_p[0] ) / segments;
    double stepY = (target_p[1] - initial_p[1]) / segments;
    double stepZ = (target_p[2] - initial_p[2]) / segments;
    
    for (int i = 0; i < segments; ++i) {
        //求出第i段直线终点坐标
        target_p[0] =  middle_p[0] + stepX;
        target_p[1] =  middle_p[1] + stepY;
        target_p[2] =  middle_p[2] + stepZ;
        //求出第i段运动重点的关节角度
        Eigen::Vector3d middle_translation(target_p[0],target_p[1],target_p[2]);      //起点的(X,Y,Z)
        mat_middle.translation() = middle_translation;
        ur_position.dhinvkinematics(mat_middle, joint_target, joint_middle);
        std::cout << "直线运动第" << i+1 << "个目标点对应的关节角度依次为：" <<std::endl;
        for (int j = 0; j < 6; ++j) {
            std::cout << joint_target[j] << std::endl;
        }
        //将终点关节角度作为下一段运动的初始关节角度
        for (int j = 0; j < 6; ++j) {
            joint_middle[j] = joint_target[j];
        }
        middle_p[0] = target_p[0];
        middle_p[1] = target_p[1];
        middle_p[2] = target_p[2];
    }

    std::cout << "直线运动完成" << std::endl;
    
	return 0;
}