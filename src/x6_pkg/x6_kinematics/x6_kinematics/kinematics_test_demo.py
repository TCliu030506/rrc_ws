# -*- coding: utf-8 -*-
#!/usr/bin/env python3

"""
Demo of Kinematics solver for parallel mechanism X6
----------------------------------------
author: Yunjiang Wang
email:  yunjiang.wang@zju.edu.cn
For more information, please refer to kinematics_X6.py
"""

import kinematics_X6 as X6
import numpy as np
import timeit

## 说明：
# 逆运动学函数：X6_Kine.invKine_pose_axang(pose_axang[0:3],pose_axang[3:6])
# 返回True/False, True表示有解，解存在X6_Kine.solution

# 正运动学函数：X6_Kine.forKine_pose_axang(q6_[0:3],q6_[3:6])
# 返回True/False, True表示有精确解，误差小于0.1°.不管解是否精确，存在X6_Kine.solution

# 测试算法性能
theta_test = np.random.rand(3)  * 30.0 + 10.0
phi_test = np.random.rand(3) * 20.0 + 20.0
joint_test = X6.X6Joint.from_urs_sru(theta_test, phi_test, degrees=True)
# 生成X6求解器实例
X6_Kine_test = X6.X6_solver()

def main(args=None):
    # X6.ExRot示例
    print("X6.ExRot示例:")
    np.set_printoptions(precision=4, suppress=True)
    # 通过坐标角表示建立旋转矩阵
    rot = X6.ExRot.from_rp(np.array([1.,0.,1.]))
    print("旋转矩阵：")
    print(rot.as_matrix())
    # 转化为角轴表示
    axang = rot.as_rotvec()
    print("角轴表示：")
    print(axang)
    # 通过角轴表示建立旋转矩阵并转化为四元数表示
    q4 = X6.ExRot.from_rotvec(np.array([1.,0.,0.])).as_quat()
    print("四元数表示：")
    print(q4)

    # X6.X6_solver示例
    print("\nX6.X6_solver示例:")
    X6_Kine = X6.X6_solver()
    # 参考: 0, 0, 40mm, 0, 0, 1°
    pose = X6.ExPose(np.array([0., 0., 40.]), X6.ExRot.from_rp(np.array([0., 0., 1.]), degrees=True))

    # 提示
    print(f"Solve inverse kinematics: pose_rotvec = ")
    print(pose.pose_rotvec(True).flatten())

    # 判断是否有解
    q6 = X6_Kine.inverse_kinematics(pose)
    if q6 is None:
        print(X6_Kine.error)
    else:
        print('Solved as:')
        print(q6.joint(True).flatten())
        
        # 提示
        print(f"\nSolve forward kinematics: q6 = {q6.joint(True).flatten()}")

        # 求解
        if (pose_out := X6_Kine.forward_kinematics(q6, precise=False)) is None:
            return
        print(X6_Kine.error)

        print('Solved as:')
        print(pose_out.pose_rotvec(True).flatten())

    # 测试算法性能
    print('\n算法性能测试: \n当前关节位置:')
    print(joint_test.joint(True).flatten())
    try:
        pose_test = X6_Kine_test.forward_kinematics(joint_test)
    except Exception as e:
        print(f"正运动学计算无解: {e}")
        return
    print('对应末端位姿（位置+坐标角）:')
    print(pose_test.pose_rp(True).flatten())
    X6_Kine_test.forward_kinematics( X6.X6Joint.from_urs_sru(np.array([40., 40., 40.]), np.array([30., 30., 30.]), degrees=True) )
    for i in range(6):
        if i == 0:
            continue
        execution_time = timeit.timeit(stmt="X6_Kine_test.forward_kinematics(joint_test)", globals=globals(), number=1)
        print(f"正运动学计算时间测试第 {i} 次，时间： {execution_time:.6f} 秒")

if __name__ == '__main__':

    main()    