# -*- coding: utf-8 -*-
from __future__ import annotations

import time
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node

from xmate_cr7_script_py.cr7_script import Cr7ScriptClient


class DGROKAE(Node):
    """
    简单机械臂控制节点（Rokae）：
    - 设置初始位姿 set_initial_pose
    - 以初始位姿为参考做相对位移 move_relative_from_initial
    - 在 control_loop 中通过终端交互按 1 执行、按 q 退出
    """

    def __init__(self) -> None:
        super().__init__('dg_rokae')

        self.cr7_client = Cr7ScriptClient()

        self.default_vel = 50.0
        self.default_acc = 30.0
        self.settle_ms = 2000  # 每次运动后等待时间（毫秒）

        # 可选：代码里预先给一个“默认初始位姿”，方便直接使用
        # 如果你不想要默认初始位姿，可以设为 None
        self._initial_pose: Optional[List[float]] = [
            0.68, -0.42, 0.4,
            90.0 * math.pi / 180.0,
            0.0,
            0.0
        ]

        # self._initial_pose: Optional[List[float]] = [
        #     0.7, 0.2, 0.3,
        #     90.0 * math.pi / 180.0,
        #     -90.0 * math.pi / 180.0,
        #     90.0 * math.pi / 180.0,
        # ]

        self.move_steps = [
            # (0.00, 0.00, -0.138),  # pin
            # (0.00, 0.00, 0.00),
            # (0.00, 0.00, -0.11),   # 砝码
            # (0.00, 0.0, 0.0),
            (0.00, 0.00, -0.153),  # cut
            (0.00, 0.00, 0.00),
        ]

        self._step_index = 0

    def _sleep(self, ms: int) -> None:
        time.sleep(ms / 1000.0)

    # 设置 / 移动至初始位姿
    def set_initial_pose(self,
                         pose_xyz_rpy: Optional[List[float]] = None,
                         vel: Optional[float] = None,
                         acc: Optional[float] = None) -> List[float]:
        """
        设置“初始位姿”：
        - 若 pose_xyz_rpy 为 None：
            使用当前 _initial_pose（若已有）；如果还没有，报错提示。
        - 若 pose_xyz_rpy 为给定 [x,y,z,rx,ry,rz]：
            将其记录为初始位姿，并通过 movep 运动到该位姿。

        返回：self._initial_pose
        """
        # 情况 1：没有传入新的初始位姿
        if pose_xyz_rpy is None:
            if self._initial_pose is None:
                raise RuntimeError("尚未设置初始位姿，请先传入 pose_xyz_rpy。")
            pose_xyz_rpy = self._initial_pose
        else:
            if len(pose_xyz_rpy) != 6:
                raise ValueError("pose_xyz_rpy 必须是长度为6的列表 [x,y,z,rx,ry,rz]")
            self._initial_pose = list(pose_xyz_rpy)

        v = self.default_vel if vel is None else float(vel)
        a = self.default_acc if acc is None else float(acc)
        ok, res = self.cr7_client.movep(self._initial_pose, v, a)
        if not ok:
            raise RuntimeError(f"movep 到初始位姿失败: {res}")

        self._sleep(self.settle_ms)
        return self._initial_pose

    # 相对“初始位置”移动，姿态保持不变（使用初始姿态）
    def move_relative_from_initial(self,
                                   dx: float,
                                   dy: float,
                                   dz: float,
                                   vel: Optional[float] = None,
                                   acc: Optional[float] = None) -> None:
        """
        以“初始位姿”的位置为参考，移动到 (x0+dx, y0+dy, z0+dz)，并保持初始姿态不变。
        只支持保持初始姿态（简化版）。
        """
        if self._initial_pose is None:
            raise RuntimeError("未设置初始位姿，请先调用 set_initial_pose()")

        x0, y0, z0, rx0, ry0, rz0 = self._initial_pose

        target = [
            x0 + float(dx),
            y0 + float(dy),
            z0 + float(dz),
            rx0, ry0, rz0
        ]

        v = self.default_vel if vel is None else float(vel)
        a = self.default_acc if acc is None else float(acc)
        ok, res = self.cr7_client.movep(target, v, a)
        if not ok:
            raise RuntimeError(f"movep 相对移动失败: {res}")

        self._sleep(self.settle_ms)

    def control_loop(self) -> None:
        print("===== DGROKAE 控制循环启动 =====")
        print("按键说明：")
        print("  i  - 设置/移动到初始位姿")
        print("  1  - 执行下一步相对位移（自动从 move_steps 读取）")
        print("  q  - 退出程序")

        print(f"[Info] 默认初始位姿: {self._initial_pose}")
        print(f"[Info] 相对位移序列：{self.move_steps}")

        try:
            while True:
                cmd = input("\n请输入指令 (i/1/q)：").strip().lower()

                if cmd == 'q':
                    print("[Loop] 退出控制循环。")
                    break

                elif cmd == 'i':
                    print(f"[Loop] 正在移动到初始位姿: {self._initial_pose}")
                    self.set_initial_pose(self._initial_pose)
                    print("[Loop] 已到达初始位姿。")

                elif cmd == '1':
                    dx, dy, dz = self.move_steps[self._step_index]
                    print(f"[Loop] 执行第 {self._step_index + 1} 步相对位移: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")

                    self.move_relative_from_initial(dx, dy, dz)
                    print("[Loop] 相对位移完成。")

                    self._step_index += 1
                    if self._step_index >= len(self.move_steps):
                        print("\n[Loop] 已完成所有 move_steps！")
                        cont = input("是否继续新一轮？(1 继续 / q 退出)：").strip().lower()
                        if cont == '1':
                            self._step_index = 0
                        else:
                            print("[Loop] 用户选择退出。")
                            break

                else:
                    print("无效指令，请输入 i / 1 / q。")

        except KeyboardInterrupt:
            print("\n[Loop] 检测到 Ctrl+C，退出。")


def main() -> None:
    rclpy.init()
    node = DGROKAE()
    try:
        node.control_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
