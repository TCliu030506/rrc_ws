import rclpy
import time
import ur5_control.URControl
from rclpy.node import Node
from ur5_control.URControl import URNode
from rclpy.timer import Timer

lie_down_pos = [0,-175,160,-90,0,0]
work_origin_pos_1 = [0,-90,0,-90,0,0]
work_origin_pos_2 = [0,-90,-90,-90,90,0]

class Ur5PosInit(Node):
    def __init__(self):
        super().__init__('ur5_pos_init')
        # 创建URNode实例
        self.ur_node = URNode()
        # 创建定时器，设置为每秒调用一次timer_callback函数
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global lie_down_pos
        global work_origin_pos_1
        global work_origin_pos_2
        self.lie_down_pos = [0]*6
        self.work_origin_pos_1 = [0]*6
        self.work_origin_pos_2 = [0]*6
        for i in range (6):
            self.lie_down_pos[i] = ur5_control.URControl.deg_to_rad(lie_down_pos[i])
        for i in range(6):
            self.work_origin_pos_1[i] = ur5_control.URControl.deg_to_rad(work_origin_pos_1[i])
        for i in range(6):
            self.work_origin_pos_2[i] = ur5_control.URControl.deg_to_rad(work_origin_pos_2[i])
        key_input = input("请输入一个数字(0 或 1)(0:回到停止姿态)(1:运动到工作姿态):\n")
        if key_input == '0':
            self.ur_node.movej(self.lie_down_pos)
            print("按键0被按下,回到停止姿态")
        elif key_input == '1':
            self.ur_node.movej(self.work_origin_pos_1)
            time.sleep(10)
            self.ur_node.movej(self.work_origin_pos_2)
            print("按键1被按下,回到工作姿态")
        else:
            print("请输入正确的数字(0 或 1)")


def main(args=None):
    rclpy.init(args=args)
    ur5_pos_init = Ur5PosInit()
    rclpy.spin(ur5_pos_init)
    ur5_pos_init.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()