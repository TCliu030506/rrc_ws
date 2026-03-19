import rclpy
import numpy as np
import ast
import time
from .x6_client import x6_clients   

def input_read(x6_input:str):
    """
    从终端读取 [x,y,z,roll,pitch,yaw] 格式的数据
    返回pose (list of float)
    """
    values = x6_input.strip("[]").split(",")
    values = [float(v.strip()) for v in values]

    pose = values[0:]
    pose = np.array(pose).T
    return pose

def check_input(x6_input: str):
    """
    检查输入是否符合要求：[int/float,int/float,...] 共6个元素
    返回合法的list，否则返回 None
    """
    try:
        values = ast.literal_eval(x6_input)
    except Exception:
        return None

    if not (isinstance(values, list) and len(values) == 6 and all(isinstance(v, (int, float)) for v in values)):
        return None
    
    return values

def main(args=None):
    rclpy.init(args=args)
    x6 = x6_clients()

    try:
        print("测试开始")
        while rclpy.ok():
            mode = input("mode ")
            if mode == "setzero":
                x6.set_zero()
            if mode == "unlock":
                x6.unlock()  
            if mode == "lock":
                x6.lock()   
            if mode == "move":
                x6_input = input("请输入电机位姿")
                #示例：[0,0,40,0,0,0]最多三位小数
                flag = check_input(x6_input)
                if flag is None:
                    print("输入格式错误，请重新输入")
                    continue

                pose = input_read(x6_input).astype(float)
                flag2 = x6.check_input(pose)
                if flag2 is None:
                    print("无解，请重新输入")
                    continue
                x6.move_abs(180,pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])

            if mode == "mode1":
                l=10
                x6.move_abs(180,l,0,30,0,0,0)
                time.sleep(2)
                x6.move_abs(180,0,l,30,0,0,0)
                time.sleep(2)
                x6.move_abs(180,-l,0,30,0,0,0)
                time.sleep(2)
                x6.move_abs(180,0,-l,30,0,0,0)

            if mode == "mode2":
                for i in range(0,10):
                    x6.move_abs(360,0,0,30,0,0,0)
                    time.sleep(1)
                    x6.move_abs(360,0,0,30,0,0,-40)
                    time.sleep(1)
                
    except KeyboardInterrupt:
        print("\n用户中断，正在关闭...")

    finally:
        x6.destroy_node()
        rclpy.shutdown()
   

if __name__ == '__main__':
    
    main()