import time

from rtde_io import RTDEIOInterface as RTDEIO
from rtde_receive import RTDEReceiveInterface as RTDEReceive


ROBOT_IP = "192.168.1.102"
TOOL_DO_INDEX = 0
HALF_PERIOD_SEC = 0.05


def main():
    rtde_io = RTDEIO(ROBOT_IP)
    rtde_receive = RTDEReceive(ROBOT_IP)

    try:
        while True:
            rtde_io.setToolDigitalOut(TOOL_DO_INDEX, True)
            time.sleep(HALF_PERIOD_SEC)
            # 读取工具数字输出状态
            tool_do_state = rtde_receive.getDigitalOutState(16)
            print(f"Tool digital out (16) state: {'HIGH' if tool_do_state else 'LOW'}")
            
            rtde_io.setToolDigitalOut(TOOL_DO_INDEX, False)
            time.sleep(HALF_PERIOD_SEC)
            # 读取工具数字输出状态
            tool_do_state = rtde_receive.getDigitalOutState(16)
            print(f"Tool digital out (16) state: {'HIGH' if tool_do_state else 'LOW'}")
            
    except KeyboardInterrupt:
        pass
    finally:
        rtde_io.setToolDigitalOut(TOOL_DO_INDEX, False)


if __name__ == "__main__":
    main()
