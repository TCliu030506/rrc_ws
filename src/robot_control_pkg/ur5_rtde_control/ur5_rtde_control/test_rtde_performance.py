import time
import numpy as np
from ur_rtde import RTDEControl, RTDEReceive

ROBOT_IP = "192.168.1.102"  # 修改为你的UR机器人IP
FREQUENCIES = [10, 50, 100, 125, 200, 250, 500, 750, 1000]  # Hz
TEST_DURATION = 10  # 每个频率测试时长（秒）


def test_rtde_performance():
    print("连接到UR机器人...", ROBOT_IP)
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    print("连接成功！")

    results = []
    for freq in FREQUENCIES:
        print(f"\n测试频率: {freq} Hz")
        period = 1.0 / freq
        timestamps = []
        delays = []
        lost = 0
        start_time = time.time()
        count = 0
        while time.time() - start_time < TEST_DURATION:
            t0 = time.time()
            try:
                # 这里只做简单的getActualQ()，可换成moveJ等命令
                _ = rtde_r.getActualQ()
                t1 = time.time()
                delay = t1 - t0
                delays.append(delay)
                timestamps.append(t0)
            except Exception as e:
                print(f"通信异常: {e}")
                lost += 1
            count += 1
            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)
        if delays:
            delays_np = np.array(delays)
            result = {
                "freq": freq,
                "count": count,
                "lost": lost,
                "max_delay": np.max(delays_np),
                "min_delay": np.min(delays_np),
                "avg_delay": np.mean(delays_np),
                "std_delay": np.std(delays_np),
            }
        else:
            result = {"freq": freq, "count": count, "lost": lost, "max_delay": None, "min_delay": None, "avg_delay": None, "std_delay": None}
        results.append(result)
        print(f"频率: {freq}Hz, 总包数: {count}, 丢包: {lost}, 最大延时: {result['max_delay']:.6f}s, 最小延时: {result['min_delay']:.6f}s, 平均延时: {result['avg_delay']:.6f}s")

    print("\n=== 测试结果汇总 ===")
    for r in results:
        print(r)

    rtde_c.disconnect()
    rtde_r.disconnect()

if __name__ == "__main__":
    test_rtde_performance()
