import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
import rtde_control
import rtde_receive

class RTDEPerformanceNode(Node):
    def __init__(self):
        super().__init__('rtde_performance_node')
        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('test_duration', 10)
        self.declare_parameter('frequencies', [10, 50, 100, 125, 200, 250, 500, 750, 1000])
        self.declare_parameter('simulated_work_ms', 0.0)

        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.test_duration = self.get_parameter('test_duration').get_parameter_value().integer_value
        self.frequencies = self.get_parameter('frequencies').get_parameter_value().integer_array_value
        self.simulated_work_ms = self.get_parameter('simulated_work_ms').get_parameter_value().double_value

        self.result_pub = self.create_publisher(String, 'rtde_performance_result', 1)
        self.get_logger().info('RTDE性能测试节点启动')
        self.run_test()

    def run_test(self):
        rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        for freq in self.frequencies:
            period = 1.0 / freq
            read_times = []
            loop_times = []
            lost = 0
            count = 0
            deadline_miss = 0
            fresh_count = 0
            stale_count = 0
            last_ts = None
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < self.test_duration:
                t0 = time.perf_counter()
                try:
                    _ = rtde_r.getActualQ()
                    read_cost = time.perf_counter() - t0
                    read_times.append(read_cost)

                    if self.simulated_work_ms > 0.0:
                        time.sleep(self.simulated_work_ms / 1000.0)

                    try:
                        curr_ts = rtde_r.getTimestamp()
                        if last_ts is not None:
                            if curr_ts > last_ts:
                                fresh_count += 1
                            else:
                                stale_count += 1
                        last_ts = curr_ts
                    except Exception:
                        # 某些 RTDE 固件或接口版本可能不支持时间戳读取。
                        pass
                except Exception:
                    lost += 1

                loop_cost = time.perf_counter() - t0
                loop_times.append(loop_cost)
                if loop_cost > period:
                    deadline_miss += 1

                count += 1

                if loop_cost < period:
                    time.sleep(period - loop_cost)

            if read_times:
                read_np = np.array(read_times)
                loop_np = np.array(loop_times)
                actual_hz = count / max(self.test_duration, 1e-9)
                miss_rate = (deadline_miss / count * 100.0) if count > 0 else 0.0
                freshness_total = fresh_count + stale_count
                freshness_rate = (fresh_count / freshness_total * 100.0) if freshness_total > 0 else -1.0

                msg = String()
                msg.data = (f"freq: {freq}Hz, count: {count}, lost: {lost}, "
                            f"actual: {actual_hz:.1f}Hz, miss: {deadline_miss} ({miss_rate:.2f}%), "
                            f"read_max: {np.max(read_np):.6f}s, read_min: {np.min(read_np):.6f}s, "
                            f"read_avg: {np.mean(read_np):.6f}s, read_std: {np.std(read_np):.6f}s, "
                            f"loop_max: {np.max(loop_np):.6f}s, loop_avg: {np.mean(loop_np):.6f}s, "
                            f"fresh_rate: {freshness_rate:.2f}%")
                self.result_pub.publish(msg)
        rtde_c.disconnect()
        rtde_r.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = RTDEPerformanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
