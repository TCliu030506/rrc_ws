import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
# from ur_rtde import RTDEControl, RTDEReceive

class RTDEPerformanceNode(Node):
    def __init__(self):
        super().__init__('rtde_performance_node')
        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('test_duration', 10)
        self.declare_parameter('frequencies', [10, 50, 100, 125, 200, 250, 500, 750, 1000])

        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.test_duration = self.get_parameter('test_duration').get_parameter_value().integer_value
        self.frequencies = self.get_parameter('frequencies').get_parameter_value().integer_array_value

        self.result_pub = self.create_publisher(String, 'rtde_performance_result', 1)
        self.get_logger().info('RTDE性能测试节点启动')
        self.run_test()

    def run_test(self):
        # rtde_c = RTDEControl(self.robot_ip)
        # rtde_r = RTDEReceive(self.robot_ip)
        for freq in self.frequencies:
            period = 1.0 / freq
            delays = []
            lost = 0
            count = 0
            start_time = time.time()
            while time.time() - start_time < self.test_duration:
                t0 = time.time()
                try:
                    # _ = rtde_r.getActualQ()
                    time.sleep(0.001)  # 模拟
                    t1 = time.time()
                    delay = t1 - t0
                    delays.append(delay)
                except Exception:
                    lost += 1
                count += 1
                elapsed = time.time() - t0
                if elapsed < period:
                    time.sleep(period - elapsed)
            if delays:
                delays_np = np.array(delays)
                msg = String()
                msg.data = (f"freq: {freq}Hz, count: {count}, lost: {lost}, "
                            f"max: {np.max(delays_np):.6f}s, min: {np.min(delays_np):.6f}s, "
                            f"avg: {np.mean(delays_np):.6f}s, std: {np.std(delays_np):.6f}s")
                self.result_pub.publish(msg)
        # rtde_c.disconnect()
        # rtde_r.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = RTDEPerformanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
