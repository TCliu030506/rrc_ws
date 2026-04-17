import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float64MultiArray

from ur5_rtde_control.ur5_rtde_control import URCONTROL


class TeleoperationControlUIExecutor(Node):
    """订阅[x,y,z,rx,ry,rz]目标位姿并实时调用UR控制。"""

    _ACTIVE_CONTROL_SIGNALS = (2, 3, 10)

    def __init__(self):
        super().__init__('teleoperation_control_ui_executor')

        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('target_cmd_topic', '/ur5/target_cmd')
        self.declare_parameter('tcp_state_topic', '/ur5/tcp_state')
        self.declare_parameter('publish_rate_hz', 125.0)
        self.declare_parameter('command_timeout_sec', 0.2)

        robot_ip = self.get_parameter('robot_ip').value
        target_cmd_topic = self.get_parameter('target_cmd_topic').value
        tcp_state_topic = self.get_parameter('tcp_state_topic').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)

        self.ur = URCONTROL(str(robot_ip))

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Float64MultiArray,
            target_cmd_topic,
            self._target_cmd_callback,
            qos_profile,
        )
        self._tcp_state_pub = self.create_publisher(
            Float64MultiArray,
            tcp_state_topic,
            qos_profile,
        )

        self.get_logger().info(
            f'Subscribing target command topic: {target_cmd_topic}, publish tcp state topic: {tcp_state_topic}, robot_ip={robot_ip}, '
            f'command_timeout_sec={self._command_timeout_sec}'
        )

        self._period = 1.0 / publish_rate_hz if publish_rate_hz > 1e-6 else 0.01
        self._running = True
        self._lock = threading.Lock()
        self._latest_target_pose = None
        self._last_cmd_time = 0.0
        self._control_signal = None
        self._last_timeout_warn_time = 0.0

        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._send_thread.start()

    def _target_cmd_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 7:
            return

        control_signal = int(msg.data[0])
        target_pose = [
            float(msg.data[1]),
            float(msg.data[2]),
            float(msg.data[3]),
            float(msg.data[4]),
            float(msg.data[5]),
            float(msg.data[6]),
        ]

        with self._lock:
            self._control_signal = control_signal
            if control_signal in (6, 7):
                self._control_signal = 3
            if self._control_signal not in self._ACTIVE_CONTROL_SIGNALS:
                self._latest_target_pose = None
                self._last_cmd_time = 0.0
                return

            self._latest_target_pose = target_pose
            self._last_cmd_time = time.monotonic()

    def _publish_tcp_state(self):
        try:
            current_tcp_pose = self.ur.get_tcp_pose()
        except AttributeError:
            return
        except Exception as exc:
            self.get_logger().warn(f'读取TCP状态失败，跳过本次发布: {exc}')
            return

        if current_tcp_pose is None or len(current_tcp_pose) != 6:
            return

        msg = Float64MultiArray()
        msg.data = [float(value) for value in current_tcp_pose]
        self._tcp_state_pub.publish(msg)

    def _send_loop(self):
        while rclpy.ok() and self._running:
            now = time.monotonic()
            with self._lock:
                target_pose = None if self._latest_target_pose is None else list(self._latest_target_pose)
                control_signal = self._control_signal
                last_cmd_time = self._last_cmd_time

            # 仅在 control_signal 为 2、3 或 10 时允许下发
            if control_signal not in self._ACTIVE_CONTROL_SIGNALS:
                self._publish_tcp_state()
                time.sleep(self._period)
                continue

            # 防止网络抖动/发布端停更时重复下发陈旧命令
            if target_pose is None or (now - last_cmd_time) > self._command_timeout_sec:
                if (now - self._last_timeout_warn_time) > 1.0:
                    self._last_timeout_warn_time = now
                    self.get_logger().warn(
                        'Target command stale or missing, skip sending until fresh command arrives.'
                    )
                self._publish_tcp_state()
                time.sleep(self._period)
                continue

            try:
                self.ur.sevol_l(target_pose)
            except AttributeError:
                self.get_logger().warn('URCONTROL.sevol_l() 未实现，无法下发目标位姿。')
            except Exception as exc:
                self.get_logger().error(f'sevol_l failed: {exc}')

            time.sleep(self._period)

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_send_thread') and self._send_thread.is_alive():
            self._send_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationControlUIExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
