"""TG-9801 遥操作节点。

仿照 dg_omni_control_ui.py 的按钮控制思路：
- 订阅 OmniButtonEvent；
- 灰键触发夹取、白键触发松开；
- 通过 set_grip 服务下发动作。
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from omni_msgs.msg import OmniButtonEvent
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


class TG9801TeleoperationNode(Node):
    """通过 Omni 手柄按钮遥操作 TG-9801。"""

    def __init__(self) -> None:
        super().__init__('tg_9801_teleoperation_node')

        # ROS 参数：按钮输入话题、夹爪服务名、按钮电平逻辑与防抖时间。
        self.declare_parameter('button_topic', '/phantom/button')
        self.declare_parameter('service_name', 'set_grip')
        # omni_common 中按钮按下发布为 1，松开为 0，因此默认 active_low=False。
        self.declare_parameter('grey_active_low', False)
        self.declare_parameter('white_active_low', False)
        self.declare_parameter('debounce_sec', 0.15)

        self._button_topic = str(self.get_parameter('button_topic').value)
        self._service_name = str(self.get_parameter('service_name').value)
        self._grey_active_low = bool(self.get_parameter('grey_active_low').value)
        self._white_active_low = bool(self.get_parameter('white_active_low').value)
        self._debounce_sec = max(0.0, float(self.get_parameter('debounce_sec').value))

        # 创建 set_grip 服务客户端，并等待服务就绪。
        self._set_grip_client = self.create_client(SetBool, self._service_name)
        while not self._set_grip_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {self._service_name} service...')

        # 边沿检测与并发请求保护状态。
        self._last_grey_pressed = False
        self._last_white_pressed = False
        self._last_trigger_time = 0.0
        self._pending_future: Optional[rclpy.task.Future] = None

        # 订阅 Omni 按钮事件。
        self.create_subscription(OmniButtonEvent, self._button_topic, self._button_callback, 10)

        self.get_logger().info(
            f'TG9801 teleoperation started | button_topic={self._button_topic} service={self._service_name} '
            f'grey_active_low={self._grey_active_low} white_active_low={self._white_active_low} '
            f'debounce={self._debounce_sec:.2f}s'
        )

    @staticmethod
    def _to_pressed(raw_value: object, active_low: bool) -> bool:
        """将按钮原始值转换为“按下/未按下”。"""
        value = int(raw_value)
        return (value == 0) if active_low else (value != 0)

    def _button_callback(self, msg: OmniButtonEvent) -> None:
        """按钮回调。

        控制规则：
        - 灰键按下沿：触发夹取；
        - 白键按下沿：触发松开；
        - 同时按下或无有效边沿：不触发动作。
        """
        grey_pressed = self._to_pressed(msg.grey_button, self._grey_active_low)
        white_pressed = self._to_pressed(msg.white_button, self._white_active_low)

        grey_edge = grey_pressed and (not self._last_grey_pressed)
        white_edge = white_pressed and (not self._last_white_pressed)

        self._last_grey_pressed = grey_pressed
        self._last_white_pressed = white_pressed

        if grey_edge and not white_pressed:
            self._try_send_set_grip(True, source='grey_button')
            return

        if white_edge and not grey_pressed:
            self._try_send_set_grip(False, source='white_button')
            return

    def _try_send_set_grip(self, grip: bool, source: str) -> None:
        """尝试发送 set_grip 请求。

        保护机制：
        - 防抖：短时间内重复触发会被忽略；
        - 并发：前一个请求未完成时，不发送新请求。
        """
        now = time.monotonic()
        if now - self._last_trigger_time < self._debounce_sec:
            self.get_logger().debug('Debounce active, ignore trigger.')
            return

        if self._pending_future is not None and not self._pending_future.done():
            self.get_logger().warn('Previous set_grip request is still pending, ignore trigger.')
            return

        # 构造服务请求：True=夹取，False=松开。
        req = SetBool.Request()
        req.data = bool(grip)

        self._pending_future = self._set_grip_client.call_async(req)
        self._pending_future.add_done_callback(
            lambda future, action=('grip' if grip else 'release'), src=source: self._on_set_grip_done(future, action, src)
        )
        self._last_trigger_time = now

    def _on_set_grip_done(self, future, action: str, source: str) -> None:
        """服务回调：记录动作执行结果。"""
        try:
            result = future.result()
            if result is None:
                self.get_logger().error(f'[{source}] {action} failed: empty response')
                return
            if result.success:
                self.get_logger().info(f'[{source}] {action} success | {result.message}')
            else:
                self.get_logger().error(f'[{source}] {action} failed | {result.message}')
        except Exception as exc:
            self.get_logger().error(f'[{source}] {action} exception: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TG9801TeleoperationNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
