"""TG-9801 状态广播节点。

定时通过 tg_9801_node 的 command_json/response_json 接口拉取夹爪状态，
并将关键状态量拆分发布到独立 topic，供上层应用直接订阅。
"""

import json
import time
from typing import Any, Dict, List, Optional

from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tg_9801_interfaces.msg import GripperFingerState, GripperMotionState, GripperStaticInfo


class TGStateBroadcaster(Node):
    """夹爪状态广播节点。"""

    def __init__(self) -> None:
        super().__init__('tg_state_broadcaster')

        self.declare_parameter('command_topic', 'command_json')
        self.declare_parameter('response_topic', 'response_json')
        self.declare_parameter('state_prefix', 'tg_state')
        self.declare_parameter('poll_period', 1.0)
        self.declare_parameter('request_timeout', 0.8)
        self.declare_parameter('protocol', 'modbus_rtu')
        self.declare_parameter('device_addr', 1)
        self.declare_parameter('modbus_speed_register', 0x000A)
        self.declare_parameter('read_speed_register', True)
        self.declare_parameter('read_follow_status', True)
        self.declare_parameter('read_finger_data', False)
        self.declare_parameter('poll_identity_every_cycle', False)
        self.declare_parameter('open_position_threshold', 50)
        self.declare_parameter('closed_position_threshold', 900)
        self.declare_parameter('finger_publish_hz', 50.0)
        self.declare_parameter('motion_publish_hz', 10.0)
        self.declare_parameter('static_publish_hz', 1.0)

        self._command_topic = str(self.get_parameter('command_topic').value)
        self._response_topic = str(self.get_parameter('response_topic').value)
        self._state_prefix = str(self.get_parameter('state_prefix').value).strip('/')
        self._poll_period = max(0.2, float(self.get_parameter('poll_period').value))
        self._request_timeout = max(0.2, float(self.get_parameter('request_timeout').value))
        self._protocol = str(self.get_parameter('protocol').value).strip().lower()
        self._device_addr = int(self.get_parameter('device_addr').value)
        self._modbus_speed_register = int(self.get_parameter('modbus_speed_register').value)
        self._read_speed_register = bool(self.get_parameter('read_speed_register').value)
        self._read_follow_status = bool(self.get_parameter('read_follow_status').value)
        self._read_finger_data = bool(self.get_parameter('read_finger_data').value)
        self._poll_identity_every_cycle = bool(self.get_parameter('poll_identity_every_cycle').value)
        self._open_position_threshold = int(self.get_parameter('open_position_threshold').value)
        self._closed_position_threshold = int(self.get_parameter('closed_position_threshold').value)
        self._finger_publish_hz = max(0.1, float(self.get_parameter('finger_publish_hz').value))
        self._motion_publish_hz = max(0.1, float(self.get_parameter('motion_publish_hz').value))
        self._static_publish_hz = max(0.1, float(self.get_parameter('static_publish_hz').value))

        self._command_pub = self.create_publisher(String, self._command_topic, 10)
        self.create_subscription(String, self._response_topic, self._on_response, 10)

        prefix = self._state_prefix
        self._pub_last_error = self.create_publisher(DiagnosticStatus, f'{prefix}/last_error', 10)
        self._pub_static_info = self.create_publisher(GripperStaticInfo, f'{prefix}/static_info', 10)
        self._pub_motion_state = self.create_publisher(GripperMotionState, f'{prefix}/motion_state', 10)
        self._pub_finger_state = self.create_publisher(GripperFingerState, f'{prefix}/finger_state', 10)

        self._snapshot: Dict[str, Any] = {
            'protocol': self._protocol,
            'device_addr': self._device_addr,
            'device_id': None,
            'serial': None,
            'status': None,
            'hardness': None,
            'position': None,
            'current': None,
            'target_force': None,
            'speed': None,
            'follow_status': None,
            'finger': None,
            'opened': None,
            'closed': None,
            'updated_at': None,
        }
        self._last_error = ''

        self._queue: List[Dict[str, Any]] = []
        self._pending_action: Optional[str] = None
        self._pending_deadline = 0.0
        self._cycle_active = False
        self._next_cycle_at = time.monotonic()
        self._identity_initialized = False

        self.create_timer(0.02, self._tick)
        self.create_timer(1.0 / self._finger_publish_hz, self._publish_finger_state)
        self.create_timer(1.0 / self._motion_publish_hz, self._publish_motion_state)
        self.create_timer(1.0 / self._static_publish_hz, self._publish_static_info)

        self.get_logger().info(
            f'tg_state_broadcaster started | protocol={self._protocol} device_addr={self._device_addr} '
            f'poll_period={self._poll_period:.2f}s command_topic={self._command_topic} response_topic={self._response_topic} '
            f'finger_hz={self._finger_publish_hz:.1f} motion_hz={self._motion_publish_hz:.1f} static_hz={self._static_publish_hz:.1f}'
        )

    def _build_poll_actions(self) -> List[Dict[str, Any]]:
        actions: List[Dict[str, Any]] = [
            {'action': 'get_status', 'protocol': self._protocol, 'device_addr': self._device_addr},
            {'action': 'get_force', 'protocol': self._protocol, 'device_addr': self._device_addr},
        ]

        need_identity = self._poll_identity_every_cycle or not self._identity_initialized
        if need_identity:
            actions.extend([
                {'action': 'get_id', 'protocol': self._protocol, 'device_addr': self._device_addr},
                {'action': 'get_serial', 'protocol': self._protocol, 'device_addr': self._device_addr},
            ])

        if self._protocol == 'modbus_rtu':
            if self._read_speed_register:
                actions.append({
                    'action': 'read_holding',
                    'protocol': 'modbus_rtu',
                    'device_addr': self._device_addr,
                    'read_start_addr': self._modbus_speed_register,
                    'read_count': 1,
                })
            if self._read_follow_status:
                actions.append({'action': 'get_follow_status', 'protocol': 'modbus_rtu', 'device_addr': self._device_addr})

        if self._protocol == 'rs485_custom' and self._read_finger_data:
            actions.append({'action': 'get_finger', 'protocol': 'rs485_custom', 'device_addr': self._device_addr})

        return actions

    def _tick(self) -> None:
        now = time.monotonic()

        if self._pending_action is not None and now > self._pending_deadline:
            self._set_error(f'action={self._pending_action} 等待 response_json 超时({self._request_timeout:.2f}s)')
            self._pending_action = None

        if self._pending_action is not None:
            return

        if self._queue:
            self._send_next()
            return

        if self._cycle_active:
            self._cycle_active = False
            self._next_cycle_at = now + self._poll_period
            return

        if now >= self._next_cycle_at:
            self._queue = self._build_poll_actions()
            self._cycle_active = True

    def _send_next(self) -> None:
        command = self._queue.pop(0)
        action = str(command.get('action', '')).strip().lower()
        if not action:
            return

        self._pending_action = action
        self._pending_deadline = time.monotonic() + self._request_timeout
        self._command_pub.publish(String(data=json.dumps(command, ensure_ascii=False)))

    def _on_response(self, msg: String) -> None:
        if self._pending_action is None:
            return

        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        if not isinstance(payload, dict):
            return

        action = str(payload.get('action', '')).strip().lower()
        if action != self._pending_action:
            return

        if not bool(payload.get('ok', False)):
            error = str(payload.get('error', f'action={action} 执行失败'))
            self._set_error(f'action={action} | {error}')
            self._pending_action = None
            return

        result = payload.get('result')
        try:
            self._merge_result(action, result)
        except Exception as exc:
            self._set_error(f'action={action} 结果解析失败: {exc}')

        self._pending_action = None

    def _set_error(self, text: str) -> None:
        self._last_error = text
        msg = DiagnosticStatus()
        msg.level = DiagnosticStatus.WARN
        msg.name = 'tg_state_broadcaster'
        msg.message = text
        msg.hardware_id = f'tg9801:{self._device_addr}'
        self._pub_last_error.publish(msg)
        self.get_logger().warn(text)

    @staticmethod
    def _to_int(value: Any) -> Optional[int]:
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, int):
            return value
        if isinstance(value, float):
            return int(value)
        return None

    def _merge_result(self, action: str, result: Any) -> None:
        if action == 'get_status' and isinstance(result, dict):
            for key in ('x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'status', 'hardness', 'position', 'current'):
                value = self._to_int(result.get(key))
                if value is not None:
                    self._snapshot[key] = value
        elif action == 'get_force':
            value = self._to_int(result)
            if value is not None:
                self._snapshot['target_force'] = value
        elif action == 'get_id':
            value = self._to_int(result)
            if value is not None:
                self._snapshot['device_id'] = value
                self._identity_initialized = self._snapshot.get('serial') is not None
        elif action == 'get_serial' and isinstance(result, str):
            self._snapshot['serial'] = result
            self._identity_initialized = self._snapshot.get('device_id') is not None
        elif action == 'read_holding' and isinstance(result, list) and result:
            value = self._to_int(result[0])
            if value is not None:
                self._snapshot['speed'] = value
        elif action == 'get_follow_status':
            value = self._to_int(result)
            if value is not None:
                self._snapshot['follow_status'] = value
        elif action == 'get_finger' and isinstance(result, dict):
            self._snapshot['finger'] = result

        position = self._to_int(self._snapshot.get('position'))
        if position is not None:
            self._snapshot['opened'] = position <= self._open_position_threshold
            self._snapshot['closed'] = position >= self._closed_position_threshold

        self._snapshot['updated_at'] = time.time()

    def _publish_static_info(self) -> None:
        if self._last_error:
            err = DiagnosticStatus()
            err.level = DiagnosticStatus.WARN
            err.name = 'tg_state_broadcaster'
            err.message = self._last_error
            err.hardware_id = f'tg9801:{self._device_addr}'
            self._pub_last_error.publish(err)

        static_info = GripperStaticInfo()
        static_info.protocol = str(self._snapshot.get('protocol') or '')
        static_info.device_addr = int(self._snapshot.get('device_addr') or 0)
        device_id = self._snapshot.get('device_id')
        serial = self._snapshot.get('serial')
        static_info.has_device_id = isinstance(device_id, int)
        static_info.has_serial = isinstance(serial, str)
        static_info.device_id = int(device_id) if isinstance(device_id, int) else 0
        static_info.serial = serial if isinstance(serial, str) else ''
        self._pub_static_info.publish(static_info)

    def _publish_motion_state(self) -> None:
        motion_state = GripperMotionState()
        motion_state.stamp = self.get_clock().now().to_msg()
        motion_state.status_code = int(self._snapshot.get('status') or 0)
        motion_state.hardness = int(self._snapshot.get('hardness') or 0)
        motion_state.current = int(self._snapshot.get('current') or 0)
        motion_state.target_force = int(self._snapshot.get('target_force') or 0)
        motion_state.speed = int(self._snapshot.get('speed') or 0)
        motion_state.follow_status = int(self._snapshot.get('follow_status') or 0)
        motion_state.opened = bool(self._snapshot.get('opened')) if self._snapshot.get('opened') is not None else False
        motion_state.closed = bool(self._snapshot.get('closed')) if self._snapshot.get('closed') is not None else False
        motion_state.has_status = isinstance(self._snapshot.get('status'), int)
        motion_state.has_force = isinstance(self._snapshot.get('target_force'), int)
        motion_state.has_speed = isinstance(self._snapshot.get('speed'), int)
        motion_state.has_follow_status = isinstance(self._snapshot.get('follow_status'), int)
        self._pub_motion_state.publish(motion_state)

    def _publish_finger_state(self) -> None:
        finger_state = GripperFingerState()
        finger_state.stamp = self.get_clock().now().to_msg()
        finger_state.position = int(self._snapshot.get('position') or 0)
        finger_state.directional_forces = [
            int(self._snapshot.get('x1') or 0),
            int(self._snapshot.get('x2') or 0),
            int(self._snapshot.get('y1') or 0),
            int(self._snapshot.get('y2') or 0),
            int(self._snapshot.get('z1') or 0),
            int(self._snapshot.get('z2') or 0),
        ]
        finger_state.valid = isinstance(self._snapshot.get('position'), int) and all(
            isinstance(self._snapshot.get(key), int) for key in ('x1', 'x2', 'y1', 'y2', 'z1', 'z2')
        )
        self._pub_finger_state.publish(finger_state)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TGStateBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
