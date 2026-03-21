"""TG-9801 ROS2 接口测试客户端。

用于验证 tg_9801_node 暴露的服务和话题是否工作正常，适合做节点健康检查和回归测试。
"""

import argparse
import json
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .protocol import parse_number_string


@dataclass
class TestResult:
    """单项测试结果。"""

    name: str
    passed: bool
    detail: str
    required: bool = True


class TG9801TestClient(Node):
    """用于验证 tg_9801_node 对外 ROS2 接口的测试客户端。"""

    def __init__(
        self,
        service_name: str,
        raw_topic: str,
        rx_topic: str,
        command_topic: str,
        response_topic: str,
        timeout: float,
        action_interval: float,
        raw_payload: str,
        require_rx: bool,
        run_all_actions: bool,
        include_destructive: bool,
        test_protocol: str,
        non_critical_actions: List[str],
    ) -> None:
        super().__init__('tg_9801_test_client')

        self._service_name = service_name
        self._raw_topic = raw_topic
        self._rx_topic = rx_topic
        self._command_topic = command_topic
        self._response_topic = response_topic
        self._timeout = timeout
        self._action_interval = action_interval
        self._raw_payload = raw_payload
        self._require_rx = require_rx
        self._run_all_actions = run_all_actions
        self._include_destructive = include_destructive
        self._test_protocol = test_protocol
        self._non_critical_actions = {item.strip() for item in non_critical_actions if item.strip()}

        self._set_grip_client = self.create_client(SetBool, self._service_name)
        self._raw_pub = self.create_publisher(String, self._raw_topic, 10)
        self._command_pub = self.create_publisher(String, self._command_topic, 10)
        self._rx_sub = self.create_subscription(String, self._rx_topic, self._on_rx, 10)
        self._response_sub = self.create_subscription(String, self._response_topic, self._on_response, 10)

        self._last_rx: Optional[str] = None
        self._rx_received_since_send = False
        self._last_response_raw: Optional[str] = None
        self._last_response: Optional[Dict[str, Any]] = None
        self._response_received_since_send = False

    def _on_rx(self, msg: String) -> None:
        self._last_rx = msg.data
        self._rx_received_since_send = True

    def _on_response(self, msg: String) -> None:
        self._last_response_raw = msg.data
        self._response_received_since_send = True
        try:
            parsed = json.loads(msg.data)
            self._last_response = parsed if isinstance(parsed, dict) else {'ok': False, 'error': 'response_json 非 object'}
        except Exception as exc:
            self._last_response = {'ok': False, 'error': f'response_json 非法 JSON: {exc}'}

    def _sleep_with_spin(self, seconds: float) -> None:
        end_time = time.time() + max(0.0, seconds)
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_condition(self, predicate, timeout: float) -> bool:
        end_time = time.time() + max(0.0, timeout)
        while rclpy.ok() and time.time() < end_time:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(predicate())

    def _wait_for_service(self) -> TestResult:
        ok = self._set_grip_client.wait_for_service(timeout_sec=self._timeout)
        if ok:
            return TestResult('服务可用性(set_grip)', True, f'服务 {self._service_name} 已发现')
        return TestResult(
            '服务可用性(set_grip)',
            False,
            f'等待超时({self._timeout:.1f}s)，未发现服务 {self._service_name}',
        )

    def _wait_for_topic_graph(self) -> List[TestResult]:
        results: List[TestResult] = []

        has_raw_sub = self._wait_for_condition(
            lambda: len(self.get_subscriptions_info_by_topic(self._raw_topic)) > 0,
            self._timeout,
        )
        if has_raw_sub:
            results.append(TestResult('订阅接口(raw_tx)', True, f'{self._raw_topic} 存在订阅者'))
        else:
            results.append(
                TestResult(
                    '订阅接口(raw_tx)',
                    False,
                    f'在 {self._timeout:.1f}s 内未发现 {self._raw_topic} 的订阅者',
                )
            )

        has_rx_pub = self._wait_for_condition(
            lambda: len(self.get_publishers_info_by_topic(self._rx_topic)) > 0,
            self._timeout,
        )
        if has_rx_pub:
            results.append(TestResult('发布接口(rx_hex)', True, f'{self._rx_topic} 存在发布者'))
        else:
            results.append(
                TestResult(
                    '发布接口(rx_hex)',
                    False,
                    f'在 {self._timeout:.1f}s 内未发现 {self._rx_topic} 的发布者',
                )
            )

        has_command_sub = self._wait_for_condition(
            lambda: len(self.get_subscriptions_info_by_topic(self._command_topic)) > 0,
            self._timeout,
        )
        if has_command_sub:
            results.append(TestResult('订阅接口(command_json)', True, f'{self._command_topic} 存在订阅者'))
        else:
            results.append(
                TestResult(
                    '订阅接口(command_json)',
                    False,
                    f'在 {self._timeout:.1f}s 内未发现 {self._command_topic} 的订阅者',
                )
            )

        has_response_pub = self._wait_for_condition(
            lambda: len(self.get_publishers_info_by_topic(self._response_topic)) > 0,
            self._timeout,
        )
        if has_response_pub:
            results.append(TestResult('发布接口(response_json)', True, f'{self._response_topic} 存在发布者'))
        else:
            results.append(
                TestResult(
                    '发布接口(response_json)',
                    False,
                    f'在 {self._timeout:.1f}s 内未发现 {self._response_topic} 的发布者',
                )
            )
        return results

    def _call_set_grip(self, grip: bool) -> TestResult:
        request = SetBool.Request()
        request.data = grip

        future = self._set_grip_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._timeout)
        action = '夹取(true)' if grip else '松开(false)'

        if not future.done():
            return TestResult(
                f'服务调用 {action}',
                False,
                f'调用超时({self._timeout:.1f}s)',
            )

        exc = future.exception()
        if exc is not None:
            return TestResult(f'服务调用 {action}', False, f'调用异常: {exc}')

        response = future.result()
        if response is None:
            return TestResult(f'服务调用 {action}', False, '返回为空')

        if response.success:
            return TestResult(f'服务调用 {action}', True, response.message)
        return TestResult(f'服务调用 {action}', False, response.message)

    def _test_raw_tx_and_rx(self) -> TestResult:
        data = parse_number_string(self._raw_payload)
        if not data:
            return TestResult('原始透传(raw_tx->rx_hex)', False, 'raw payload 解析结果为空')

        self._rx_received_since_send = False
        self._raw_pub.publish(String(data=self._raw_payload))
        got_rx = self._wait_for_condition(lambda: self._rx_received_since_send, self._timeout)

        if got_rx:
            return TestResult(
                '原始透传(raw_tx->rx_hex)',
                True,
                f'已收到回包: {self._last_rx}',
                required=self._require_rx,
            )

        if self._require_rx:
            return TestResult(
                '原始透传(raw_tx->rx_hex)',
                False,
                f'发送后 {self._timeout:.1f}s 未收到回包',
                required=True,
            )

        return TestResult(
            '原始透传(raw_tx->rx_hex)',
            True,
            f'发送后 {self._timeout:.1f}s 未收到回包（按配置不计失败）',
            required=False,
        )

    def _send_command_and_wait(self, command: Dict[str, Any]) -> TestResult:
        action = str(command.get('action', '')).strip() or 'unknown'
        self._response_received_since_send = False
        self._last_response = None
        self._last_response_raw = None
        self._command_pub.publish(String(data=json.dumps(command, ensure_ascii=False)))

        got_response = self._wait_for_condition(lambda: self._response_received_since_send, self._timeout)
        if not got_response:
            return TestResult(f'command_json:{action}', False, f'发送后 {self._timeout:.1f}s 未收到 response_json')

        if not isinstance(self._last_response, dict):
            return TestResult(
                f'command_json:{action}',
                False,
                f'返回不可解析: {self._last_response_raw}',
            )

        ok = bool(self._last_response.get('ok', False))
        if ok:
            return TestResult(
                f'command_json:{action}',
                True,
                f'执行成功: {json.dumps(self._last_response, ensure_ascii=False)}',
            )

        return TestResult(
            f'command_json:{action}',
            False,
            f'执行失败: {json.dumps(self._last_response, ensure_ascii=False)}',
        )

    def _build_action_matrix(self) -> List[Dict[str, Any]]:
        common_defaults: Dict[str, Any] = {
            'device_addr': 1,
            'speed': 1000,
            'target_force': 50,
            'position': 800,
            'register_addr': 0x000A,
            'register_value': 0x0001,
            'register_values': [0x0001, 0x0002],
            'read_start_addr': 0x0000,
            'read_count': 1,
            'reserved': 0,
            'protocol_mode': 'modbus',
            'position_mode': 'position',
            'modbus_start_addr': 0x000A,
            'rs485_release_cmd': 1,
            'baudrate_value': 1000000,
            'new_device_addr': 1,
        }

        protocol = self._test_protocol
        actions: List[Dict[str, Any]] = [
            {'action': 'grip', 'protocol': protocol},
            {'action': 'release', 'protocol': protocol},
            {'action': 'get_id', 'protocol': protocol},
            {'action': 'get_serial', 'protocol': protocol},
            {'action': 'get_status', 'protocol': protocol},
            {'action': 'get_force', 'protocol': protocol},
            {'action': 'set_force', 'protocol': protocol},
            {'action': 'set_position_mode', 'protocol': protocol},
            {'action': 'position', 'protocol': protocol},
            {'action': 'calibrate', 'protocol': protocol},
            {'action': 'hardware_zero', 'protocol': protocol},
            {'action': 'record_zero', 'protocol': protocol},
            {'action': 'set_id', 'protocol': protocol, 'new_device_addr': 1},
        ]

        if protocol == 'rs485_custom':
            actions.extend([
                {'action': 'get_finger', 'protocol': 'rs485_custom'},
                {'action': 'set_baudrate', 'protocol': 'rs485_custom', 'baudrate_value': 1000000},
            ])
        else:
            actions.extend([
                {'action': 'get_follow_status', 'protocol': 'modbus_rtu'},
                {'action': 'read_holding', 'protocol': 'modbus_rtu', 'read_start_addr': 0x0000, 'read_count': 1},
                {'action': 'read_input', 'protocol': 'modbus_rtu', 'read_start_addr': 0x0000, 'read_count': 1},
                {'action': 'write_single', 'protocol': 'modbus_rtu', 'register_addr': 0x000A, 'register_value': 0x0001},
                {'action': 'write_multi', 'protocol': 'modbus_rtu', 'register_addr': 0x000A, 'register_values': [0x0001, 0x0002]},
                {'action': 'set_speed', 'protocol': 'modbus_rtu', 'speed': 1000},
                {'action': 'set_command', 'protocol': 'modbus_rtu', 'register_value': 0x0001},
            ])

        actions.append(
            {
                'action': 'set_protocol',
                'protocol': protocol,
                'protocol_mode': 'custom' if protocol == 'rs485_custom' else 'modbus',
            }
        )

        return [{**common_defaults, **command} for command in actions]

    def _is_destructive_action(self, action: str) -> bool:
        return action in {
            'set_force',
            'set_id',
            'set_protocol',
            'calibrate',
            'set_position_mode',
            'position',
            'hardware_zero',
            'record_zero',
            'set_baudrate',
            'write_single',
            'write_multi',
            'set_speed',
            'set_command',
            'grip',
            'release',
        }

    def _test_all_actions(self) -> List[TestResult]:
        if not self._run_all_actions:
            return [
                TestResult(
                    'command_json 全量动作测试',
                    True,
                    '未启用 --run-all-actions，已跳过',
                    required=False,
                )
            ]

        results: List[TestResult] = []
        for command in self._build_action_matrix():
            action = str(command.get('action', ''))
            if self._is_destructive_action(action) and not self._include_destructive:
                results.append(
                    TestResult(
                        f'command_json:{action}',
                        True,
                        '危险写操作已跳过（使用 --include-destructive 可执行）',
                        required=False,
                    )
                )
                continue

            result = self._send_command_and_wait(command)
            if action in self._non_critical_actions:
                result.required = False
                if not result.passed:
                    result.detail = f'{result.detail} | 已配置为非关键动作'
            results.append(result)
            self._sleep_with_spin(0.05)
        return results

    def _test_invalid_command(self) -> TestResult:
        self._response_received_since_send = False
        self._last_response = None
        self._last_response_raw = None
        self._command_pub.publish(String(data='not a json'))

        got_response = self._wait_for_condition(lambda: self._response_received_since_send, self._timeout)
        if not got_response:
            return TestResult('command_json 非法JSON处理', False, f'发送后 {self._timeout:.1f}s 未收到 response_json')

        if isinstance(self._last_response, dict) and not bool(self._last_response.get('ok', True)):
            return TestResult('command_json 非法JSON处理', True, json.dumps(self._last_response, ensure_ascii=False))

        return TestResult('command_json 非法JSON处理', False, f'返回未体现错误处理: {self._last_response_raw}')

    def run_all(self) -> List[TestResult]:
        results: List[TestResult] = []

        results.append(self._wait_for_service())
        if not results[-1].passed:
            return results

        results.extend(self._wait_for_topic_graph())

        grip_res = self._call_set_grip(True)
        results.append(grip_res)
        self._sleep_with_spin(self._action_interval)

        release_res = self._call_set_grip(False)
        results.append(release_res)
        self._sleep_with_spin(self._action_interval)

        results.append(self._test_raw_tx_and_rx())
        results.append(self._test_invalid_command())
        results.extend(self._test_all_actions())
        return results


def _print_summary(results: List[TestResult]) -> bool:
    print('\n========== TG-9801 ROS2 接口测试结果 ==========' )
    required_pass = True
    passed_count = 0

    for item in results:
        if item.passed:
            tag = 'PASS'
            passed_count += 1
        else:
            tag = 'FAIL' if item.required else 'WARN'
        if item.required and not item.passed:
            required_pass = False
        print(f'[{tag}] {item.name} | {item.detail}')

    print('-----------------------------------------------')
    print(f'通过项: {passed_count}/{len(results)}')
    print(f'结论: {"全部关键项通过" if required_pass else "存在关键失败项"}')
    print('===============================================\n')
    return required_pass


def build_arg_parser() -> argparse.ArgumentParser:
    """构建命令行参数。"""
    parser = argparse.ArgumentParser(description='TG-9801 ROS2 服务节点客户端测试工具')
    parser.add_argument('--service-name', default='set_grip', help='待测服务名，默认 set_grip')
    parser.add_argument('--raw-topic', default='raw_tx', help='原始发送 topic，默认 raw_tx')
    parser.add_argument('--rx-topic', default='rx_hex', help='回包 topic，默认 rx_hex')
    parser.add_argument('--command-topic', default='command_json', help='命令输入 topic，默认 command_json')
    parser.add_argument('--response-topic', default='response_json', help='命令响应 topic，默认 response_json')
    parser.add_argument('--timeout', type=float, default=2.0, help='等待超时秒，默认2.0')
    parser.add_argument('--action-interval', type=float, default=0.4, help='动作间隔秒，默认0.4')
    parser.add_argument(
        '--raw-payload',
        default='0xA3 0xB4 0x02 0x01 0x01 0x04',
        help='raw_tx 测试帧，默认使用自定义协议夹取帧示例',
    )
    parser.add_argument(
        '--require-rx',
        action='store_true',
        help='将 raw_tx 后必须收到 rx_hex 回包作为失败判定条件',
    )
    parser.add_argument(
        '--run-all-actions',
        action='store_true',
        help='启用 command_json 全动作测试（覆盖 tg_9801_node 全部 action）',
    )
    parser.add_argument(
        '--include-destructive',
        action='store_true',
        help='执行可能改写设备状态/参数的动作（默认仅测试安全项）',
    )
    parser.add_argument(
        '--test-protocol',
        choices=['rs485_custom', 'modbus_rtu'],
        default='modbus_rtu',
        help='全动作测试使用的协议，默认 modbus_rtu',
    )
    parser.add_argument(
        '--non-critical-actions',
        default='',
        help='逗号分隔的 action 白名单，失败时按非关键项处理（如 read_input,get_finger）',
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    non_critical_actions = [item.strip() for item in str(args.non_critical_actions).split(',') if item.strip()]
    rclpy.init()

    node = TG9801TestClient(
        service_name=args.service_name,
        raw_topic=args.raw_topic,
        rx_topic=args.rx_topic,
        command_topic=args.command_topic,
        response_topic=args.response_topic,
        timeout=args.timeout,
        action_interval=args.action_interval,
        raw_payload=args.raw_payload,
        require_rx=bool(args.require_rx),
        run_all_actions=bool(args.run_all_actions),
        include_destructive=bool(args.include_destructive),
        test_protocol=str(args.test_protocol),
        non_critical_actions=non_critical_actions,
    )
    try:
        results = node.run_all()
        all_required_passed = _print_summary(results)
        return 0 if all_required_passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())