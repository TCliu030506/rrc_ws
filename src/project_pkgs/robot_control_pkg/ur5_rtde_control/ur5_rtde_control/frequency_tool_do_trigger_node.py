"""
在接触扫查阶段按指定频率触发 UR 工具数字输出，并记录每次触发时的末端位姿。

节点订阅 `/contact_scan/state` 和 `/asm_ee_site/pose`。当状态进入
`contact_scan` 后，节点按固定频率产生 tool DO 上升沿；每次上升沿记录
当前时间和最新位姿。达到指定触发次数后，节点保存 CSV 并退出。
"""

from pathlib import Path
from datetime import datetime
import socket
from typing import Iterable, Sequence

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import String

from ur5_rtde_control.tool_do_pose_trigger_node import (
    PoseTuple,
    pose_msg_to_tuple,
    write_pose_records_csv,
)


def find_ultra_scanning_data_dir(
    search_starts: Iterable[Path] | None = None,
) -> Path:
    """从源码或 install 路径向上查找工作区中的超声数据目录."""
    starts = search_starts or (Path(__file__).resolve(), Path.cwd().resolve())
    relative_data_dir = Path(
        'src/project_pkgs/robot_control_pkg/ultra_scanning_system/data'
    )
    for start in starts:
        current = Path(start)
        if not current.is_dir():
            current = current.parent
        for parent in (current, *current.parents):
            candidate = parent / relative_data_dir
            if candidate.is_dir():
                return candidate
    raise FileNotFoundError(
        'Cannot find src/project_pkgs/robot_control_pkg/'
        'ultra_scanning_system/data'
    )


def resolve_records_file(records_file: str) -> Path:
    """将相对文件名解析到源码中的 ultra_scanning_system/data 目录."""
    path = Path(records_file).expanduser()
    if path.is_absolute():
        return path
    return find_ultra_scanning_data_dir() / path


def timestamp_records_file(path: Path, now: datetime | None = None) -> Path:
    """在记录文件名最前面加上当前日期时间前缀."""
    timestamp = (now or datetime.now()).strftime('%Y%m%d_%H%M')
    return path.with_name(f'{timestamp}_{path.name}')


def validate_trigger_timing(
    trigger_frequency_hz: float,
    pulse_width_sec: float,
) -> float:
    """校验触发参数并返回触发周期."""
    if trigger_frequency_hz <= 0.0:
        raise ValueError('trigger_frequency_hz must be > 0')
    if pulse_width_sec <= 0.0:
        raise ValueError('pulse_width_sec must be > 0')
    period_sec = 1.0 / trigger_frequency_hz
    if pulse_width_sec >= period_sec:
        raise ValueError('pulse_width_sec must be shorter than trigger period')
    return period_sec


def rtde_tool_do_value_for_physical_level(physical_high: bool) -> bool:
    """将期望的物理电平转换为当前硬件接线下的机器人 Tool DO 逻辑值."""
    return not physical_high


def build_set_tool_do_script(tool_do_index: int, robot_value: bool) -> str:
    """构造只设置 Tool DO 的 URScript secondary program."""
    value_text = 'True' if robot_value else 'False'
    return (
        'sec set_tool_do_trigger():\n'
        f'  set_tool_digital_out({int(tool_do_index)}, {value_text})\n'
        'end\n'
    )


class FrequencyToolDOTriggerNode(Node):
    """按固定频率输出 tool DO 脉冲并记录最新末端位姿."""

    def __init__(self) -> None:
        super().__init__('frequency_tool_do_trigger_node')

        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('pose_topic', '/asm_ee_site/pose')
        self.declare_parameter('state_topic', '/contact_scan/state')
        self.declare_parameter('trigger_state', 'contact_scan')
        self.declare_parameter('tool_do_index', 0)
        self.declare_parameter('trigger_frequency_hz', 1.1)
        self.declare_parameter('pulse_width_sec', 0.4)
        self.declare_parameter('trigger_count', 1000)
        self.declare_parameter('urscript_port', 30002)
        self.declare_parameter('urscript_timeout_sec', 0.2)
        self.declare_parameter(
            'records_file',
            'frequency_tool_do_triggers.csv',
        )

        self.robot_ip = str(self.get_parameter('robot_ip').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.trigger_state = str(self.get_parameter('trigger_state').value)
        self.tool_do_index = int(self.get_parameter('tool_do_index').value)
        self.trigger_frequency_hz = float(
            self.get_parameter('trigger_frequency_hz').value
        )
        self.pulse_width_sec = float(
            self.get_parameter('pulse_width_sec').value
        )
        self.trigger_count = int(self.get_parameter('trigger_count').value)
        if self.trigger_count <= 0:
            raise ValueError('trigger_count must be > 0')
        self.urscript_port = int(self.get_parameter('urscript_port').value)
        self.urscript_timeout_sec = float(
            self.get_parameter('urscript_timeout_sec').value
        )
        if self.urscript_timeout_sec <= 0.0:
            raise ValueError('urscript_timeout_sec must be > 0')
        self.records_file = timestamp_records_file(
            resolve_records_file(str(self.get_parameter('records_file').value))
        )
        self.period_sec = validate_trigger_timing(
            self.trigger_frequency_hz,
            self.pulse_width_sec,
        )

        self.current_pose: PoseTuple | None = None
        self.records: list[tuple[float, PoseTuple]] = []
        self.scan_active = False
        self.pulse_active = False
        self.pulse_timer = None
        self.records_saved = False
        self.done = False
        self.urscript_socket: socket.socket | None = None

        self.set_tool_do(False)
        self.create_subscription(Pose, self.pose_topic, self._on_pose, 10)
        self.create_subscription(String, self.state_topic, self._on_state, 10)
        self.create_timer(self.period_sec, self._trigger)

        self.get_logger().info(
            'Frequency tool DO trigger started: '
            f'frequency={self.trigger_frequency_hz:.3f} Hz, '
            f'pulse_width={self.pulse_width_sec:.6f} s, '
            f'pose_topic={self.pose_topic}, state_topic={self.state_topic}, '
            f'trigger_state={self.trigger_state}, '
            f'trigger_count={self.trigger_count}, '
            f'urscript_port={self.urscript_port}, '
            f'records_file={self.records_file}'
        )

    def set_tool_do(self, value: bool) -> None:
        """设置工具数字输出的物理电平，True 表示物理高电平."""
        robot_value = rtde_tool_do_value_for_physical_level(bool(value))
        script = build_set_tool_do_script(self.tool_do_index, robot_value)
        self._send_urscript(script)

    def _connect_urscript_socket(self) -> socket.socket:
        if self.urscript_socket is not None:
            return self.urscript_socket
        self.urscript_socket = socket.create_connection(
            (self.robot_ip, self.urscript_port),
            timeout=self.urscript_timeout_sec,
        )
        self.urscript_socket.settimeout(self.urscript_timeout_sec)
        return self.urscript_socket

    def _close_urscript_socket(self) -> None:
        if self.urscript_socket is None:
            return
        try:
            self.urscript_socket.close()
        finally:
            self.urscript_socket = None

    def _send_urscript(self, script: str) -> None:
        data = script.encode('utf-8')
        try:
            self._connect_urscript_socket().sendall(data)
        except OSError:
            self._close_urscript_socket()
            self._connect_urscript_socket().sendall(data)

    def _on_pose(self, msg: Pose) -> None:
        self.current_pose = pose_msg_to_tuple(msg)

    def _on_state(self, msg: String) -> None:
        was_scan_active = self.scan_active
        self.scan_active = msg.data == self.trigger_state
        if self.scan_active and not was_scan_active:
            self._trigger()
        elif (
            was_scan_active
            and not self.scan_active
            and len(self.records) < self.trigger_count
        ):
            self.save_records()

    def _trigger(self) -> None:
        """产生一次上升沿，并记录该时刻的最新末端位姿."""
        if (
            self.done
            or not self.scan_active
            or self.current_pose is None
            or self.pulse_active
            or len(self.records) >= self.trigger_count
        ):
            return

        timestamp_sec = self.get_clock().now().nanoseconds * 1e-9
        try:
            self.set_tool_do(True)
        except OSError as exc:
            self.get_logger().warn(f'Failed to set trigger DO high: {exc}')
            return

        self.records.append((timestamp_sec, self.current_pose))
        self.records_saved = False
        self.pulse_active = True
        self.pulse_timer = self.create_timer(
            self.pulse_width_sec,
            self._finish_pulse,
        )

    def _finish_pulse(self) -> None:
        try:
            self.set_tool_do(False)
        except OSError as exc:
            self.get_logger().warn(f'Failed to set trigger DO low: {exc}')
        self.pulse_active = False
        if self.pulse_timer is not None:
            self.destroy_timer(self.pulse_timer)
            self.pulse_timer = None
        if len(self.records) >= self.trigger_count:
            self._complete()

    def _complete(self) -> None:
        if self.done:
            return
        self.done = True
        self.save_records()
        self.get_logger().info(
            f'Completed {len(self.records)} triggers, node will exit.'
        )

    def save_records(self) -> None:
        if self.records_saved:
            return
        write_pose_records_csv(self.records_file, self.records)
        self.records_saved = True
        self.get_logger().info(
            f'Saved {len(self.records)} trigger records to {self.records_file}'
        )

    def destroy_node(self) -> bool:
        try:
            self.set_tool_do(False)
        except Exception as exc:
            self.get_logger().warn(f'Failed to reset tool DO: {exc}')
        self.save_records()
        self._close_urscript_socket()
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrequencyToolDOTriggerNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
