"""
按指定频率触发 UR 工具数字输出，并记录每次触发时的末端位姿。

节点订阅 `/asm_ee_site/pose`，在每次 tool DO 上升沿记录当前时间和最新
位姿。退出时将 DO 恢复为低电平，并把记录保存到
`ultra_scanning_system/data` 下的 CSV 文件。
"""

from pathlib import Path
from typing import Iterable, Sequence

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rtde_io import RTDEIOInterface as RTDEIO

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


class FrequencyToolDOTriggerNode(Node):
    """按固定频率输出 tool DO 脉冲并记录最新末端位姿."""

    def __init__(self) -> None:
        super().__init__('frequency_tool_do_trigger_node')

        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('pose_topic', '/asm_ee_site/pose')
        self.declare_parameter('tool_do_index', 0)
        self.declare_parameter('trigger_frequency_hz', 10.0)
        self.declare_parameter('pulse_width_sec', 0.001)
        self.declare_parameter(
            'records_file',
            'frequency_tool_do_triggers.csv',
        )

        self.robot_ip = str(self.get_parameter('robot_ip').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.tool_do_index = int(self.get_parameter('tool_do_index').value)
        self.trigger_frequency_hz = float(
            self.get_parameter('trigger_frequency_hz').value
        )
        self.pulse_width_sec = float(
            self.get_parameter('pulse_width_sec').value
        )
        self.records_file = resolve_records_file(
            str(self.get_parameter('records_file').value)
        )
        self.period_sec = validate_trigger_timing(
            self.trigger_frequency_hz,
            self.pulse_width_sec,
        )

        self.rtde_io = RTDEIO(self.robot_ip)
        self.current_pose: PoseTuple | None = None
        self.records: list[tuple[float, PoseTuple]] = []
        self.pulse_active = False
        self.pulse_timer = None
        self.records_saved = False

        self.set_tool_do(False)
        self.create_subscription(Pose, self.pose_topic, self._on_pose, 10)
        self.create_timer(self.period_sec, self._trigger)

        self.get_logger().info(
            'Frequency tool DO trigger started: '
            f'frequency={self.trigger_frequency_hz:.3f} Hz, '
            f'pulse_width={self.pulse_width_sec:.6f} s, '
            f'pose_topic={self.pose_topic}, records_file={self.records_file}'
        )

    def set_tool_do(self, value: bool) -> None:
        """设置工具数字输出."""
        self.rtde_io.setToolDigitalOut(self.tool_do_index, bool(value))

    def _on_pose(self, msg: Pose) -> None:
        self.current_pose = pose_msg_to_tuple(msg)

    def _trigger(self) -> None:
        """产生一次上升沿，并记录该时刻的最新末端位姿."""
        if self.current_pose is None or self.pulse_active:
            return

        timestamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self.records.append((timestamp_sec, self.current_pose))
        self.records_saved = False
        self.pulse_active = True
        self.set_tool_do(True)
        self.pulse_timer = self.create_timer(
            self.pulse_width_sec,
            self._finish_pulse,
        )

    def _finish_pulse(self) -> None:
        self.set_tool_do(False)
        self.pulse_active = False
        if self.pulse_timer is not None:
            self.destroy_timer(self.pulse_timer)
            self.pulse_timer = None

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
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrequencyToolDOTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
