import csv
from pathlib import Path
from typing import Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rtde_io import RTDEIOInterface as RTDEIO
from std_srvs.srv import SetBool

from ur5_rtde_control.rtde_servol_frame_pose_math import rotate_vector


Vector3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
PoseTuple = Tuple[Vector3, Quat]


def pose_msg_to_tuple(msg: Pose) -> PoseTuple:
    """把 ROS Pose 转成便于计算和保存的元组."""
    return (
        (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        ),
        (
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        ),
    )


def projected_tool_x_distance(
    reference_pose: PoseTuple,
    current_pose: PoseTuple,
) -> float:
    """
    计算当前位置相对参考位姿，沿参考工具 X 轴方向的投影距离.

    返回值带符号；后续触发判断使用绝对值，因此正向/反向扫查都能触发。
    """
    reference_position, reference_orientation = reference_pose
    current_position, _ = current_pose
    tool_x_axis = rotate_vector(reference_orientation, (1.0, 0.0, 0.0))
    delta = (
        current_position[0] - reference_position[0],
        current_position[1] - reference_position[1],
        current_position[2] - reference_position[2],
    )
    return (
        delta[0] * tool_x_axis[0]
        + delta[1] * tool_x_axis[1]
        + delta[2] * tool_x_axis[2]
    )


def should_trigger_by_tool_x_distance(
    reference_pose: PoseTuple,
    current_pose: PoseTuple,
    *,
    trigger_distance: float,
) -> bool:
    """判断末端沿工具 X 轴位移是否达到触发间隔."""
    if trigger_distance <= 0.0:
        return False
    distance = projected_tool_x_distance(reference_pose, current_pose)
    return abs(distance) >= trigger_distance


def pose_record_to_row(timestamp_sec: float, pose: PoseTuple) -> list[float]:
    """把一次触发记录转成 CSV 行."""
    position, orientation = pose
    return [
        timestamp_sec,
        position[0],
        position[1],
        position[2],
        orientation[0],
        orientation[1],
        orientation[2],
        orientation[3],
    ]


class ToolDOPoseTriggerNode(Node):
    """
    基于末端工具位姿的 toolDO0 触发节点.

    启用后，每当 `/asm_ee_site/pose` 沿工具 X 轴移动指定距离，就输出一个
    toolDO0 高电平脉冲，并记录触发时刻和末端位姿。
    """

    def __init__(self) -> None:
        super().__init__('tool_do_pose_trigger_node')

        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('pose_topic', '/asm_ee_site/pose')
        self.declare_parameter('tool_do_index', 0)
        self.declare_parameter('trigger_distance', 0.0001)
        self.declare_parameter('pulse_width_sec', 0.001)
        self.declare_parameter('records_file', 'tool_do_pose_triggers.csv')
        self.declare_parameter('clear_records_on_enable', True)

        self.robot_ip = str(self.get_parameter('robot_ip').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.tool_do_index = int(self.get_parameter('tool_do_index').value)
        self.trigger_distance = float(
            self.get_parameter('trigger_distance').value
        )
        self.pulse_width_sec = float(
            self.get_parameter('pulse_width_sec').value
        )
        self.records_file = Path(
            str(self.get_parameter('records_file').value)
        ).expanduser()
        self.clear_records_on_enable = bool(
            self.get_parameter('clear_records_on_enable').value
        )

        if self.trigger_distance <= 0.0:
            raise ValueError('trigger_distance must be > 0')
        if self.pulse_width_sec <= 0.0:
            raise ValueError('pulse_width_sec must be > 0')

        self.rtde_io = RTDEIO(self.robot_ip)
        self.enabled = False
        self.current_pose: PoseTuple | None = None
        self.last_trigger_pose: PoseTuple | None = None
        self.records: list[tuple[float, PoseTuple]] = []
        self.pulse_active = False
        self.pulse_timer = None
        self.records_saved = False

        self.set_tool_do(False)
        self.create_subscription(Pose, self.pose_topic, self._on_pose, 10)
        self.create_service(SetBool, '~/set_enabled', self._on_set_enabled)

        self.get_logger().info(
            'Tool DO pose trigger started: '
            f'robot_ip={self.robot_ip}, pose_topic={self.pose_topic}, '
            f'tool_do{self.tool_do_index}, '
            f'trigger_distance={self.trigger_distance:.6f} m, '
            f'pulse_width={self.pulse_width_sec:.4f} s, '
            f'records_file={self.records_file}'
        )

    def set_tool_do(self, value: bool) -> None:
        """设置工具 DO 输出."""
        self.rtde_io.setToolDigitalOut(self.tool_do_index, bool(value))

    def _on_set_enabled(self, request, response):
        """SetBool 服务回调：true 启用触发，false 停止触发."""
        self.enabled = bool(request.data)
        self.set_tool_do(False)
        self.pulse_active = False
        self.last_trigger_pose = self.current_pose
        if self.enabled and self.clear_records_on_enable:
            self.records.clear()
            self.records_saved = False
        response.success = True
        response.message = (
            'tool DO pose trigger enabled'
            if self.enabled
            else 'tool DO pose trigger disabled'
        )
        self.get_logger().info(response.message)
        return response

    def _on_pose(self, msg: Pose) -> None:
        """处理末端位姿，并在启用时检查工具 X 轴位移触发."""
        pose = pose_msg_to_tuple(msg)
        self.current_pose = pose
        if not self.enabled:
            return
        if self.last_trigger_pose is None:
            self.last_trigger_pose = pose
            return
        if self.pulse_active:
            return
        if not should_trigger_by_tool_x_distance(
            self.last_trigger_pose,
            pose,
            trigger_distance=self.trigger_distance,
        ):
            return

        self._trigger_pulse(pose)

    def _trigger_pulse(self, pose: PoseTuple) -> None:
        """输出一个高电平脉冲，并记录触发时的时间和位姿."""
        timestamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self.records.append((timestamp_sec, pose))
        self.records_saved = False
        self.last_trigger_pose = pose
        self.pulse_active = True
        self.set_tool_do(True)
        self.pulse_timer = self.create_timer(
            self.pulse_width_sec,
            self._finish_pulse,
        )

    def _finish_pulse(self) -> None:
        """结束当前 toolDO0 脉冲."""
        self.set_tool_do(False)
        self.pulse_active = False
        if self.pulse_timer is not None:
            self.destroy_timer(self.pulse_timer)
            self.pulse_timer = None

    def save_records(self) -> None:
        """把触发时间戳和对应末端位姿保存为 CSV 文件."""
        if self.records_saved:
            return
        self.records_file.parent.mkdir(parents=True, exist_ok=True)
        with self.records_file.open('w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([
                'timestamp_sec',
                'position_x',
                'position_y',
                'position_z',
                'orientation_x',
                'orientation_y',
                'orientation_z',
                'orientation_w',
            ])
            for timestamp_sec, pose in self.records:
                writer.writerow(pose_record_to_row(timestamp_sec, pose))
        self.records_saved = True
        self.get_logger().info(
            f'Saved {len(self.records)} tool DO trigger records '
            f'to {self.records_file}'
        )

    def destroy_node(self) -> bool:
        """节点退出前关闭 DO 并保存触发记录."""
        try:
            self.set_tool_do(False)
        except Exception as exc:
            self.get_logger().warn(f'Failed to reset tool DO: {exc}')
        self.save_records()
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ToolDOPoseTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
