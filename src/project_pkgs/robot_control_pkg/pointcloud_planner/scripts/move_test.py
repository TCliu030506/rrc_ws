#!/usr/bin/env python3
import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from ur5_msg.msg import RobotState
from ur5_control.URControl import URNode


def parse_path_file(path):
    points = []
    with open(path, 'r', encoding='utf-8') as path_file:
        for line_number, line in enumerate(path_file, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue

            values = stripped.split()
            if len(values) != 6:
                raise ValueError(
                    f'{path}:{line_number} expected 6 values, got {len(values)}'
                )

            try:
                points.append([float(value) for value in values])
            except ValueError as exc:
                raise ValueError(f'{path}:{line_number} contains non-numeric values') from exc

    return points


class MoveTestNode(URNode):
    def __init__(self):
        super().__init__()
        default_path = (
            Path(get_package_share_directory('pointcloud_planner')) /
            'data' / 'path_planning' / 'path_map_concave.txt'
        )

        self.declare_parameter('path_file', str(default_path))
        self.declare_parameter('start_index', 0)
        self.declare_parameter('max_points', 0)
        self.declare_parameter('move_acceleration', 0.1)
        self.declare_parameter('move_velocity', 0.1)
        self.declare_parameter('move_time', 0.0)
        self.declare_parameter('blend_radius', 0.0)
        self.declare_parameter('point_delay_s', 0.2)
        self.declare_parameter('robot_state_topic', 'robotstate')
        self.declare_parameter('target_position_tolerance_m', 0.003)
        self.declare_parameter('target_rotation_tolerance_rad', 0.03)
        self.declare_parameter('target_timeout_s', 30.0)
        self.declare_parameter('dry_run', False)
        self.latest_robot_state = None
        self.robot_state_sub = self.create_subscription(
            RobotState,
            self.get_parameter('robot_state_topic').value,
            self.robot_state_callback,
            10,
        )

    def robot_state_callback(self, msg):
        self.latest_robot_state = msg

    def is_pose_reached(self, target_pose, current_pose, position_tolerance, rotation_tolerance):
        position_error = math.sqrt(sum(
            (current_pose[index] - target_pose[index]) ** 2 for index in range(3)
        ))
        rotation_error = math.sqrt(sum(
            (current_pose[index] - target_pose[index]) ** 2 for index in range(3, 6)
        ))
        return position_error <= position_tolerance and rotation_error <= rotation_tolerance

    def wait_until_target_reached(self, target_pose, point_index):
        position_tolerance = float(self.get_parameter('target_position_tolerance_m').value)
        rotation_tolerance = float(self.get_parameter('target_rotation_tolerance_rad').value)
        timeout_s = float(self.get_parameter('target_timeout_s').value)
        start_time = time.monotonic()

        self.get_logger().info(
            f'Waiting until path point {point_index} is reached: '
            f'position_tolerance={position_tolerance}, rotation_tolerance={rotation_tolerance}'
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_robot_state is not None:
                current_pose = list(self.latest_robot_state.carte_pos)
                if self.is_pose_reached(target_pose, current_pose, position_tolerance, rotation_tolerance):
                    self.get_logger().info(f'Path point {point_index} reached.')
                    return

            if time.monotonic() - start_time > timeout_s:
                raise TimeoutError(f'Path point {point_index} was not reached within {timeout_s:.1f}s')

    def run(self):
        path_file = Path(self.get_parameter('path_file').value).expanduser()
        start_index = int(self.get_parameter('start_index').value)
        max_points = int(self.get_parameter('max_points').value)
        move_acceleration = float(self.get_parameter('move_acceleration').value)
        move_velocity = float(self.get_parameter('move_velocity').value)
        move_time = float(self.get_parameter('move_time').value)
        blend_radius = float(self.get_parameter('blend_radius').value)
        point_delay_s = float(self.get_parameter('point_delay_s').value)
        dry_run = bool(self.get_parameter('dry_run').value)

        points = parse_path_file(path_file)
        if start_index < 0 or start_index >= len(points):
            raise ValueError(f'start_index {start_index} is outside path range 0-{len(points) - 1}')

        selected_points = points[start_index:]
        if max_points > 0:
            selected_points = selected_points[:max_points]

        self.get_logger().info(
            f'Loaded {len(points)} path points from {path_file}; '
            f'executing {len(selected_points)} points from index {start_index}; dry_run={dry_run}'
        )

        for offset, pose in enumerate(selected_points):
            point_index = start_index + offset
            self.get_logger().info(f'Path point {point_index}: {pose}')
            if not dry_run:
                self.movel(
                    pose,
                    a=move_acceleration,
                    v=move_velocity,
                    t=move_time,
                    r=blend_radius,
                )
                self.wait_until_target_reached(pose, point_index)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(point_delay_s)

        self.get_logger().info('Move test finished.')


def main(args=None):
    rclpy.init(args=args)
    node = MoveTestNode()
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
