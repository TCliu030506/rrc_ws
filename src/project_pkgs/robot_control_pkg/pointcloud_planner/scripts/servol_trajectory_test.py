#!/usr/bin/env python3
"""Run a path_map trajectory directly with RTDE servoL."""

import argparse
import math
import time
from pathlib import Path

import rtde_control
import rtde_receive


def default_path_file() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('pointcloud_planner')) /
            'data' / 'path_planning' / 'path_map_concave.txt'
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[1] /
            'data' / 'path_planning' / 'path_map_concave.txt'
        )


def parse_path_file(path: Path) -> list[list[float]]:
    points = []
    with path.open('r', encoding='utf-8') as path_file:
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
                raise ValueError(
                    f'{path}:{line_number} contains non-numeric values'
                ) from exc
    if not points:
        raise ValueError(f'{path} contains no path points')
    return points


def vector_norm(values) -> float:
    return math.sqrt(sum(value * value for value in values))


def rotvec_to_quat(rotvec):
    angle = vector_norm(rotvec)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    scale = math.sin(0.5 * angle) / angle
    return normalize_quat((
        rotvec[0] * scale,
        rotvec[1] * scale,
        rotvec[2] * scale,
        math.cos(0.5 * angle),
    ))


def quat_to_rotvec(quat):
    qx, qy, qz, qw = normalize_quat(quat)
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    vector = math.sqrt(qx * qx + qy * qy + qz * qz)
    if vector < 1e-12:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(vector, qw)
    scale = angle / vector
    return [qx * scale, qy * scale, qz * scale]


def normalize_quat(quat):
    qx, qy, qz, qw = quat
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def quat_dot(lhs, rhs) -> float:
    return sum(lhs[index] * rhs[index] for index in range(4))


def quat_angle(lhs, rhs) -> float:
    dot = abs(quat_dot(normalize_quat(lhs), normalize_quat(rhs)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def slerp(lhs, rhs, ratio: float):
    qa = normalize_quat(lhs)
    qb = normalize_quat(rhs)
    dot = quat_dot(qa, qb)
    if dot < 0.0:
        qb = tuple(-value for value in qb)
        dot = -dot

    if dot > 0.9995:
        return normalize_quat(tuple(
            qa[index] + (qb[index] - qa[index]) * ratio
            for index in range(4)
        ))

    dot = max(-1.0, min(1.0, dot))
    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    theta = theta0 * ratio
    scale_a = math.cos(theta) - dot * math.sin(theta) / sin_theta0
    scale_b = math.sin(theta) / sin_theta0
    return normalize_quat(tuple(
        qa[index] * scale_a + qb[index] * scale_b
        for index in range(4)
    ))


def pose_distance(start, target) -> tuple[float, float]:
    linear = vector_norm(target[index] - start[index] for index in range(3))
    angular = quat_angle(rotvec_to_quat(start[3:6]), rotvec_to_quat(target[3:6]))
    return linear, angular


def interpolate_pose(start, target, ratio: float) -> list[float]:
    ratio = max(0.0, min(1.0, ratio))
    position = [
        start[index] + (target[index] - start[index]) * ratio
        for index in range(3)
    ]
    orientation = slerp(
        rotvec_to_quat(start[3:6]),
        rotvec_to_quat(target[3:6]),
        ratio,
    )
    return position + quat_to_rotvec(orientation)


def build_servo_samples(
    waypoints,
    *,
    start_pose,
    linear_speed,
    angular_speed,
    dt,
):
    previous = list(start_pose)
    samples = []
    for target in waypoints:
        linear_distance, angular_distance = pose_distance(previous, target)
        duration = max(
            linear_distance / linear_speed,
            angular_distance / angular_speed,
            dt,
        )
        step_count = max(1, int(math.ceil(duration / dt)))
        for step in range(1, step_count + 1):
            samples.append(interpolate_pose(previous, target, step / step_count))
        previous = list(target)
    return samples


def parse_args():
    parser = argparse.ArgumentParser(
        description='Send a path_map trajectory to a UR robot using RTDE servoL.'
    )
    parser.add_argument('--robot-ip', default='192.168.1.102')
    parser.add_argument('--path-file', type=Path, default=default_path_file())
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--max-points', type=int, default=0)
    parser.add_argument('--linear-speed', type=float, default=0.008)
    parser.add_argument('--angular-speed', type=float, default=0.05)
    parser.add_argument('--servo-speed', type=float, default=0.15)
    parser.add_argument('--servo-acceleration', type=float, default=0.1)
    parser.add_argument('--dt', type=float, default=0.002)
    parser.add_argument('--lookahead-time', type=float, default=0.1)
    parser.add_argument('--gain', type=float, default=300.0)
    parser.add_argument('--settle-time', type=float, default=0.2)
    parser.add_argument(
        '--tcp',
        type=float,
        nargs=6,
        default=[0.0, 0.0, 0.150, 0.0, 0.0, math.radians(180)],
        metavar=('X', 'Y', 'Z', 'RX', 'RY', 'RZ'),
        help='TCP offset passed to rtde_control.setTcp, in meters/radians.',
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        help='Actually connect and move. Without this flag the script is dry-run.',
    )
    return parser.parse_args()


def validate_args(args):
    if args.start_index < 0:
        raise ValueError('--start-index must be >= 0')
    if args.max_points < 0:
        raise ValueError('--max-points must be >= 0')
    if args.linear_speed <= 0.0:
        raise ValueError('--linear-speed must be > 0')
    if args.angular_speed <= 0.0:
        raise ValueError('--angular-speed must be > 0')
    if args.servo_speed <= 0.0:
        raise ValueError('--servo-speed must be > 0')
    if args.servo_acceleration <= 0.0:
        raise ValueError('--servo-acceleration must be > 0')
    if args.dt <= 0.0:
        raise ValueError('--dt must be > 0')
    if args.lookahead_time <= 0.0:
        raise ValueError('--lookahead-time must be > 0')
    if args.gain <= 0.0:
        raise ValueError('--gain must be > 0')
    if args.settle_time < 0.0:
        raise ValueError('--settle-time must be >= 0')


def select_waypoints(points, start_index, max_points):
    if start_index >= len(points):
        raise ValueError(
            f'--start-index {start_index} is outside path range 0-{len(points) - 1}'
        )
    selected = points[start_index:]
    if max_points > 0:
        selected = selected[:max_points]
    return selected


def main():
    args = parse_args()
    validate_args(args)
    points = parse_path_file(args.path_file.expanduser())
    selected = select_waypoints(points, args.start_index, args.max_points)

    print(f'path_file: {args.path_file}')
    print(f'loaded_points: {len(points)}')
    print(f'selected_points: {len(selected)} from index {args.start_index}')
    print(f'tcp: {args.tcp}')
    print(f'execute: {args.execute}')

    rtde_c = None
    rtde_r = None
    try:
        if args.execute:
            rtde_c = rtde_control.RTDEControlInterface(args.robot_ip)
            rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
            if not rtde_c.isConnected() or not rtde_r.isConnected():
                raise RuntimeError('RTDE connection failed')
            rtde_c.setTcp(args.tcp)
            start_pose = [float(value) for value in rtde_r.getActualTCPPose()]
        else:
            start_pose = selected[0]

        samples = build_servo_samples(
            selected,
            start_pose=start_pose,
            linear_speed=args.linear_speed,
            angular_speed=args.angular_speed,
            dt=args.dt,
        )
        print(f'servo_samples: {len(samples)}')
        if not args.execute:
            print('dry-run only; add --execute to move the robot.')
            return

        settle_count = int(math.ceil(args.settle_time / args.dt))
        for _ in range(settle_count):
            rtde_c.servoL(
                start_pose,
                args.servo_speed,
                args.servo_acceleration,
                args.dt,
                args.lookahead_time,
                args.gain,
            )
            time.sleep(args.dt)

        for index, pose in enumerate(samples):
            t0 = time.perf_counter()
            rtde_c.servoL(
                pose,
                args.servo_speed,
                args.servo_acceleration,
                args.dt,
                args.lookahead_time,
                args.gain,
            )
            elapsed = time.perf_counter() - t0
            sleep_time = max(0.0, args.dt - elapsed)
            time.sleep(sleep_time)
            if index % 500 == 0:
                print(f'sent servo sample {index}/{len(samples)}')

        rtde_c.servoStop()
        print('servoL trajectory finished.')
    finally:
        if rtde_c is not None:
            try:
                rtde_c.stopScript()
            except Exception:
                pass
            rtde_c.disconnect()
        if rtde_r is not None:
            rtde_r.disconnect()


if __name__ == '__main__':
    main()
