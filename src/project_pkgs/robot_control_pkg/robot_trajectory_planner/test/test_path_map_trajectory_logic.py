import math

import pytest

from robot_trajectory_planner.path_map_trajectory_logic import (
    PathMapTrajectory,
    PoseState,
    format_path_map_lines,
    offset_path_points_along_tool_z,
    parse_path_map_lines,
    quat_to_rotvec,
    resample_waypoints,
    rotvec_to_quat_xyzw,
)


def test_parse_path_map_lines_reads_positions_and_rotvec_orientation():
    points = parse_path_map_lines([
        '# x y z rx ry rz',
        '',
        '0.1 0.2 0.3 0.0 0.0 0.0',
        '0.4 0.5 0.6 3.141592653589793 0.0 0.0',
    ])

    assert len(points) == 2
    assert points[0].position == pytest.approx((0.1, 0.2, 0.3))
    assert points[0].orientation_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert points[1].position == pytest.approx((0.4, 0.5, 0.6))
    assert points[1].orientation_xyzw == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_parse_path_map_lines_rejects_bad_rows_with_line_number():
    with pytest.raises(ValueError, match='line 2'):
        parse_path_map_lines([
            '0.1 0.2 0.3 0.0 0.0 0.0',
            '0.1 0.2 0.3',
        ])


def test_rotvec_to_quat_xyzw_normalizes_zero_and_pi_rotation():
    assert rotvec_to_quat_xyzw((0.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 0.0, 0.0, 1.0)
    )
    assert rotvec_to_quat_xyzw((0.0, 0.0, math.pi)) == pytest.approx(
        (0.0, 0.0, 1.0, 0.0),
        abs=1e-12,
    )
    assert quat_to_rotvec(rotvec_to_quat_xyzw((0.0, math.pi / 2.0, 0.0))) == (
        pytest.approx(0.0),
        pytest.approx(math.pi / 2.0),
        pytest.approx(0.0),
    )


def test_offset_path_points_moves_each_point_along_its_local_z_axis():
    points = parse_path_map_lines([
        '0.0 0.0 0.0 0.0 0.0 0.0',
        f'1.0 2.0 3.0 0.0 {math.pi / 2.0} 0.0',
    ])

    offset_points = offset_path_points_along_tool_z(points, offset_distance=0.1)

    assert offset_points[0].position == pytest.approx((0.0, 0.0, 0.1))
    assert offset_points[1].position == pytest.approx((1.1, 2.0, 3.0))
    assert offset_points[0].orientation_xyzw == pytest.approx(
        points[0].orientation_xyzw
    )
    assert offset_points[1].orientation_xyzw == pytest.approx(
        points[1].orientation_xyzw
    )


def test_format_path_map_lines_writes_position_and_rotation_vector():
    points = parse_path_map_lines([
        f'1.0 2.0 3.0 0.0 {math.pi / 2.0} 0.0',
    ])

    lines = format_path_map_lines(points)

    fields = lines[0].split()
    assert [float(value) for value in fields[:3]] == pytest.approx([1.0, 2.0, 3.0])
    assert [float(value) for value in fields[3:]] == pytest.approx(
        [0.0, math.pi / 2.0, 0.0]
    )


def test_path_map_trajectory_blends_from_current_pose_then_follows_path():
    points = parse_path_map_lines([
        '0.10 0.00 0.00 0.0 0.0 0.0',
        '0.20 0.00 0.00 0.0 0.0 0.0',
    ])
    start = PoseState(
        position=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    trajectory = PathMapTrajectory(
        waypoints=points,
        start_pose=start,
        max_linear_speed=0.05,
        max_angular_speed=0.2,
        loop_path=False,
    )

    first = trajectory.advance(1.0)
    assert first.pose.position == pytest.approx((0.05, 0.0, 0.0))
    assert first.twist_linear == pytest.approx((0.05, 0.0, 0.0))
    assert first.path_index == 0

    second = trajectory.advance(1.0)
    assert second.pose.position == pytest.approx((0.10, 0.0, 0.0))
    assert second.path_index == 1

    third = trajectory.advance(1.0)
    assert third.pose.position == pytest.approx((0.15, 0.0, 0.0))
    assert third.twist_linear == pytest.approx((0.05, 0.0, 0.0))
    assert third.path_index == 1


def test_resample_waypoints_limits_linear_and_angular_steps():
    points = parse_path_map_lines([
        '0.0 0.0 0.0 0.0 0.0 0.0',
        '0.1 0.0 0.0 0.0 0.0 1.5707963267948966',
    ])

    resampled = resample_waypoints(
        points,
        max_linear_step=0.025,
        max_angular_step=0.5,
    )

    assert len(resampled) == 5
    assert resampled[0].position == pytest.approx((0.0, 0.0, 0.0))
    assert resampled[-1].position == pytest.approx((0.1, 0.0, 0.0))

    for previous, current in zip(resampled, resampled[1:]):
        dx = current.position[0] - previous.position[0]
        dy = current.position[1] - previous.position[1]
        dz = current.position[2] - previous.position[2]
        assert math.sqrt(dx * dx + dy * dy + dz * dz) <= 0.025 + 1e-12
