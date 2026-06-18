import math
import time

from robot_trajectory_planner.path_map_trajectory_logic import PoseState

from ultra_scanning_system.contact_scan_trajectory_node import (
    ContactScanTrajectoryNode,
)
from ultra_scanning_system.contact_scan_trajectory_logic import ContactScanState


class FakeLogger:
    def info(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


def test_timer_does_not_periodically_publish_state_without_state_change():
    node = object.__new__(ContactScanTrajectoryNode)
    node.last_time = None
    node.current_pose = None
    publish_calls = []
    node._publish_state = lambda: publish_calls.append('state')
    node.get_logger = lambda: FakeLogger()

    ContactScanTrajectoryNode._on_timer(node)

    assert publish_calls == []


def test_approach_uses_dedicated_linear_speed_for_initial_move():
    node = object.__new__(ContactScanTrajectoryNode)
    target_pose = PoseState(
        position=(0.1, 0.2, 0.3),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    node.last_time = None
    node.current_pose = PoseState(
        position=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    node.latest_wrench = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    node.state = ContactScanState.APPROACH
    node.force_axis = 'z'
    node.force_axis_sign = -1.0
    node.max_contact_force = 20.0
    node.path_points = [target_pose]
    node.approach_linear_speed = 0.002
    captured = {}
    command = type(
        'Command',
        (),
        {
            'finished': False,
            'pose': target_pose,
            'twist_linear': (0.0, 0.0, 0.0),
            'twist_angular': (0.0, 0.0, 0.0),
        },
    )()
    node._publish_measured_control_wrench = lambda: None

    def capture_trajectory(
        waypoints,
        loop_path,
        start_pose=None,
        max_linear_speed=None,
    ):
        captured.update(
            waypoints=waypoints,
            max_linear_speed=max_linear_speed,
        )

    node._ensure_trajectory = capture_trajectory
    node._advance_trajectory = lambda dt: command
    node._publish_command = lambda command: None
    node._set_state = lambda new_state, reason: setattr(node, 'state', new_state)

    ContactScanTrajectoryNode._on_timer(node)

    assert captured['waypoints'] == [target_pose]
    assert captured['max_linear_speed'] == node.approach_linear_speed


def test_pre_contact_enters_settle_from_measured_force_and_actual_offset():
    node = object.__new__(ContactScanTrajectoryNode)
    last_command_pose = PoseState(
        position=(0.1, 0.2, 0.313),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    last_published_pose = PoseState(
        position=(0.1, 0.2, 0.312),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    node.last_time = None
    node.current_pose = PoseState(
        position=(0.1, 0.2, 0.314),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    node.latest_wrench = (0.0, 0.0, -12.0, 0.0, 0.0, 0.0)
    node.latest_wrench_frame = 'asm_force_sensor_link'
    node.state = ContactScanState.PRE_CONTACT
    node.force_axis = 'z'
    node.force_axis_sign = -1.0
    node.contact_force_threshold = 10.0
    node.target_contact_force = 15.0
    node.max_contact_force = 20.0
    node.search_distance = 0.004
    node.approach_axis_sign = 1.0
    node.contact_settle_pose = None
    node.last_pre_contact_command_pose = last_command_pose
    node.last_published_desired_pose = last_published_pose
    node.contact_settle_stable_time = 0.0
    node.contact_path_points = []
    node.path_points = [
        PoseState(
            position=(0.1, 0.2, 0.3),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
    ]
    published_wrenches = []
    node._publish_measured_control_wrench = (
        lambda: published_wrenches.append(node.latest_wrench)
    )
    node._ensure_trajectory = lambda *args, **kwargs: None
    node._set_state = lambda new_state, reason: setattr(node, 'state', new_state)
    node.get_logger = lambda: FakeLogger()

    ContactScanTrajectoryNode._on_timer(node)

    assert node.state == ContactScanState.CONTACT_SETTLE
    assert node.contact_force_ref == 12.0
    assert node.contact_settle_pose == last_published_pose
    assert math.isclose(node.delta_c, 0.014)
    assert node.contact_path_points[0].position == (0.1, 0.2, 0.314)
    assert published_wrenches == [node.latest_wrench]


def test_contact_settle_control_wrench_starts_from_pre_contact_measured_wrench():
    node = object.__new__(ContactScanTrajectoryNode)
    node.latest_wrench_frame = 'asm_force_sensor_link'
    node.control_wrench_ramp_frame = node.latest_wrench_frame
    node.control_wrench_ref = (1.0, 2.0, -6.0, 0.3, -0.2, 0.1)
    node.force_axis = 'z'
    node.force_axis_sign = -1.0
    node.contact_force_ref = 6.0
    node.target_contact_force = 15.0
    node.force_ramp_rate = 0.5
    node.zero_torque_rx_ry_enabled = True
    node.target_torque_rx = 0.0
    node.target_torque_ry = 0.0
    node.torque_ramp_rate = 0.05
    published = []
    node._publish_control_wrench_vector = (
        lambda wrench, frame_id: published.append((wrench, frame_id))
    )

    ContactScanTrajectoryNode._publish_target_control_wrench(node, 0.0)

    assert published == [
        (
            (1.0, 2.0, -6.0, 0.3, -0.2, 0.1),
            'asm_force_sensor_link',
        )
    ]


def test_target_control_wrench_uses_torque_ramp_for_rx_ry_zero_torque():
    node = object.__new__(ContactScanTrajectoryNode)
    node.control_wrench_ramp_frame = 'asm_force_sensor_link'
    node.control_wrench_ref = (0.0, 0.0, -10.0, 0.30, -0.20, 0.10)
    node.force_axis = 'z'
    node.force_axis_sign = -1.0
    node.contact_force_ref = 10.0
    node.target_contact_force = 10.0
    node.force_ramp_rate = 5.0
    node.torque_ramp_rate = 0.05
    node.zero_torque_rx_ry_enabled = True
    node.target_torque_rx = 0.0
    node.target_torque_ry = 0.0
    published = []
    node._publish_control_wrench_vector = (
        lambda wrench, frame_id: published.append((wrench, frame_id))
    )

    ContactScanTrajectoryNode._publish_target_control_wrench(node, 1.0)

    wrench, frame_id = published[0]
    assert frame_id == 'asm_force_sensor_link'
    assert all(
        math.isclose(actual, expected)
        for actual, expected in zip(
            wrench,
            (0.0, 0.0, -10.0, 0.25, -0.15, 0.05),
        )
    )


def test_pre_contact_publishes_measured_wrench_to_cancel_admittance_before_contact():
    node = object.__new__(ContactScanTrajectoryNode)
    node.last_time = None
    node.current_pose = PoseState(
        position=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    node.latest_wrench = (1.0, 2.0, -0.5, 0.1, 0.2, 0.3)
    node.latest_wrench_frame = 'asm_force_sensor_link'
    node.state = ContactScanState.PRE_CONTACT
    node.force_axis = 'z'
    node.force_axis_sign = -1.0
    node.contact_force_threshold = 10.0
    node.max_contact_force = 20.0
    node.search_distance = 0.004
    node.pre_contact_speed = 0.001
    node.max_search_distance = 0.1
    node.approach_axis_sign = 1.0
    node.path_points = [
        PoseState(
            position=(0.0, 0.0, 0.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
    ]
    published_wrenches = []
    published_poses = []
    node._publish_measured_control_wrench = (
        lambda: published_wrenches.append(node.latest_wrench)
    )
    node._publish_pose_twist_accel = (
        lambda pose, linear, angular: published_poses.append((pose, linear, angular))
    )
    node._set_state = lambda new_state, reason: setattr(node, 'state', new_state)
    node.get_logger = lambda: FakeLogger()

    ContactScanTrajectoryNode._on_timer(node)

    assert node.state == ContactScanState.PRE_CONTACT
    assert published_wrenches == [node.latest_wrench]
    assert published_poses


def test_contact_settle_holds_pose_until_force_is_stable():
    node = object.__new__(ContactScanTrajectoryNode)
    hold_pose = PoseState(
        position=(0.1, 0.2, 0.3),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    contact_path_point = PoseState(
        position=(0.11, 0.2, 0.3),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    node.last_time = time.monotonic() - 0.2
    node.current_pose = hold_pose
    node.latest_wrench = (0.0, 0.0, -5.1, 0.0, 0.0, 0.0)
    node.state = ContactScanState.CONTACT_SETTLE
    node.force_axis = 'z'
    node.force_axis_sign = -1.0
    node.max_contact_force = 20.0
    node.contact_force_ref = 5.0
    node.target_contact_force = 5.0
    node.force_ramp_rate = 1.0
    node.contact_settle_duration = 0.1
    node.contact_settle_force_tolerance = 0.5
    node.contact_settle_stable_time = 0.0
    node.contact_settle_pose = hold_pose
    node.last_published_desired_pose = hold_pose
    node.contact_path_points = [contact_path_point]
    node.control_wrench_ref = (0.0, 0.0, -5.0, 0.0, 0.0, 0.0)
    node.control_wrench_ramp_frame = 'asm_force_sensor_link'
    node.zero_torque_rx_ry_enabled = True
    node.target_torque_rx = 0.0
    node.target_torque_ry = 0.0
    node.torque_ramp_rate = 0.05
    node.trajectory = None
    published_poses = []
    trajectory_waypoints = []
    node._publish_pose_twist_accel = (
        lambda pose, linear, angular: published_poses.append(pose)
    )
    node._publish_control_wrench = lambda signed_force: None
    node._publish_control_wrench_vector = lambda wrench, frame_id: None
    node._ensure_trajectory = (
        lambda waypoints, loop_path, start_pose=None: trajectory_waypoints.extend(
            waypoints
        )
    )
    node._set_state = lambda new_state, reason: setattr(node, 'state', new_state)
    node.get_clock = lambda: None
    node.get_logger = lambda: FakeLogger()

    ContactScanTrajectoryNode._on_timer(node)

    assert published_poses == [hold_pose]
    assert trajectory_waypoints == [hold_pose]
    assert node.state == ContactScanState.CONTACT_SCAN


def test_contact_scan_starts_from_last_desired_pose_after_settle():
    node = object.__new__(ContactScanTrajectoryNode)
    hold_pose = PoseState(
        position=(0.1, 0.2, 0.240),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    current_pose = PoseState(
        position=(0.1, 0.2, 0.236),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    contact_path_point = PoseState(
        position=(0.1, 0.2, 0.314),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    node.last_time = time.monotonic() - 0.2
    node.current_pose = current_pose
    node.latest_wrench = (0.0, 0.0, -5.1, 0.0, 0.0, 0.0)
    node.state = ContactScanState.CONTACT_SETTLE
    node.force_axis = 'z'
    node.force_axis_sign = -1.0
    node.max_contact_force = 20.0
    node.contact_force_ref = 5.0
    node.target_contact_force = 5.0
    node.force_ramp_rate = 1.0
    node.contact_settle_duration = 0.1
    node.contact_settle_force_tolerance = 0.5
    node.contact_settle_stable_time = 0.0
    node.contact_settle_pose = hold_pose
    node.last_published_desired_pose = hold_pose
    node.contact_path_points = [contact_path_point]
    node.control_wrench_ref = (0.0, 0.0, -5.0, 0.0, 0.0, 0.0)
    node.control_wrench_ramp_frame = 'asm_force_sensor_link'
    node.zero_torque_rx_ry_enabled = True
    node.target_torque_rx = 0.0
    node.target_torque_ry = 0.0
    node.torque_ramp_rate = 0.05
    node.trajectory = None
    captured = {}
    node._publish_pose_twist_accel = lambda pose, linear, angular: None
    node._publish_control_wrench_vector = lambda wrench, frame_id: None
    node._ensure_trajectory = lambda waypoints, loop_path, start_pose=None: captured.update(
        waypoints=waypoints,
        start_pose=start_pose,
    )
    node._set_state = lambda new_state, reason: setattr(node, 'state', new_state)
    node.get_logger = lambda: FakeLogger()

    ContactScanTrajectoryNode._on_timer(node)

    assert captured['waypoints'][0] == hold_pose
    assert captured['start_pose'] == hold_pose
    assert node.state == ContactScanState.CONTACT_SCAN


def test_fault_state_latches_first_fault_pose():
    node = object.__new__(ContactScanTrajectoryNode)
    first_pose = PoseState(
        position=(0.1, 0.2, 0.3),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    second_pose = PoseState(
        position=(0.4, 0.5, 0.6),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    node.state = ContactScanState.CONTACT_SCAN
    node.current_pose = first_pose
    node.get_logger = lambda: FakeLogger()
    node._publish_state = lambda: None

    ContactScanTrajectoryNode._set_state(
        node,
        ContactScanState.FAULT,
        reason='test fault',
    )
    node.current_pose = second_pose
    ContactScanTrajectoryNode._set_state(
        node,
        ContactScanState.FAULT,
        reason='repeat fault',
    )

    assert node.fault_hold_pose == first_pose
