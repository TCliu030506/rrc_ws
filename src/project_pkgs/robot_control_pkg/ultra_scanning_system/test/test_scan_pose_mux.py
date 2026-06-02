import sys
import types
import math


class _Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _FakePose:
    def __init__(self):
        self.position = _Vector3()
        self.orientation = _Quaternion()


class _FakeNode:
    pass


sys.modules.setdefault("rclpy", types.SimpleNamespace(init=lambda args=None: None))
sys.modules.setdefault("rclpy.node", types.SimpleNamespace(Node=_FakeNode))
sys.modules.setdefault(
    "rclpy.qos",
    types.SimpleNamespace(DurabilityPolicy=object, QoSProfile=object),
)
sys.modules.setdefault("geometry_msgs.msg", types.SimpleNamespace(Pose=_FakePose))
sys.modules.setdefault("std_msgs.msg", types.SimpleNamespace(String=object))


def _should_use_direct_pose(state_value, direct_states):
    from ultra_scanning_system.scan_pose_mux import should_use_direct_pose

    return should_use_direct_pose(state_value, direct_states)


def test_should_use_direct_pose_only_for_configured_contact_states():
    direct_states = {'approach', 'pre_contact'}

    assert _should_use_direct_pose('approach', direct_states)
    assert _should_use_direct_pose('pre_contact', direct_states)
    assert not _should_use_direct_pose('contact_settle', direct_states)
    assert not _should_use_direct_pose('contact_scan', direct_states)


def test_should_use_direct_pose_falls_back_to_admittance_without_state():
    assert not _should_use_direct_pose('', {'approach', 'pre_contact'})


def test_blend_ratio_clamps_to_unit_interval():
    from ultra_scanning_system.scan_pose_mux import blend_ratio

    assert blend_ratio(-0.1, 0.5) == 0.0
    assert blend_ratio(0.25, 0.5) == 0.5
    assert blend_ratio(1.0, 0.5) == 1.0
    assert blend_ratio(0.0, 0.0) == 1.0


def test_interpolate_pose_msg_blends_position_and_orientation():
    from ultra_scanning_system.scan_pose_mux import interpolate_pose_msg

    start = _FakePose()
    start.position.z = 0.24
    start.orientation.w = 1.0
    target = _FakePose()
    target.position.z = 0.20
    target.orientation.z = 1.0
    target.orientation.w = 0.0

    blended = interpolate_pose_msg(start, target, 0.25)

    assert math.isclose(blended.position.z, 0.23)
    assert 0.0 < blended.orientation.z < 1.0
    assert 0.0 < blended.orientation.w < 1.0
