import sys
import types


class _FakeNode:
    pass


sys.modules.setdefault("rclpy", types.SimpleNamespace(init=lambda args=None: None))
sys.modules.setdefault("rclpy.node", types.SimpleNamespace(Node=_FakeNode))
sys.modules.setdefault(
    "rclpy.qos",
    types.SimpleNamespace(DurabilityPolicy=object, QoSProfile=object),
)
sys.modules.setdefault("geometry_msgs.msg", types.SimpleNamespace(Pose=object))
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
