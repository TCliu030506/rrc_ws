import math
import sys
import types


class _FakeNode:
    pass


sys.modules.setdefault(
    "geometry_msgs.msg",
    types.SimpleNamespace(Pose=object),
)
sys.modules.setdefault(
    "rclpy",
    types.SimpleNamespace(init=lambda args=None: None, spin=lambda node: None, ok=lambda: False, shutdown=lambda: None),
)
sys.modules.setdefault(
    "rclpy.node",
    types.SimpleNamespace(Node=_FakeNode),
)
sys.modules.setdefault(
    "std_msgs.msg",
    types.SimpleNamespace(Float64MultiArray=object),
)
sys.modules.setdefault("rtde_control", types.SimpleNamespace(RTDEControlInterface=object))
sys.modules.setdefault("rtde_receive", types.SimpleNamespace(RTDEReceiveInterface=object))

from ur5_rtde_control.rtde_servol_pose_controller_node import (
    URServoLPoseControllerNode,
)


def test_pose_error_uses_minimal_relative_rotation_angle():
    actual_pose = [0.1, -0.2, 0.3, 0.0, 0.0, 0.0]
    target_pose = [0.1, -0.2, 0.3, 0.0, 0.0, 2.0 * math.pi]

    pos_err, rot_err = URServoLPoseControllerNode.compute_pose_errors(
        actual_pose,
        target_pose,
    )

    assert pos_err == 0.0
    assert rot_err < 1e-9
