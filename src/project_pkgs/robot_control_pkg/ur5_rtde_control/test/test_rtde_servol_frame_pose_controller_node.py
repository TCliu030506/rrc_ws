import sys
import types


class _FakeNode:
    pass


class _FakePose:
    def __init__(self):
        self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class _FakePoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id='')
        self.pose = _FakePose()


sys.modules.setdefault(
    "geometry_msgs.msg",
    types.SimpleNamespace(Pose=_FakePose, PoseStamped=_FakePoseStamped),
)
sys.modules.setdefault(
    "rclpy",
    types.SimpleNamespace(
        init=lambda args=None: None,
        spin=lambda node: None,
        ok=lambda: False,
        shutdown=lambda: None,
        time=types.SimpleNamespace(Time=lambda: None),
    ),
)
sys.modules.setdefault("rclpy.duration", types.SimpleNamespace(Duration=object))
sys.modules.setdefault("rclpy.node", types.SimpleNamespace(Node=_FakeNode))
sys.modules.setdefault("std_msgs.msg", types.SimpleNamespace(Float64MultiArray=object))
sys.modules.setdefault(
    "tf2_ros",
    types.SimpleNamespace(
        Buffer=object,
        TransformException=Exception,
        TransformListener=object,
    ),
)
sys.modules.setdefault("rtde_control", types.SimpleNamespace(RTDEControlInterface=object))
sys.modules.setdefault("rtde_receive", types.SimpleNamespace(RTDEReceiveInterface=object))

from ur5_rtde_control.rtde_servol_frame_pose_controller_node import (
    transform_to_pose_msg,
)


def test_transform_to_pose_msg_preserves_transformed_tool_pose():
    msg = transform_to_pose_msg(
        ((1.0, 2.0, 3.0), (0.1, 0.2, 0.3, 0.9)),
    )

    assert msg.position.x == 1.0
    assert msg.position.y == 2.0
    assert msg.position.z == 3.0
    assert msg.orientation.x == 0.1
    assert msg.orientation.y == 0.2
    assert msg.orientation.z == 0.3
    assert msg.orientation.w == 0.9
