import math
import sys
import types


class _FakeNode:
    pass


class _FakeSetBool:
    class Request:
        pass

    class Response:
        pass


sys.modules.setdefault(
    "rclpy",
    types.SimpleNamespace(init=lambda args=None: None),
)
sys.modules.setdefault("rclpy.node", types.SimpleNamespace(Node=_FakeNode))
sys.modules.setdefault(
    "geometry_msgs.msg",
    types.SimpleNamespace(Pose=object),
)
sys.modules.setdefault(
    "std_srvs.srv",
    types.SimpleNamespace(SetBool=_FakeSetBool),
)
sys.modules.setdefault(
    "rtde_io",
    types.SimpleNamespace(RTDEIOInterface=object),
)

from ur5_rtde_control.tool_do_pose_trigger_node import (  # noqa: E402
    pose_record_to_row,
    projected_tool_x_distance,
    should_trigger_by_tool_x_distance,
)


def test_projected_tool_x_distance_uses_reference_tool_x_axis():
    reference_pose = (
        (1.0, 2.0, 3.0),
        (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    )
    current_pose = (
        (1.0, 2.0002, 3.0),
        (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    )

    distance = projected_tool_x_distance(reference_pose, current_pose)

    assert math.isclose(distance, 0.0002)


def test_should_trigger_by_absolute_tool_x_distance():
    reference_pose = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    current_pose = ((-0.00011, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    assert should_trigger_by_tool_x_distance(
        reference_pose,
        current_pose,
        trigger_distance=0.0001,
    )


def test_pose_record_to_row_is_csv_friendly():
    row = pose_record_to_row(
        12.3,
        ((1.0, 2.0, 3.0), (0.1, 0.2, 0.3, 0.9)),
    )

    assert row == [12.3, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9]
