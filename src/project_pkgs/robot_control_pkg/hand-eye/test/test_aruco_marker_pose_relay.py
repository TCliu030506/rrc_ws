import importlib.util
from pathlib import Path
from types import SimpleNamespace


def load_relay_module():
    script_path = (
        Path(__file__).resolve().parents[1]
        / "script"
        / "aruco_marker_pose_relay.py"
    )
    spec = importlib.util.spec_from_file_location("aruco_marker_pose_relay", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_select_marker_pose_returns_pose_for_target_marker_id():
    relay = load_relay_module()
    pose_0 = SimpleNamespace(name="pose_0")
    pose_1 = SimpleNamespace(name="pose_1")
    msg = SimpleNamespace(marker_ids=[2, 0], poses=[pose_0, pose_1])

    assert relay.select_marker_pose(msg, 0) is pose_1


def test_select_marker_pose_returns_none_when_target_is_absent():
    relay = load_relay_module()
    msg = SimpleNamespace(marker_ids=[1, 2], poses=[object(), object()])

    assert relay.select_marker_pose(msg, 0) is None
