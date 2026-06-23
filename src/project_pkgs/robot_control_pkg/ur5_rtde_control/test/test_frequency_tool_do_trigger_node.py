import sys
import types

import pytest


class _FakeNode:
    pass


sys.modules.setdefault(
    "rclpy",
    types.SimpleNamespace(init=lambda args=None: None),
)
sys.modules.setdefault("rclpy.node", types.SimpleNamespace(Node=_FakeNode))
sys.modules.setdefault("geometry_msgs.msg", types.SimpleNamespace(Pose=object))
sys.modules.setdefault(
    "std_srvs.srv",
    types.SimpleNamespace(SetBool=object),
)
sys.modules.setdefault(
    "rtde_io",
    types.SimpleNamespace(RTDEIOInterface=object),
)

from ur5_rtde_control.frequency_tool_do_trigger_node import (  # noqa: E402
    find_ultra_scanning_data_dir,
    resolve_records_file,
    validate_trigger_timing,
)


def test_relative_records_file_is_under_ultra_scanning_data():
    path = resolve_records_file("ultrasound_triggers.csv")

    assert path.name == "ultrasound_triggers.csv"
    assert path.parent.name == "data"
    assert path.parent.parent.name == "ultra_scanning_system"


def test_data_dir_can_be_found_from_install_prefix(tmp_path):
    data_dir = (
        tmp_path
        / "src/project_pkgs/robot_control_pkg/ultra_scanning_system/data"
    )
    data_dir.mkdir(parents=True)
    installed_module = (
        tmp_path
        / "install/ur5_rtde_control/lib/python3.10/site-packages/module.py"
    )

    assert find_ultra_scanning_data_dir([installed_module]) == data_dir


def test_validate_trigger_timing_returns_period():
    assert validate_trigger_timing(20.0, 0.001) == 0.05


def test_validate_trigger_timing_rejects_overlapping_pulses():
    with pytest.raises(ValueError):
        validate_trigger_timing(10.0, 0.1)
