from ultra_scanning_ui.ui_command_manager_logic import (
    command_for_flag,
    default_command_specs,
    managed_process_popen_kwargs,
)


def test_default_commands_cover_known_ui_buttons():
    specs = default_command_specs()

    assert command_for_flag(specs, 1) == [
        "ros2",
        "launch",
        "ultra_scanning_system",
        "ultra_scanning_hardware_init.launch.py",
    ]
    assert command_for_flag(specs, 2) == [
        "ros2",
        "launch",
        "pointcloudslam_cpp",
        "point_cloud_slam.launch.py",
    ]
    assert command_for_flag(specs, 3) == [
        "ros2",
        "launch",
        "ultra_scanning_system",
        "ultra_scanning_motion.launch.py",
    ]


def test_unconfigured_buttons_have_no_command():
    specs = default_command_specs()

    assert command_for_flag(specs, 4) == []
    assert command_for_flag(specs, 99) == []


def test_managed_processes_start_in_new_session():
    assert managed_process_popen_kwargs() == {"start_new_session": True}
