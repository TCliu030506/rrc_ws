"""Command mapping helpers for the UI command manager node."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CommandSpec:
    """A command associated with one UI control flag."""

    flag: int
    name: str
    command: list[str]


def default_command_specs() -> list[CommandSpec]:
    """Return the built-in command mapping for the current UI buttons."""
    return [
        CommandSpec(
            flag=1,
            name="system_init",
            command=[
                "ros2",
                "launch",
                "ultra_scanning_system",
                "ultra_scanning_hardware_init.launch.py",
            ],
        ),
        CommandSpec(
            flag=2,
            name="plan_scan_path",
            command=[
                "ros2",
                "launch",
                "pointcloud_planner",
                "point_cloud_planning.launch.py",
            ],
        ),
        CommandSpec(
            flag=3,
            name="motion_scan",
            command=[
                "ros2",
                "launch",
                "ultra_scanning_system",
                "ultra_scanning_motion.launch.py",
            ],
        ),
        CommandSpec(flag=4, name="reconstruct_3d", command=[]),
    ]


def command_for_flag(specs: list[CommandSpec], flag: int) -> list[str]:
    """Return the command configured for a UI flag, or an empty list."""
    for spec in specs:
        if spec.flag == flag:
            return list(spec.command)
    return []


def name_for_flag(specs: list[CommandSpec], flag: int) -> str:
    """Return the readable command name configured for a UI flag."""
    for spec in specs:
        if spec.flag == flag:
            return spec.name
    return f"flag_{flag}"


def managed_process_popen_kwargs() -> dict[str, bool]:
    """Return Popen options for a managed process group."""
    return {"start_new_session": True}
