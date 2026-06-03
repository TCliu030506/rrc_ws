"""ROS2 node that launches programs from UI control commands."""

import os
import shlex
import signal
import subprocess

import rclpy
from rclpy.node import Node

from ui_control_msg.msg import UiControl

from ultra_scanning_ui.ui_command_manager_logic import (
    CommandSpec,
    command_for_flag,
    default_command_specs,
    managed_process_popen_kwargs,
    name_for_flag,
)


class UICommandManagerNode(Node):
    """Subscribe to UI commands and manage child processes."""

    def __init__(self) -> None:
        super().__init__("ui_command_manager_node")

        self.declare_parameter("control_topic", "tus_control")
        self.command_specs = self._load_command_specs()
        self.processes: dict[int, subprocess.Popen] = {}

        control_topic = str(self.get_parameter("control_topic").value)
        self.create_subscription(
            UiControl,
            control_topic,
            self._on_ui_control,
            10,
        )

        self.get_logger().info(
            f"UI command manager started: control_topic={control_topic}"
        )

    def _load_command_specs(self) -> list[CommandSpec]:
        """Load default commands, allowing parameters to override them."""
        specs = []
        for default_spec in default_command_specs():
            parameter_name = f"command_{default_spec.flag}"
            self.declare_parameter(
                parameter_name,
                " ".join(default_spec.command),
            )
            parameter_value = self.get_parameter(parameter_name).value
            command_text = str(parameter_value).strip()
            command = shlex.split(command_text) if command_text else []
            specs.append(
                CommandSpec(
                    flag=default_spec.flag,
                    name=default_spec.name,
                    command=command,
                )
            )
        return specs

    def _on_ui_control(self, msg: UiControl) -> None:
        """Dispatch one UI control command."""
        # 获取 control_flag 并查找对应命令
        flag = int(msg.control_flag)
        if flag == 5:
            self.get_logger().info(
                "Exit command received; stopping child processes."
            )
            self._stop_all_processes()
            rclpy.shutdown()
            return

        command = command_for_flag(self.command_specs, flag)
        name = name_for_flag(self.command_specs, flag)
        if not command:
            self.get_logger().warn(
                f"No command configured for control_flag={flag} ({name})."
            )
            return

        existing = self.processes.get(flag)
        # 避免重复启动同一命令。
        if existing is not None and existing.poll() is None:
            self.get_logger().warn(
                f"Command already running for control_flag={flag} "
                f"({name}), pid={existing.pid}."
            )
            return

        # 启动新进程
        self.get_logger().info(
            f"Starting command for control_flag={flag} ({name}): "
            f"{' '.join(command)}"
        )
        self.processes[flag] = subprocess.Popen(
            command,
            **managed_process_popen_kwargs(),
        )

    def _stop_all_processes(self) -> None:
        """Terminate every child process started by this manager."""
        for flag, process in list(self.processes.items()):
            if process.poll() is not None:
                continue
            name = name_for_flag(self.command_specs, flag)
            self.get_logger().info(
                f"Stopping command for control_flag={flag} "
                f"({name}), pid={process.pid}."
            )
            self._signal_process_group(process, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    "Command did not stop cleanly; "
                    f"killing process group pid={process.pid}."
                )
                self._signal_process_group(process, signal.SIGKILL)
                process.wait(timeout=1.0)

    def _signal_process_group(
        self,
        process: subprocess.Popen,
        sig: signal.Signals,
    ) -> None:
        """Signal the whole process group created for one managed command."""
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except ProcessLookupError:
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UICommandManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_all_processes()
    finally:
        node._stop_all_processes()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
