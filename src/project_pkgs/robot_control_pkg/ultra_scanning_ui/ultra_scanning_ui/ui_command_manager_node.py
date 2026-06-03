"""ROS2 node that launches programs from UI control commands."""

import subprocess
import shlex

import rclpy
from rclpy.node import Node

from ui_control_msg.msg import UiControl

from ultra_scanning_ui.ui_command_manager_logic import (
    CommandSpec,
    command_for_flag,
    default_command_specs,
    name_for_flag,
)


class UICommandManagerNode(Node):
    """Subscribe to UI commands and manage the corresponding child processes."""

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
        """Load default commands, allowing launch parameters to override them."""
        specs = []
        for default_spec in default_command_specs():
            parameter_name = f"command_{default_spec.flag}"
            self.declare_parameter(parameter_name, " ".join(default_spec.command))
            command_text = str(self.get_parameter(parameter_name).value).strip()
            command = shlex.split(command_text) if command_text else []
            specs.append(
                CommandSpec(
                    flag=default_spec.flag, # 原始默认值中的 flag 不变，保持与 UI 控制标志一致
                    name=default_spec.name, # 原始默认值中的 name 不变，保持可读性
                    command=command, # 从参数解析的命令覆盖默认值，允许用户通过参数自定义命令
                )
            )
        return specs

    def _on_ui_control(self, msg: UiControl) -> None:
        """Dispatch one UI control command."""
        # 获取 control_flag 并查找对应命令
        flag = int(msg.control_flag)
        if flag == 5:
            self.get_logger().info("Exit command received; stopping child processes.")
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
        # 检查是否有正在运行的进程，如果有则不启动新进程，避免重复启动同一命令
        if existing is not None and existing.poll() is None:
            self.get_logger().warn(
                f"Command already running for control_flag={flag} "
                f"({name}), pid={existing.pid}."
            )
            return

        # 启动新进程
        self.get_logger().info(
            f"Starting command for control_flag={flag} ({name}): {' '.join(command)}"
        )
        self.processes[flag] = subprocess.Popen(command)

    def _stop_all_processes(self) -> None:
        """Terminate every child process started by this manager."""
        for flag, process in list(self.processes.items()):
            if process.poll() is not None:
                continue
            name = name_for_flag(self.command_specs, flag)
            self.get_logger().info(
                f"Stopping command for control_flag={flag} ({name}), pid={process.pid}."
            )
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    f"Command did not stop cleanly; killing pid={process.pid}."
                )
                process.kill()
                process.wait(timeout=1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UICommandManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_all_processes()
    finally:
        if rclpy.ok():
            node._stop_all_processes()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
