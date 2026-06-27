import socket
from typing import Sequence

import rclpy
from rclpy.node import Node


ROBOT_IP = "192.168.1.102"
TOOL_DO_INDEX = 0
TRIGGER_FREQUENCY_HZ = 100.0
PULSE_WIDTH_SEC = 0.009
TRIGGER_COUNT = 10000
URSCRIPT_PORT = 30002
URSCRIPT_TIMEOUT_SEC = 1.0
ACTIVE_LOW = True


def robot_value_for_physical_level(
    physical_high: bool,
    *,
    active_low: bool = ACTIVE_LOW,
) -> bool:
    """当前工具 DO 硬件为反相：物理高电平对应机器人逻辑 False."""
    return not physical_high if active_low else physical_high


def build_set_tool_do_script(tool_do_index: int, robot_value: bool) -> str:
    value_text = "True" if robot_value else "False"
    return (
        "sec periodic_tool_do_demo():\n"
        f"  set_tool_digital_out({int(tool_do_index)}, {value_text})\n"
        "end\n"
    )


class URScriptToolDO:
    def __init__(
        self,
        robot_ip: str,
        *,
        tool_do_index: int = TOOL_DO_INDEX,
        port: int = URSCRIPT_PORT,
        timeout_sec: float = URSCRIPT_TIMEOUT_SEC,
        active_low: bool = ACTIVE_LOW,
    ) -> None:
        self.robot_ip = robot_ip
        self.tool_do_index = tool_do_index
        self.port = port
        self.timeout_sec = timeout_sec
        self.active_low = active_low
        self.sock: socket.socket | None = None

    def connect(self) -> socket.socket:
        if self.sock is not None:
            return self.sock
        self.sock = socket.create_connection(
            (self.robot_ip, self.port),
            timeout=self.timeout_sec,
        )
        self.sock.settimeout(self.timeout_sec)
        return self.sock

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.close()
        finally:
            self.sock = None

    def send_script(self, script: str) -> None:
        data = script.encode("utf-8")
        try:
            self.connect().sendall(data)
        except OSError:
            self.close()
            self.connect().sendall(data)

    def set_physical_level(self, physical_high: bool) -> None:
        robot_value = robot_value_for_physical_level(
            physical_high,
            active_low=self.active_low,
        )
        self.send_script(build_set_tool_do_script(self.tool_do_index, robot_value))


class PeriodicToolIOURScriptDemo(Node):
    """按 ROS2 定时器频率通过 URScript 产生 Tool DO 脉冲."""

    def __init__(self) -> None:
        super().__init__("periodic_tool_io_demo_urscript")

        self.declare_parameter("robot_ip", ROBOT_IP)
        self.declare_parameter("tool_do_index", TOOL_DO_INDEX)
        self.declare_parameter("trigger_frequency_hz", TRIGGER_FREQUENCY_HZ)
        self.declare_parameter("pulse_width_sec", PULSE_WIDTH_SEC)
        self.declare_parameter("trigger_count", TRIGGER_COUNT)
        self.declare_parameter("urscript_port", URSCRIPT_PORT)
        self.declare_parameter("urscript_timeout_sec", URSCRIPT_TIMEOUT_SEC)
        self.declare_parameter("active_low", ACTIVE_LOW)

        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.tool_do_index = int(self.get_parameter("tool_do_index").value)
        self.trigger_frequency_hz = float(
            self.get_parameter("trigger_frequency_hz").value
        )
        self.pulse_width_sec = float(self.get_parameter("pulse_width_sec").value)
        self.trigger_count = int(self.get_parameter("trigger_count").value)
        self.urscript_port = int(self.get_parameter("urscript_port").value)
        self.urscript_timeout_sec = float(
            self.get_parameter("urscript_timeout_sec").value
        )
        self.active_low = bool(self.get_parameter("active_low").value)

        if self.trigger_frequency_hz <= 0.0:
            raise ValueError("trigger_frequency_hz must be > 0")
        if self.pulse_width_sec <= 0.0:
            raise ValueError("pulse_width_sec must be > 0")
        self.period_sec = 1.0 / self.trigger_frequency_hz
        if self.pulse_width_sec >= self.period_sec:
            raise ValueError("pulse_width_sec must be shorter than trigger period")
        if self.trigger_count <= 0:
            raise ValueError("trigger_count must be > 0")
        if self.urscript_timeout_sec <= 0.0:
            raise ValueError("urscript_timeout_sec must be > 0")

        self.tool_do = URScriptToolDO(
            self.robot_ip,
            tool_do_index=self.tool_do_index,
            port=self.urscript_port,
            timeout_sec=self.urscript_timeout_sec,
            active_low=self.active_low,
        )
        self.sent_count = 0
        self.pulse_active = False
        self.pulse_timer = None
        self.done = False

        self.tool_do.set_physical_level(False)
        self.create_timer(self.period_sec, self._trigger)

        self.get_logger().info(
            "Periodic Tool DO URScript demo started: "
            f"frequency={self.trigger_frequency_hz:.3f} Hz, "
            f"pulse_width={self.pulse_width_sec:.6f} s, "
            f"trigger_count={self.trigger_count}, "
            f"urscript_port={self.urscript_port}, active_low={self.active_low}"
        )

    def _trigger(self) -> None:
        if self.done or self.pulse_active or self.sent_count >= self.trigger_count:
            return

        try:
            self.tool_do.set_physical_level(True)
        except OSError as exc:
            self.get_logger().warn(f"Failed to set Tool DO high: {exc}")
            return

        self.sent_count += 1
        self.pulse_active = True
        self.pulse_timer = self.create_timer(self.pulse_width_sec, self._finish_pulse)

    def _finish_pulse(self) -> None:
        try:
            self.tool_do.set_physical_level(False)
        except OSError as exc:
            self.get_logger().warn(f"Failed to set Tool DO low: {exc}")

        self.pulse_active = False
        if self.pulse_timer is not None:
            self.destroy_timer(self.pulse_timer)
            self.pulse_timer = None
        if self.sent_count >= self.trigger_count:
            self.done = True
            self.get_logger().info(f"Completed {self.sent_count} Tool DO triggers.")

    def destroy_node(self) -> bool:
        try:
            self.tool_do.set_physical_level(False)
        except Exception as exc:
            self.get_logger().warn(f"Failed to reset Tool DO: {exc}")
        self.tool_do.close()
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PeriodicToolIOURScriptDemo()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
