from typing import Sequence

import rclpy
from rclpy.node import Node
from rtde_io import RTDEIOInterface as RTDEIO


ROBOT_IP = "192.168.1.102"
TOOL_DO_INDEX = 0
TRIGGER_FREQUENCY_HZ = 5.0
PULSE_WIDTH_SEC = 0.005
TRIGGER_COUNT = 10
ACTIVE_LOW = True


class PeriodicToolIODemo(Node):
    """按定时器频率产生 Tool DO 脉冲的简单测试节点."""

    def __init__(self) -> None:
        super().__init__("periodic_tool_io_demo")

        self.declare_parameter("robot_ip", ROBOT_IP)
        self.declare_parameter("tool_do_index", TOOL_DO_INDEX)
        self.declare_parameter("trigger_frequency_hz", TRIGGER_FREQUENCY_HZ)
        self.declare_parameter("pulse_width_sec", PULSE_WIDTH_SEC)
        self.declare_parameter("trigger_count", TRIGGER_COUNT)
        self.declare_parameter("active_low", ACTIVE_LOW)

        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.tool_do_index = int(self.get_parameter("tool_do_index").value)
        self.trigger_frequency_hz = float(
            self.get_parameter("trigger_frequency_hz").value
        )
        self.pulse_width_sec = float(self.get_parameter("pulse_width_sec").value)
        self.trigger_count = int(self.get_parameter("trigger_count").value)
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

        self.rtde_io = RTDEIO(self.robot_ip)
        self.sent_count = 0
        self.pulse_active = False
        self.pulse_timer = None
        self.done = False

        self.set_physical_level(False)
        self.create_timer(self.period_sec, self._trigger)

        self.get_logger().info(
            "Periodic tool IO demo started: "
            f"frequency={self.trigger_frequency_hz:.3f} Hz, "
            f"pulse_width={self.pulse_width_sec:.3f} s, "
            f"trigger_count={self.trigger_count}, active_low={self.active_low}"
        )

    def robot_value_for_physical_level(self, physical_high: bool) -> bool:
        return not physical_high if self.active_low else physical_high

    def set_physical_level(self, physical_high: bool) -> None:
        self.rtde_io.setToolDigitalOut(
            self.tool_do_index,
            self.robot_value_for_physical_level(physical_high),
        )

    def _trigger(self) -> None:
        if self.done or self.pulse_active or self.sent_count >= self.trigger_count:
            return

        self.set_physical_level(True)
        self.sent_count += 1
        self.pulse_active = True
        self.pulse_timer = self.create_timer(self.pulse_width_sec, self._finish_pulse)

    def _finish_pulse(self) -> None:
        self.set_physical_level(False)
        self.pulse_active = False
        if self.pulse_timer is not None:
            self.destroy_timer(self.pulse_timer)
            self.pulse_timer = None
        if self.sent_count >= self.trigger_count:
            self.done = True
            self.get_logger().info(f"Completed {self.sent_count} tool DO triggers.")

    def destroy_node(self) -> bool:
        try:
            self.set_physical_level(False)
        except Exception as exc:
            self.get_logger().warn(f"Failed to reset tool DO: {exc}")
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PeriodicToolIODemo()
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
