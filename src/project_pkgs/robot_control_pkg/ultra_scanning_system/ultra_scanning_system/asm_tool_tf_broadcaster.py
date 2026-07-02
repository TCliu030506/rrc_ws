#!/usr/bin/env python3

from __future__ import annotations

from typing import Iterable

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ultra_scanning_system.asm_tool_kinematics import (
    TransformSpec,
    default_dynamic_transforms,
    default_static_transforms,
    has_required_joint_states,
)


class AsmToolTfBroadcaster(Node):
    """Publish the ASM tool TF chain mounted after the UR tool frame."""

    def __init__(self) -> None:
        super().__init__("asm_tool_tf_broadcaster")

        self.declare_parameter("asm_version", 1)
        self.declare_parameter("parent_frame", "tool0")
        self.declare_parameter("encoder1_joint_state_topic", "encoder1/joint_state")
        self.declare_parameter("encoder2_joint_state_topic", "encoder2/joint_state")
        self.declare_parameter("encoder3_joint_state_topic", "encoder3/joint_state")
        self.declare_parameter("encoder1_joint_name", "brt_encoder1_joint")
        self.declare_parameter("encoder2_joint_name", "brt_encoder2_joint")
        self.declare_parameter("encoder3_joint_name", "brt_encoder3_joint")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("publish_without_joint_state", True)

        self.asm_version = int(self.get_parameter("asm_version").value)
        if self.asm_version not in (1, 2, 3, 4):
            raise ValueError("asm_version must be 1, 2, 3, or 4")

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.encoder1_joint_state_topic = str(
            self.get_parameter("encoder1_joint_state_topic").value
        )
        self.encoder2_joint_state_topic = str(
            self.get_parameter("encoder2_joint_state_topic").value
        )
        self.encoder3_joint_state_topic = str(
            self.get_parameter("encoder3_joint_state_topic").value
        )
        self.encoder1_joint_name = str(
            self.get_parameter("encoder1_joint_name").value
        )
        self.encoder2_joint_name = str(
            self.get_parameter("encoder2_joint_name").value
        )
        self.encoder3_joint_name = str(
            self.get_parameter("encoder3_joint_name").value
        )
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.publish_without_joint_state = bool(
            self.get_parameter("publish_without_joint_state").value
        )

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be > 0")

        self._joint1_position = 0.0
        self._joint2_position = 0.0
        self._joint3_position = 0.0
        self._received_joint1 = False
        self._received_joint2 = False
        self._received_joint3 = False

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._dynamic_broadcaster = TransformBroadcaster(self)

        self._static_broadcaster.sendTransform(
            self._to_tf_messages(
                default_static_transforms(
                    self.parent_frame,
                    asm_version=self.asm_version,
                )
            )
        )

        self.create_subscription(
            JointState,
            self.encoder1_joint_state_topic,
            self._on_encoder1_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.encoder2_joint_state_topic,
            self._on_encoder2_joint_state,
            qos_profile_sensor_data,
        )
        if self.asm_version in (2, 4):
            self.create_subscription(
                JointState,
                self.encoder3_joint_state_topic,
                self._on_encoder3_joint_state,
                qos_profile_sensor_data,
            )
        self.create_timer(1.0 / self.publish_rate, self._publish_dynamic_tf)

        self.get_logger().info(
            "ASM tool TF broadcaster started: "
            f"asm_version={self.asm_version}, "
            f"parent_frame={self.parent_frame}, "
            f"encoder1={self.encoder1_joint_state_topic}/"
            f"{self.encoder1_joint_name}, "
            f"encoder2={self.encoder2_joint_state_topic}/"
            f"{self.encoder2_joint_name}, "
            f"encoder3={self.encoder3_joint_state_topic}/"
            f"{self.encoder3_joint_name}"
        )

    def _on_encoder1_joint_state(self, msg: JointState) -> None:
        position = self._read_joint_position(msg, self.encoder1_joint_name)
        if position is None:
            return
        self._joint1_position = position
        self._received_joint1 = True

    def _on_encoder2_joint_state(self, msg: JointState) -> None:
        position = self._read_joint_position(msg, self.encoder2_joint_name)
        if position is None:
            return
        self._joint2_position = position
        self._received_joint2 = True

    def _on_encoder3_joint_state(self, msg: JointState) -> None:
        position = self._read_joint_position(msg, self.encoder3_joint_name)
        if position is None:
            return
        self._joint3_position = position
        self._received_joint3 = True

    @staticmethod
    def _read_joint_position(
        msg: JointState,
        joint_name: str,
    ) -> float | None:
        if not msg.position:
            return None

        if joint_name and msg.name:
            try:
                index = list(msg.name).index(joint_name)
            except ValueError:
                index = -1
            if 0 <= index < len(msg.position):
                return float(msg.position[index])

        if len(msg.position) == 1:
            return float(msg.position[0])

        return None

    def _publish_dynamic_tf(self) -> None:
        if (
            not self.publish_without_joint_state
            and not has_required_joint_states(
                self.asm_version,
                self._received_joint1,
                self._received_joint2,
                self._received_joint3,
            )
        ):
            return

        transforms = default_dynamic_transforms(
            self._joint1_position,
            self._joint2_position,
            self._joint3_position,
            asm_version=self.asm_version,
        )
        self._dynamic_broadcaster.sendTransform(
            self._to_tf_messages(transforms)
        )

    def _to_tf_messages(
        self,
        transforms: Iterable[TransformSpec],
    ) -> list[TransformStamped]:
        now = self.get_clock().now().to_msg()
        messages = []
        for transform in transforms:
            msg = TransformStamped()
            msg.header.stamp = now
            msg.header.frame_id = transform.parent
            msg.child_frame_id = transform.child
            msg.transform.translation.x = transform.xyz[0]
            msg.transform.translation.y = transform.xyz[1]
            msg.transform.translation.z = transform.xyz[2]
            msg.transform.rotation.x = transform.quat_xyzw[0]
            msg.transform.rotation.y = transform.quat_xyzw[1]
            msg.transform.rotation.z = transform.quat_xyzw[2]
            msg.transform.rotation.w = transform.quat_xyzw[3]
            messages.append(msg)
        return messages


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AsmToolTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
