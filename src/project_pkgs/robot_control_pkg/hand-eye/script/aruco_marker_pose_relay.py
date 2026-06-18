#!/usr/bin/env python3


def select_marker_pose(markers_msg, target_marker_id):
    for marker_id, pose in zip(markers_msg.marker_ids, markers_msg.poses):
        if int(marker_id) == int(target_marker_id):
            return pose
    return None


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from ros2_aruco_interfaces.msg import ArucoMarkers

    class ArucoMarkerPoseRelay(Node):
        def __init__(self):
            super().__init__("aruco_marker_pose_relay")
            self.declare_parameter("target_marker_id", 0)
            self.declare_parameter("input_topic", "aruco_markers")
            self.declare_parameter("output_topic", "aruco_single/pose")

            self.target_marker_id = (
                self.get_parameter("target_marker_id")
                .get_parameter_value()
                .integer_value
            )
            input_topic = (
                self.get_parameter("input_topic").get_parameter_value().string_value
            )
            output_topic = (
                self.get_parameter("output_topic").get_parameter_value().string_value
            )

            self.pose_pub = self.create_publisher(PoseStamped, output_topic, 10)
            self.create_subscription(
                ArucoMarkers, input_topic, self.markers_callback, 10
            )

            self.get_logger().info(
                f"Relaying marker id {self.target_marker_id} from "
                f"{input_topic} to {output_topic}"
            )

        def markers_callback(self, markers_msg):
            pose = select_marker_pose(markers_msg, self.target_marker_id)
            if pose is None:
                return

            pose_msg = PoseStamped()
            pose_msg.header = markers_msg.header
            pose_msg.pose = pose
            self.pose_pub.publish(pose_msg)

    rclpy.init(args=args)
    node = ArucoMarkerPoseRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
