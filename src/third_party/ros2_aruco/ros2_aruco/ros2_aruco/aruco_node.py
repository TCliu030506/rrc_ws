"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


def image_msg_to_mono8(img_msg):
    if img_msg.encoding == "mono8":
        return np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width
        )

    if img_msg.encoding in ("rgb8", "bgr8"):
        color_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, 3
        )
        if img_msg.encoding == "rgb8":
            return cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        return cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    raise ValueError(f"Unsupported image encoding: {img_msg.encoding}")


def rotation_matrix_to_quaternion(rotation_matrix):
    trace = np.trace(rotation_matrix)
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / scale
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / scale
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / scale
    elif (
        rotation_matrix[0, 0] > rotation_matrix[1, 1]
        and rotation_matrix[0, 0] > rotation_matrix[2, 2]
    ):
        scale = np.sqrt(
            1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1]
            - rotation_matrix[2, 2]
        ) * 2.0
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / scale
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / scale
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        scale = np.sqrt(
            1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0]
            - rotation_matrix[2, 2]
        ) * 2.0
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / scale
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / scale
    else:
        scale = np.sqrt(
            1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0]
            - rotation_matrix[1, 1]
        ) * 2.0
        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / scale
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / scale
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / scale
        qz = 0.25 * scale

    return np.array([qx, qy, qz, qw])


def get_predefined_dictionary(dictionary_id):
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def detect_markers(cv_image, aruco_dictionary, aruco_parameters):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
        return detector.detectMarkers(cv_image)

    return cv2.aruco.detectMarkers(
        cv_image, aruco_dictionary, parameters=aruco_parameters
    )


def marker_object_points(marker_size):
    half_size = marker_size / 2.0
    return np.array(
        [
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ],
        dtype=np.float32,
    )


def estimate_marker_poses(corners, marker_size, intrinsic_mat, distortion):
    object_points = marker_object_points(marker_size)
    rvecs = []
    tvecs = []

    for marker_corners in corners:
        image_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            intrinsic_mat,
            distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                intrinsic_mat,
                distortion,
            )
        if success:
            rvecs.append(rvec.reshape(3))
            tvecs.append(tvec.reshape(3))

    return rvecs, tvecs


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = get_predefined_dictionary(dictionary_id)
        self.aruco_parameters = create_detector_parameters()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        try:
            cv_image = image_msg_to_mono8(img_msg)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = detect_markers(
            cv_image, self.aruco_dictionary, self.aruco_parameters
        )
        if marker_ids is not None:
            rvecs, tvecs = estimate_marker_poses(
                corners, self.marker_size, self.intrinsic_mat, self.distortion
            )
            for i, (marker_id, rvec, tvec) in enumerate(zip(marker_ids, rvecs, tvecs)):
                pose = Pose()
                pose.position.x = tvec[0]
                pose.position.y = tvec[1]
                pose.position.z = tvec[2]

                rot_matrix = cv2.Rodrigues(np.array(rvec))[0]
                quat = rotation_matrix_to_quaternion(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


def main():
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
