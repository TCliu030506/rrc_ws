#!/usr/bin/env python3

import ast
from typing import List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_visualizer_node')

        self.declare_parameter('camera_name', 'camera_fixed')
        self.declare_parameter('camera_names', '')
        self.declare_parameter('camera_topic_prefix', 'camera')
        self.declare_parameter('show_depth', False)

        self.camera_name = str(self.get_parameter('camera_name').value)
        self.camera_names = self._parse_camera_names(
            self.get_parameter('camera_names').value
        )
        self.camera_topic_prefix = str(
            self.get_parameter('camera_topic_prefix').value
        )
        self.show_depth = bool(self.get_parameter('show_depth').value)

        self.subscription_handles = []
        if self.camera_names:
            for camera_name in self.camera_names:
                self._subscribe_camera(camera_name)
        else:
            self._subscribe_single_legacy_camera()

        self.get_logger().info(
            f'Camera visualizer started. cameras={self.camera_names or [self.camera_name]}, show_depth={self.show_depth}'
        )

    def _parse_camera_names(self, value: object) -> List[str]:
        """解析相机名称列表，支持多种输入格式"""
        if value is None:
            return []
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return []
            # 尝试解析列表格式 ["a", "b"] 或 ['a', 'b']
            if text.startswith('[') or text.startswith('('):
                try:
                    parsed = ast.literal_eval(text)
                    if isinstance(parsed, (list, tuple)):
                        return [str(name).strip() for name in parsed if str(name).strip()]
                except (ValueError, SyntaxError):
                    pass
            # 逗号分隔格式 "a,b,c"
            if ',' in text:
                return [name.strip().strip("'\"") for name in text.split(',') if name.strip()]
            # 单个相机名称
            return [text]
        # 如果是列表类型
        if isinstance(value, (list, tuple)):
            return [str(name).strip() for name in value if str(name).strip()]
        return []

    def _subscribe_single_legacy_camera(self) -> None:
        self.subscription_handles.append(
            self.create_subscription(
                Image,
                'camera/rgb/image_raw',
                self._rgb_legacy_callback,
                qos_profile_sensor_data,
            )
        )
        cv2.namedWindow('RGB Camera', cv2.WINDOW_NORMAL)
        
        if self.show_depth:
            self.subscription_handles.append(
                self.create_subscription(
                    Image,
                    'camera/depth/image_raw',
                    self._depth_legacy_callback,
                    qos_profile_sensor_data,
                )
            )
            cv2.namedWindow('Depth Camera', cv2.WINDOW_NORMAL)

    def _subscribe_camera(self, camera_name: str) -> None:
        rgb_topic = f'{self.camera_topic_prefix}/{camera_name}/rgb/image_raw'
        self.subscription_handles.append(
            self.create_subscription(
                Image,
                rgb_topic,
                lambda msg, name=camera_name: self._rgb_callback(msg, name),
                qos_profile_sensor_data,
            )
        )

        cv2.namedWindow(f'{camera_name} RGB', cv2.WINDOW_NORMAL)
        
        if self.show_depth:
            depth_topic = f'{self.camera_topic_prefix}/{camera_name}/depth/image_raw'
            self.subscription_handles.append(
                self.create_subscription(
                    Image,
                    depth_topic,
                    lambda msg, name=camera_name: self._depth_callback(msg, name),
                    qos_profile_sensor_data,
                )
            )
            cv2.namedWindow(f'{camera_name} Depth', cv2.WINDOW_NORMAL)

    def _rgb_legacy_callback(self, msg: Image) -> None:
        self._display_rgb(msg, 'RGB Camera')

    def _depth_legacy_callback(self, msg: Image) -> None:
        self._display_depth(msg, 'Depth Camera')

    def _rgb_callback(self, msg: Image, camera_name: str) -> None:
        self._display_rgb(msg, f'{camera_name} RGB')

    def _depth_callback(self, msg: Image, camera_name: str) -> None:
        self._display_depth(msg, f'{camera_name} Depth')

    def _display_rgb(self, msg: Image, window_name: str) -> None:
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            cv2.imshow(window_name, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f'Failed to process RGB image: {exc}')

    def _display_depth(self, msg: Image, window_name: str) -> None:
        try:
            depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            valid_depth = depth_img[np.isfinite(depth_img) & (depth_img > 0.0)]
            if valid_depth.size > 0:
                depth_min = float(valid_depth.min())
                depth_max = float(valid_depth.max())
                if depth_max > depth_min:
                    normalized = (depth_img - depth_min) / (depth_max - depth_min) * 255.0
                else:
                    normalized = np.zeros_like(depth_img, dtype=np.float32)
                normalized = np.clip(normalized, 0, 255).astype(np.uint8)
            else:
                normalized = np.zeros_like(depth_img, dtype=np.uint8)
            depth_colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
            cv2.imshow(window_name, depth_colored)
            cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f'Failed to process depth image: {exc}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()