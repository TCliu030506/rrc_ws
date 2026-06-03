"""
遥操作超声扫查系统主界面.

功能：
- 创建 PyQt5 全屏界面，显示扫查图像、操作视角图像和操作按钮。
- 后台运行 ROS2 节点 `ui_node`，在 Qt 事件循环之外独立 spin。
- 将按钮操作发布为 `ui_control_msg/msg/UiControl` 控制指令。

ROS2 接口：
- 订阅 `us_images` (`sensor_msgs/msg/Image`)：显示到左侧“扫查图像”区域。
- 订阅 `/camera/camera/color/image_raw` (`sensor_msgs/msg/Image`)：显示到右侧“操作视角”区域，
  QoS 匹配相机 publisher 的 RELIABLE + TRANSIENT_LOCAL。
- 发布 `tus_control` (`ui_control_msg/msg/UiControl`)：按钮控制指令。

控制指令：
- `control_flag=1`：初始化系统。
- `control_flag=2`：规划扫查路径。
- `control_flag=3`：开始自动扫查。
- `control_flag=4`：重建三维图像。
- `control_flag=5`：退出系统。

键盘：
- `Esc`：退出全屏。
- `F1`：进入全屏。

图像转换：
- 直接把 ROS Image 转为 QImage，不依赖 cv_bridge/cv2。
- 支持 rgb8、bgr8、mono8、8uc1、rgba8、bgra8 编码。
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy import logging as rclpy_logging

from sensor_msgs.msg import Image
from ui_control_msg.msg import UiControl

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QGroupBox,
)
from PyQt5.QtGui import QPixmap, QFont, QImage
from PyQt5.QtCore import Qt


CAMERA_COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw"


def camera_color_image_qos() -> QoSProfile:
    """Create the QoS profile used by the depth camera RGB image publisher."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def ros_image_to_qimage(msg: Image) -> QImage:
    """
    Convert sensor_msgs/Image to QImage without cv_bridge/cv2.

    当前系统只需要在 UI 中显示图像，不需要 OpenCV 处理。直接转换可以避开
    NumPy 2.x 与 ROS Humble cv_bridge 的二进制兼容问题，也避免 cv2 自带
    Qt 插件和 PyQt5 的 xcb 插件冲突。
    """
    encoding = msg.encoding.lower()
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    data = bytes(msg.data)

    if width <= 0 or height <= 0:
        raise ValueError('image width and height must be positive')

    if encoding in ('rgb8', 'bgr8'):
        step = step or width * 3
        q_image = QImage(data, width, height, step, QImage.Format_RGB888)
        if encoding == 'bgr8':
            # usb_cam/cv_bridge 常见输出是 BGR；Qt 显示需要 RGB。
            return q_image.rgbSwapped().copy()
        return q_image.copy()

    if encoding in ('mono8', '8uc1'):
        step = step or width
        return QImage(data, width, height, step, QImage.Format_Grayscale8).copy()

    if encoding == 'rgba8':
        step = step or width * 4
        return QImage(data, width, height, step, QImage.Format_RGBA8888).copy()

    if encoding == 'bgra8':
        step = step or width * 4
        q_image = QImage(data, width, height, step, QImage.Format_RGBA8888)
        return q_image.rgbSwapped().copy()

    raise ValueError(f'unsupported image encoding: {msg.encoding}')


class CustomQApplication(QApplication):
    """Custom QApplication that logs exceptions from Qt event handlers."""

    def notify(self, receiver, event):
        try:
            return super().notify(receiver, event)
        except Exception as e:
            rclpy_logging.get_logger("ui_node").error(
                f"Exception in event handler: {e}"
            )
            return False


class FullScreenWidget(QWidget):
    """Custom full-screen widget with keyboard shortcuts."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def keyPressEvent(self, event):
        # 仅保留 Esc/F1 全屏切换
        if event.key() == Qt.Key_Escape:
            if self.isFullScreen():
                self.showNormal()
        elif event.key() == Qt.Key_F1:
            if not self.isFullScreen():
                self.showFullScreen()
        else:
            super().keyPressEvent(event)


class UINode(Node):
    def __init__(self, app: CustomQApplication):
        super().__init__("ui_node")
        self.app = app

        # 仅保留 ROS 自旋线程
        self.ros_spin_thread = None

        # ---------------- UI 构建（提取到单独函数） ----------------
        self._create_ui()

        # ---------------- ROS 通信 ----------------
        self.video_subscriber = self.create_subscription(
            Image, "us_images", self.video_listener_callback, 5
        )
        self.color_video_subscriber = self.create_subscription(
            Image,
            CAMERA_COLOR_IMAGE_TOPIC,
            self.color_video_listener_callback,
            camera_color_image_qos(),
        )
        # 使用 publisher_ui 作为操作区按钮的 publisher
        self.publisher_ui = self.create_publisher(
            UiControl,
            "tus_control",
            10
        )

    def _create_ui(self):
        """Create and lay out the UI widgets."""
        # ---------------- UI 构建 ----------------
        self.window = FullScreenWidget()
        self.window.setWindowTitle("Ultra Scanning System UI")

        # logo
        self.logo_label = None
        logo = QPixmap(
            "/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_system/"
            "ui_test/resource/logo.jpg"
        )
        if not logo.isNull():
            self.logo_label = QLabel(self.window)
            self.logo_label.setPixmap(logo.scaled(250, 200, Qt.KeepAspectRatio))
            self.logo_label.move(10, -25)
            self.logo_label.show()
        else:
            self.get_logger().error("Failed to load logo image.")

        # 标题
        # self.title_label = QLabel("面向钢管缺陷无损检测的遥操作超声实时成像系统", self.window)
        self.title_label = QLabel("面向曲面缺陷无损检测的机器人超声扫查成像系统", self.window)
        font = QFont()
        font.setPointSize(50)
        font.setBold(True)
        font.setItalic(False)
        self.title_label.setFont(font)
        self.title_label.setStyleSheet(
            "QLabel {"
            " background-color: rgb(86, 200, 229);"
            " border: 4px solid rgb(0, 81, 255);"
            " padding: 5px;"
            " border-radius: 10px;"
            "}"
        )
        self.title_label.move(280, 12)

        # ---------------- 操作区 ----------------
        self.settings_group = QGroupBox("操作区", self.window)
        self.settings_group.setStyleSheet(
            "QGroupBox {"
            " background-color: rgb(240, 240, 240);"
            " border: 2px solid rgb(128, 128, 128);"
            " border-radius: 0px;"
            " margin-top: 1ex;"
            "}"
            "QGroupBox::title {"
            " subcontrol-origin: margin;"
            " left: 10px;"
            " padding: 0 3px;"
            " font-size: 30px;"
            " font-weight: bold;"
            " color: rgb(64, 64, 64);"
            "}"
        )
        self.settings_layout = QVBoxLayout(self.settings_group)

        for i in range(5):
            msg = UiControl()
            msg.control_flag = i + 1

            if i == 0:
                button = QPushButton("初始化系统", self.settings_group)
            elif i == 1:
                button = QPushButton("规划扫查路径", self.settings_group)
            elif i == 2:
                button = QPushButton("开始自动扫查", self.settings_group)
            elif i == 3:
                button = QPushButton("重建三维图像", self.settings_group)
            else:  # i == 4
                button = QPushButton("退出系统", self.settings_group)
                # 退出按钮只绑定退出槽
                button.clicked.connect(
                    lambda checked=False, m=msg: self.on_exit_clicked(m)
                )

            button.setStyleSheet(
                "QPushButton {"
                " background-color: rgb(78, 109, 148);"
                " color: white;"
                " border: 1px solid rgb(78, 109, 148);"
                " border-radius: 5px;"
                " padding: 8px 16px;"
                " font-size: 18px;"
                "}"
                "QPushButton:hover {"
                " background-color: rgb(40, 62, 101);"
                "}"
                "QPushButton:pressed {"
                " background-color: rgb(38, 46, 59);"
                "}"
            )
            button.setFixedSize(920, 40)
            # 通用槽函数：仅非退出按钮绑定发布逻辑
            if i != 4:
                button.clicked.connect(
                    lambda checked=False, m=msg: self.on_control_clicked(m)
                )
            self.settings_layout.addWidget(button)

        self.settings_group.setLayout(self.settings_layout)
        setting_x = 940
        setting_y = 155 + 540 + 20
        # 原来为 300，这里适当加宽
        setting_width = 950
        setting_height = 340
        self.settings_group.move(setting_x, setting_y)
        self.settings_group.resize(setting_width, setting_height)
        self.settings_group.show()

        # ---------------- 视频显示区 ----------------
        # 内窥镜视频 QLabel
        # 320 x 500
        self.video_label_in = QLabel(self.window)
        self.video_label_in.move(20, 155)
        self.video_label_in.resize(1000, 900)
        self.video_label_in.show()

        # 工业摄像头视频 QLabel
        self.video_label_out = QLabel(self.window)
        self.video_label_out.move(940, 155)
        self.video_label_out.resize(960, 540)
        self.video_label_out.show()

        # 摄像头画面说明标签
        self.label_font = QFont()
        self.label_font.setPointSize(20)
        self.label_font.setBold(True)

        self.in_camera_label = QLabel("扫查图像", self.window)
        self.in_camera_label.setFont(self.label_font)
        self.in_camera_label.setStyleSheet(
            "QLabel {"
            " background-color: rgba(255, 255, 255, 150);"
            " color: black;"
            " padding: 5px;"
            " border-radius: 5px;"
            "}"
        )
        self.in_camera_label.move(30, 165)
        self.in_camera_label.show()

        self.out_camera_label = QLabel("操作视角", self.window)
        self.out_camera_label.setFont(self.label_font)
        self.out_camera_label.setStyleSheet(
            "QLabel {"
            " background-color: rgba(255, 255, 255, 150);"
            " color: black;"
            " padding: 5px;"
            " border-radius: 5px;"
            "}"
        )
        self.out_camera_label.move(950, 165)
        self.out_camera_label.show()

        # 置顶操作区
        self.settings_group.raise_()

        # 全屏显示
        self.window.showFullScreen()

    # ---------------- 运行与退出 ----------------
    def run(self):
        # 启动 ROS 自旋线程
        self.ros_spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True
        )
        self.ros_spin_thread.start()
        # 启动 Qt 事件循环
        return self.app.exec_()

    def on_exit_clicked(self, msg: UiControl):
        # 退出按钮专有逻辑：发布退出指令并关闭 ROS 和 Qt
        if rclpy.ok():
            self.publisher_ui.publish(msg)
            self.get_logger().info(
                f"UI published control command (exit): {msg.control_flag}"
            )
            rclpy.shutdown()
        if self.ros_spin_thread and self.ros_spin_thread.is_alive():
            # 等待自旋线程结束
            self.ros_spin_thread.join(timeout=1.0)
        self.app.quit()

    def on_control_clicked(self, msg: UiControl):
        # 操作区按钮：通过 publisher_ui 发布控制指令
        if not rclpy.ok():
            self.get_logger().warn(
                f"ROS context not ok, skip publishing control command: {msg.control_flag}"
            )
            return
        self.publisher_ui.publish(msg)
        self.get_logger().info(
            f"UI published control command: {msg.control_flag}"
        )

    # ---------------- ROS 回调 ----------------
    def video_listener_callback(self, msg: Image):
        # image_detected → 颅内画面
        try:
            q_image = ros_image_to_qimage(msg)
        except Exception as e:
            self.get_logger().error(f"image conversion exception: {e}")
            return

        pixmap = QPixmap.fromImage(q_image)
        if self.video_label_in:
            self.video_label_in.setPixmap(
                pixmap.scaled(
                    self.video_label_in.size(), Qt.KeepAspectRatio
                )
            )

    def color_video_listener_callback(self, msg: Image):
        # /camera/camera/color/image_raw → 操作视角画面
        try:
            q_image = ros_image_to_qimage(msg)
        except Exception as e:
            self.get_logger().error(f"image conversion exception: {e}")
            return

        pixmap = QPixmap.fromImage(q_image)
        if self.video_label_out:
            self.video_label_out.setPixmap(
                pixmap.scaled(
                    self.video_label_out.size(), Qt.KeepAspectRatio
                )
            )


def main():
    rclpy.init(args=sys.argv)
    app = CustomQApplication(sys.argv)
    node = UINode(app)

    try:
        result = node.run()
        # 节点内部可能已经调用过 rclpy.shutdown，这里加保护
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(result)
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
