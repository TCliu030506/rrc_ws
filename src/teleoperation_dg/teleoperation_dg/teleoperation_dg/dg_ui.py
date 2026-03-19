import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy import logging as rclpy_logging

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QVBoxLayout,
    QGroupBox,
    QDesktopWidget,
)
from PyQt5.QtGui import QPixmap, QFont, QImage
from PyQt5.QtCore import Qt


class CustomQApplication(QApplication):
    """自定义 QApplication，捕获事件处理中的异常并打印 ROS 日志"""

    def notify(self, receiver, event):
        try:
            return super().notify(receiver, event)
        except Exception as e:
            rclpy_logging.get_logger("ui_node").error(
                f"Exception in event handler: {e}"
            )
            return False


class FullScreenWidget(QWidget):
    """自定义窗口，重写 keyPressEvent，与 C++ 逻辑一致"""

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

        # OpenCV / cv_bridge
        self.bridge = CvBridge()

        # ---------------- UI 构建（提取到单独函数） ----------------
        self._create_ui()

        # ---------------- ROS 通信 ----------------
        self.usb_video_subscriber = self.create_subscription(
            Image, "usb_images", self.usb_video_listener_callback, 5
        )

    def _create_ui(self):
        """构建和布局整个 UI（从原 __init__ 中抽取）"""

        # ---------------- UI 构建 ----------------
        self.window = FullScreenWidget()
        self.window.setWindowTitle("Teleopration UI")

        # logo
        self.logo_label = None
        logo = QPixmap(
            "/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_dg/teleoperation_dg/resource/logo.png"
        )
        if not logo.isNull():
            self.logo_label = QLabel(self.window)
            self.logo_label.setPixmap(logo.scaled(250, 200, Qt.KeepAspectRatio))
            self.logo_label.move(10, -40)
            self.logo_label.show()
        else:
            self.get_logger().error("Failed to load logo image.")

        # 标题
        self.title_label = QLabel("Teleoperation System of Gripper based on MD", self.window)
        font = QFont("Arial")  # 推荐使用 Arial，简洁且跨平台
        font.setPointSize(50)
        font.setBold(True)
        font.setItalic(False)
        self.title_label.setFont(font)
        self.title_label.setStyleSheet(
            "QLabel {"
            " padding: 5px;"
            " border-radius: 10px;"
            "}"
        )
        # 设置标题宽度为窗口宽度并水平居中
        screen = QDesktopWidget().screenGeometry()
        self.title_label.setFixedWidth(screen.width())
        self.title_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.title_label.move(0, 12)

        # ---------------- 视频显示区 ----------------
        # 工业摄像头视频 QLabel
        screen = QDesktopWidget().screenGeometry()
        video_width = int(screen.width() * 1.0)
        video_height = int(screen.height() * 1.0)
        video_x = (screen.width() - video_width) // 2
        video_y = int((screen.height() - video_height) * 1.2)

        self.video_label_out = QLabel(self.window)
        self.video_label_out.move(video_x, video_y)
        self.video_label_out.resize(video_width, video_height)
        self.video_label_out.lower()  # 置于最底层
        self.video_label_out.show()

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

    # ---------------- ROS 回调 ----------------
    def usb_video_listener_callback(self, msg: Image):
        # 摄像头图像显示回调
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge exception: {e}")
            return

        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        q_image = QImage(
            rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888
        )
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
