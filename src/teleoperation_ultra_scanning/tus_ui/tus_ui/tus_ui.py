import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy import logging as rclpy_logging

from sensor_msgs.msg import Image
from ui_control_msg.msg import UiControl

from cv_bridge import CvBridge
import cv2

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
        self.video_subscriber = self.create_subscription(
            Image, "us_images", self.video_listener_callback, 5
        )
        self.usb_video_subscriber = self.create_subscription(
            Image, "usb_images", self.usb_video_listener_callback, 5
        )
        # 使用 publisher_ui 作为操作区按钮的 publisher
        self.publisher_ui = self.create_publisher(
            UiControl,
            "tus_control",
            10
        )

    def _create_ui(self):
        """构建和布局整个 UI（从原 __init__ 中抽取）"""

        # ---------------- UI 构建 ----------------
        self.window = FullScreenWidget()
        self.window.setWindowTitle("Teleopration UI")

        # logo
        self.logo_label = None
        logo = QPixmap(
            "/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_system/ui_test/resource/logo.jpg"
        )
        if not logo.isNull():
            self.logo_label = QLabel(self.window)
            self.logo_label.setPixmap(logo.scaled(250, 200, Qt.KeepAspectRatio))
            self.logo_label.move(10, -25)
            self.logo_label.show()
        else:
            self.get_logger().error("Failed to load logo image.")

        # 标题
        self.title_label = QLabel("面向钢管缺陷无损检测的遥操作超声实时成像系统", self.window)
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
                button = QPushButton("机器人初始化", self.settings_group)
            elif i == 1:
                button = QPushButton("机器人作业准备", self.settings_group)
            elif i == 2:
                button = QPushButton("开启系统遥操作", self.settings_group)
            elif i == 3:
                button = QPushButton("关闭系统遥操作", self.settings_group)
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
        # self.video_label_in.resize(1600, 900)
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
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge exception: {e}")
            return

        # BGR → RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        q_image = QImage(
            rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888
        )
        pixmap = QPixmap.fromImage(q_image)
        if self.video_label_in:
            self.video_label_in.setPixmap(
                pixmap.scaled(
                    self.video_label_in.size(), Qt.KeepAspectRatio
                )
            )

    def usb_video_listener_callback(self, msg: Image):
        # camera2/image_raw → 体外画面
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
