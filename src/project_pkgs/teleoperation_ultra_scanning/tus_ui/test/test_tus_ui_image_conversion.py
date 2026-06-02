from sensor_msgs.msg import Image


def test_rgb8_image_message_converts_to_qimage_without_cv_bridge():
    from tus_ui.tus_ui import ros_image_to_qimage

    msg = Image()
    msg.height = 1
    msg.width = 2
    msg.encoding = 'rgb8'
    msg.step = 6
    msg.data = bytes([
        255, 0, 0,
        0, 255, 0,
    ])

    q_image = ros_image_to_qimage(msg)

    assert q_image.width() == 2
    assert q_image.height() == 1
    assert q_image.pixelColor(0, 0).red() == 255
    assert q_image.pixelColor(1, 0).green() == 255


def test_bgr8_image_message_swaps_channels_for_qimage():
    from tus_ui.tus_ui import ros_image_to_qimage

    msg = Image()
    msg.height = 1
    msg.width = 1
    msg.encoding = 'bgr8'
    msg.step = 3
    msg.data = bytes([255, 0, 0])

    q_image = ros_image_to_qimage(msg)

    assert q_image.pixelColor(0, 0).blue() == 255
