#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np                                     # Python数值计算库
import cv2                                             # Opencv图像处理库

# lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
# upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

lower_red = np.array([0, 60, 100])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

# 红色范围 1：低 H 值部分
lower_red1 = np.array([0, 40, 60])
upper_red1 = np.array([18, 255, 255])
# 红色范围 2：高 H 值部分
lower_red2 = np.array([155, 40, 60])
upper_red2 = np.array([180, 255, 255])

class ImageProcessor:
    def __init__(self):
        self.processed_image = None

    def object_detect(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)             # 图像从BGR颜色模型转换为HSV模型
        # mask_red = cv2.inRange(hsv_img, lower_red, upper_red)        # 图像二值化

         # 生成两个范围的掩码
        mask_red1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        # 合并两个掩码
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)          # 图像中轮廓检测
        whole_area = 0
        for cnt in contours:                                         # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 100:
                continue

            contour_area = cv2.contourArea(cnt)                      # 计算轮廓面积
            # 当面积小于1000时，用黄色绘制轮廓
            if contour_area < 2000:
                cv2.drawContours(image, [cnt], -1, (0, 255, 255), 2)     # 黄色绘制轮廓
            else:
                cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)       # 绿色绘制轮廓
            whole_area += contour_area

        self.processed_image = image                                 # 保存处理后的图像

def main():
    # 读取图像文件，需替换为实际图像路径
    image_path = '/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_system/camera_service/camera_service/raw_image.png'
    image = cv2.imread(image_path)

    if image is None:
        print(f"Failed to read image from {image_path}")
        return

    processor = ImageProcessor()
    processor.object_detect(image)

    # 保存处理后的图像
    output_path = 'processed_image.png'
    cv2.imwrite(output_path, processor.processed_image)
    print(f"Processed image saved to {output_path}")

if __name__ == '__main__':
    main()