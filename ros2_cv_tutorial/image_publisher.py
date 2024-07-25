#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.realsense_camera_id = 4
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(self.realsense_camera_id)  # カメラからキャプチャする場合

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(image_message)


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
