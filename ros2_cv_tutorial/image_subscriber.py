#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# image_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 左上に赤い円を描画
            cv2.circle(cv_image, (50, 50), 20, (0, 0, 255), -1)
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().info(f"Could not convert from '{msg.encoding}' to 'bgr8'.")


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
