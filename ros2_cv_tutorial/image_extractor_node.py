#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import cv2
import numpy as np
import os


class RGBDepthImageSaver(Node):
    def __init__(self):
        super().__init__('rgb_depth_image_saver')

        # Create subscribers for RGB and Depth images
        self.rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')  # RGB画像トピック
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')  # Depth画像トピック

        # Synchronize the two topics
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # cv_bridge to convert ROS Image message to OpenCV format
        self.bridge = CvBridge()

        # Save directories for RGB and Depth images
        self.rgb_dir = 'images'
        self.depth_dir = 'depths'

        # Create directories if they don't exist
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

    def callback(self, rgb_msg, depth_msg):
        try:
            # Convert the ROS Image message to OpenCV format for RGB
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            # Convert the ROS Image message to OpenCV format for Depth
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            depth_image = depth_image.astype(np.uint16)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Depth画像の値を255にスケーリング（0〜255の範囲に）
        # depth_image = np.array(depth_image, dtype=np.float32)
        # print(np.max(depth_image))
        # depth_image = depth_image / 1000
        # print(np.max(depth_image))
        # depth_image = depth_image * 1.0
        cv2.normalize(depth_image, depth_image, 0, 255, cv2.NORM_MINMAX)
        # depth_image = np.uint8(depth_image)  # 8ビット画像に変換
        cv2.imshow("depth_image", depth_image)
        cv2.waitKey(1)

        # RGB画像のタイムスタンプを取得し、ファイル名に使用する
        timestamp = rgb_msg.header.stamp
        timestamp_str = f"{timestamp.sec}_{timestamp.nanosec}"

        # Save the RGB and Depth images as PNG files with the timestamp as the filename
        rgb_filename = os.path.join(self.rgb_dir, f"{timestamp_str}.png")
        depth_filename = os.path.join(self.depth_dir, f"{timestamp_str}.png")

        cv2.imwrite(rgb_filename, rgb_image)
        cv2.imwrite(depth_filename, depth_image)

        self.get_logger().info(f"Saved RGB image: {rgb_filename} and Depth image: {depth_filename}")


def main(args=None):
    rclpy.init(args=args)
    node = RGBDepthImageSaver()
    rclpy.spin(node)

    # ノード終了時のクリーンアップ
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
