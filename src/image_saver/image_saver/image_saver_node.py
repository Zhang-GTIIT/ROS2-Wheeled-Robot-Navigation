#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 可根据需要修改话题名
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

        # 设置保存路径
        self.save_dir = os.path.join(os.getcwd(), "saved_images")
        os.makedirs(self.save_dir, exist_ok=True)

        # 控制保存频率
        self.last_save_time = 0.0
        self.save_interval = 0.1  # 每秒保存一次

        self.get_logger().info("Image Saver Node started, listening to /camera/image_raw")

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_save_time >= self.save_interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                timestamp = self.get_clock().now().to_msg()
                filename = f"{timestamp.sec}_{timestamp.nanosec}.png"
                save_path = os.path.join(self.save_dir, filename)
                cv2.imwrite(save_path, cv_image)
                self.get_logger().info(f"Saved image: {save_path}")
                self.last_save_time = current_time
            except Exception as e:
                self.get_logger().error(f"Error saving image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down image saver node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

