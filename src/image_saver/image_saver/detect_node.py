#!/home/agv/anaconda3/envs/yolo/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import time

from ultralytics import YOLO  # pip install ultralytics


class YoloClassifier(Node):
    def __init__(self):
        super().__init__('yolo_classifier')

        # 订阅摄像头
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            qos_profile=10
        )

        self.bridge = CvBridge()

        # 加载 YOLO 模型
        model_path = "/home/agv/wzb/best.pt"
        self.model = YOLO(model_path)
        self.class_names = self.model.names

        # 发布结果
        self.result_pub = self.create_publisher(String, '/yolo/result', 10)

        # 推理频率控制
        self.last_detect_time = 0.0
        self.detect_interval = 0.1

        # ⭐新增：记录最近 5 次分类结果
        self.history = []
        self.history_size = 3
        self.conf_threshold = 0.75

        self.get_logger().info("YOLO11 Classification Node Started.")

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_detect_time < self.detect_interval:
            return

        self.last_detect_time = current_time

        try:
            # Convert ROS Img → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # YOLO 推理（分类）
            results = self.model(cv_image)

            # YOLO 分类结果
            pred = results[0].probs
            class_id = int(pred.top1)
            class_name = self.class_names[class_id]
            confidence = float(pred.top1conf)

            # ⭐把结果放入历史记录队列
            self.history.append((class_name, confidence))
            if len(self.history) > self.history_size:
                self.history.pop(0)

            # 打印即时结果
            # self.get_logger().info(f"YOLO classification: {class_name} ({confidence:.2f})")

            # ⭐如果历史记录还不够 5 条，不发布
            if len(self.history) < self.history_size:
                return

            # ⭐检查最近 5 次是否都为同一类别 & 置信度均高于阈值
            classes = [c for c, _ in self.history]
            confs = [conf for _, conf in self.history]

            if len(set(classes)) == 1 and all(conf > self.conf_threshold for conf in confs):
                # 发布最终确认的类别
                self.result_pub.publish(String(data=class_name))
                # self.get_logger().info(f"***** Stable result published: {class_name} *****")

        except Exception as e:
            self.get_logger().error(f"YOLO detect error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloClassifier()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down YOLO classifier.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
