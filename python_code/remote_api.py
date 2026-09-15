import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import requests
import io
from PIL import Image as PILImage

API_URL = "https://u264769-b554-2d26bdc0.westc.gpuhub.com:8443/predict" 

class VLARosClient(Node):
    def __init__(self):
        super().__init__('vla_ros_client')
        # 1. 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10) # 队列深度
        
        # 2. 远程服务器配置
        self.api_url = API_URL
        self.instruction = "pick up the cup and place it on the table"
        
        self.get_logger().info('VLA ROS Client 已启动，等待图像...')

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 Numpy 数组
        # 你的数据是 rgb8 格式，直接 reshape 即可
        img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        
        # 预处理：OpenVLA 只需要 224x224
        img_resized = cv2.resize(img_np, (224, 224))
        
        # 发送请求
        self.call_remote_vla(img_resized)

    def call_remote_vla(self, img_np):
        _, img_encoded = cv2.imencode('.jpg', img_np)
        files = {'file': ('image.jpg', img_encoded.tobytes(), 'image/jpeg')}
        data = {'instruction': self.instruction, 'unnorm_key': 'taco_play'}
        
        try:
            response = requests.post(self.api_url, files=files, data=data, timeout=5)
            if response.status_code == 200:
                action = response.json()['action']
                self.get_logger().info(f'预测动作: {action}')
            else:
                self.get_logger().error(f'服务器错误: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'请求失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    vla_client = VLARosClient()
    rclpy.spin(vla_client)
    vla_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()