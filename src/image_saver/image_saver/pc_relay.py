# pc_relay.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudRelay(Node):
    def __init__(self):
        super().__init__('pointcloud_relay')
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.cb,
            10)
        self.pub_local = self.create_publisher(
            PointCloud2,
            '/camera/depth/points_local',
            10)
        self.pub_global = self.create_publisher(
            PointCloud2,
            '/camera/depth/points_global',
            10)

    def cb(self, msg):
        self.pub_local.publish(msg)
        self.pub_global.publish(msg)

def main():
    rclpy.init()
    node = PointCloudRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()