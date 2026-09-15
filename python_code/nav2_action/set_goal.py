#!/usr/bin/env python3
import rclpy
import json
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class SimpleNav(Node):
    def __init__(self):
        super().__init__('simple_nav_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def yaw_to_quaternion(self, yaw):
        """
        将 yaw 角 (弧度) 转换为 四元数 (x, y, z, w)
        公式: 
        z = sin(yaw/2)
        w = cos(yaw/2)
        """
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def send_goal_from_json(self, json_path):
        self.get_logger().info('Waiting for Nav2 action server...')
        self._client.wait_for_server()

        # 1. 读取 JSON
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
                coords = data.get('coords', {})
        except Exception as e:
            self.get_logger().error(f'Failed to read JSON: {e}')
            return

        # 2. 提取数据
        target_x = coords.get('x', 0.0)
        target_y = coords.get('y', 0.0)
        target_yaw = coords.get('yaw', 0.0)

        # 3. 转换角度 (Yaw -> Quaternion)
        _, _, qz, qw = self.yaw_to_quaternion(target_yaw)

        # 4. 构建消息
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Navigate to: {data.get('location_name')}")
        self.get_logger().info(f"Coords: x={target_x}, y={target_y}, yaw={target_yaw}")

        # 5. 发送目标
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return

        self.get_logger().info('Goal accepted, moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info('✅ Navigation Succeeded!')
        else:
            self.get_logger().info(f'❌ Navigation Ended with status: {status}')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SimpleNav()
    
    # 替换为你的实际文件路径
    json_file = '/home/agv/wzb/python_code/nav2_task/my_task.json'
    
    node.send_goal_from_json(json_file)
    rclpy.spin(node)

if __name__ == '__main__':
    main()