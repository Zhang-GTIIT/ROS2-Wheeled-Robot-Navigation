#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import math
import json
import numpy as np
import time

class Nav2GoalSender(Node):

    def __init__(self):
        super().__init__('nav2_goal_sender')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._goal_handle = None
        self._last_feedback_time = 0.0

    def send_goals(self, waypoints, yaw_strategy='auto'):
        """
        发送多个路径点进行导航
        
        Args:
            waypoints: [(x, y), ...] 列表
            yaw_strategy: 'auto' - 根据路径方向自动计算朝向
                          'fixed' - 所有点使用固定朝向（如 0.0）
        """
        # 等待 Action Server 就绪
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        # 构造所有路径点
        poses = []
        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            
            # 设置朝向：自动计算路径方向 或 固定值
            if yaw_strategy == 'auto' and len(waypoints) > 1:
                if i < len(waypoints) - 1:
                    # 指向下一个点
                    next_x, next_y = waypoints[i + 1]
                    yaw = math.atan2(next_y - y, next_x - x)
                else:
                    # 最后一个点：保持与前一个点相同的方向
                    prev_x, prev_y = waypoints[i - 1]
                    yaw = math.atan2(y - prev_y, x - prev_x)
            else:
                yaw = 0.0  # 固定朝向（可根据需求修改）
            
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
            
            self.get_logger().info(f'Waypoint {i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°)')
        self._total_poses = len(poses)
        # 构造目标
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.get_logger().info(f'Sending {len(poses)} waypoints via navigate_through_poses...')
        
        # 发送目标
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted. Executing navigation through poses...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation through poses succeeded!')
        elif status == 5:  # ABORTED
            self.get_logger().error('Navigation through poses aborted!')
        elif status == 6:  # CANCELED
            self.get_logger().warn('Navigation through poses canceled!')
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_time = time.time()
        
        # 降频：每秒最多输出1次日志
        if current_time - self._last_feedback_time < 1.0:
            return
        
        self._last_feedback_time = current_time
        # 兼容 Humble/Galactic: 使用 number_of_poses_remaining 推算当前索引
        if hasattr(feedback, 'number_of_poses_remaining'):
            remaining = feedback.number_of_poses_remaining
            current_index = self._total_poses - remaining  if self._total_poses > 0 else -1
            
            # 获取当前目标位姿（用于显示位置）
            current_x = feedback.current_pose.pose.position.x if hasattr(feedback, 'current_pose') else 0.0
            current_y = feedback.current_pose.pose.position.y if hasattr(feedback, 'current_pose') else 0.0
            
            self.get_logger().info(
                f'📍 Progress: waypoint {current_index}/{self._total_poses} | '
                f'Pos: ({current_x:.2f}, {current_y:.2f}) | '
                f'Remaining Distance: {feedback.distance_remaining:.2f}m'
            )
        else:
            # 最基础的反馈（兼容所有版本）
            self.get_logger().debug(
                f'Feedback: time={feedback.navigation_time.sec}s, '
                f'dist_rem={feedback.distance_remaining:.2f}m, '
                f'recoveries={feedback.number_of_recoveries}'
            )

    def cancel_goal(self):
        """取消当前导航目标"""
        if self._goal_handle and self._goal_handle.accepted:
            self.get_logger().info('Canceling navigation goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info('Goal cancellation requested.')

def pos_from_unity(wp_path: str = 'waypoints.json', tf_path: str = 'tf_unity_to_real.json'):
    # 定义坐标转换函数：Unity Y-up → ROS Z-up (XZ平面)
    def z2y(pos):
        x, y, z = pos
        return (x, z)
    
    # 读取json中的unity点位置信息
    with open(wp_path, 'r') as f:
        waypoints_data = json.load(f)
    waypoints_unity = waypoints_data['path']['waypoints']

    # 读取tf矩阵
    with open(tf_path, 'r') as f:
        tf_data = json.load(f)
    T_unity_to_real = np.array(tf_data['T_unity_to_real'])

    waypoints_real = []
    for wp in waypoints_unity:
        wpp = wp['position']
        pos = z2y((wpp['x'], wpp['y'], wpp['z']))
        pos_h = np.array([pos[0], pos[1], 0, 1])
        pos_real_h = T_unity_to_real @ pos_h
        waypoints_real.append(tuple(pos_real_h[:2]))  # 转为 tuple 便于后续处理
    return waypoints_real

def main(args=None):
    rclpy.init(args=args)

    node = Nav2GoalSender()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # 获取转换后的路径点
        waypoints = pos_from_unity()
        if not waypoints:
            node.get_logger().error('No waypoints loaded!')
            return

        node.get_logger().info(f'Loaded {len(waypoints)} waypoints from Unity')

        # 发送所有路径点（一次性）
        # yaw_strategy: 'auto' 根据路径自动计算朝向，'fixed' 使用固定朝向
        success = node.send_goals(waypoints, yaw_strategy='auto')
        
        if not success:
            return

        
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            

    except KeyboardInterrupt:
        node.get_logger().info('Navigation interrupted by user. Canceling goal...')
        node.cancel_goal()
        # 等待取消完成
        time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()