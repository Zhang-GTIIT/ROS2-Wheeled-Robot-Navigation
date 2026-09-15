#!/usr/bin/env python3
import socket
import os
import json
import time
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# ================= 配置区域 =================
HOST = '0.0.0.0'
PORT = 8000  # 监听端口
SAVE_DIR = "/home/agv/wzb/python_code/nav2_task"

if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

class Nav2Executor(Node):
    """
    负责执行 Nav2 导航任务的 ROS2 节点
    """
    def __init__(self):
        super().__init__('nav2_socket_executor')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def yaw_to_quaternion(self, yaw):
        """将欧拉角 yaw 转换为四元数 (z, w)"""
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def execute_navigation_blocking(self, x, y, yaw):
        """
        发送导航目标并阻塞等待结果
        返回: (success: bool, message: str)
        """
        # 1. 等待 Action Server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return False, "Nav2 Action Server not available"

        # 2. 构建目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        qz, qw = self.yaw_to_quaternion(float(yaw))
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Navigate Request: x={x}, y={y}, yaw={yaw}")

        # 3. 发送目标 (异步发送，但我们要同步等待)
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # 阻塞等待 Goal 被接受
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            return False, "Goal was rejected by Nav2"

        self.get_logger().info("Goal accepted. Moving...")

        # 4. 等待执行结果
        get_result_future = goal_handle.get_result_async()
        
        # 阻塞等待导航完成 (这就是实现 Socket 阻塞的关键)
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result()
        status = result.status

        # status 4 means SUCCEEDED in action_msgs/msg/GoalStatus
        if status == 4:
            return True, "Navigation Succeeded"
        else:
            return False, f"Navigation Failed/Canceled with status code: {status}"

def main():
    # 1. 初始化 ROS2
    rclpy.init()
    nav_node = Nav2Executor()
    
    # 2. 初始化 Socket Server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)
    
    nav_node.get_logger().info(f"🚀 Nav2 Socket Server listening on port {PORT}...")

    try:
        while True:
            # 2.1 等待连接
            conn, addr = server.accept()
            nav_node.get_logger().info(f"Connected by {addr}")
            
            try:
                data = conn.recv(4096)
                if not data:
                    conn.close()
                    continue
                
                # 2.2 解析与保存 JSON
                json_str = data.decode('utf-8')
                
                # 保存备份
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = f"{SAVE_DIR}/task_{timestamp}.json"
                with open(save_path, "w") as f:
                    f.write(json_str)
                
                task_data = json.loads(json_str)
                action = task_data.get("action")
                coords = task_data.get("coords", {})

                response_msg = ""
                success = False

                # 2.3 执行逻辑
                if action == "navigate":
                    x = coords.get("x")
                    y = coords.get("y")
                    yaw = coords.get("yaw")

                    if x is not None and y is not None and yaw is not None:
                        # === 调用核心阻塞导航函数 ===
                        success, response_msg = nav_node.execute_navigation_blocking(x, y, yaw)
                    else:
                        response_msg = "Error: Missing x, y, or yaw in coords"
                        nav_node.get_logger().error(response_msg)
                else:
                    response_msg = f"Unknown action: {action}"
                    nav_node.get_logger().warn(response_msg)

                # 2.4 发送回执
                final_response = {
                    "status": "success" if success else "error",
                    "message": response_msg,
                    "location": task_data.get("location_name", "unknown")
                }
                conn.sendall(json.dumps(final_response).encode('utf-8'))
                nav_node.get_logger().info(f"Task Finished. Response sent: {response_msg}")

            except json.JSONDecodeError:
                err = "Invalid JSON format"
                conn.sendall(json.dumps({"status": "error", "message": err}).encode('utf-8'))
                nav_node.get_logger().error(err)
            except Exception as e:
                err = f"Internal Error: {str(e)}"
                conn.sendall(json.dumps({"status": "error", "message": err}).encode('utf-8'))
                nav_node.get_logger().error(err)
            finally:
                conn.close()

    except KeyboardInterrupt:
        nav_node.get_logger().info("Shutting down server...")
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()
        server.close()

if __name__ == "__main__":
    main()