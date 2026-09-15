#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory

from elevator_manager.zone_utils import in_waiting_zone


class ElevatorManager(Node):

    def __init__(self):
        super().__init__('elevator_manager')

        # ===== Load config =====
        pkg_share = get_package_share_directory('elevator_manager')
        cfg_path = os.path.join(pkg_share, 'config', 'elevator_zones.yaml')

        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        wz = cfg['waiting_zone']
        self.waiting_zone = {
            'cx': wz['center'][0],
            'cy': wz['center'][1],
            'yaw': wz['yaw'],
            'length': wz['length'],
            'width': wz['width'],
            'yaw_tol': wz['yaw_tolerance']
        }

        # ===== TF =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===== Nav2 client =====
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # ===== Elevator Action Server =====
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'elevator_navigate',
            self.execute_callback
        )

        # ===== FSM =====
        self.state = 'IDLE'
        self.current_pose = None
        self.goal_handle = None

        # ===== Timer =====
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info("ElevatorManager READY")

    # --------------------------------------------------
    # Action callback
    # --------------------------------------------------
    async def execute_callback(self, goal_handle):
        self.get_logger().info("Received elevator_navigate goal")
        self.goal_handle = goal_handle
        self.state = 'NAV_TO_WAITING_ZONE'

        # send nav2 goal to waiting zone center
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.pose.position.x = self.waiting_zone['cx']
        nav_goal.pose.pose.position.y = self.waiting_zone['cy']
        nav_goal.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(nav_goal)

        # wait FSM to finish
        while rclpy.ok():
            if self.state == 'DONE':
                goal_handle.succeed()
                self.get_logger().info("Elevator navigate finished")
                return NavigateToPose.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.state = 'IDLE'
                return NavigateToPose.Result()

            await rclpy.sleep(0.1)

    # --------------------------------------------------
    # Timer FSM
    # --------------------------------------------------
    def timer_cb(self):
        # --- update pose ---
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation

            yaw = euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )[2]

            self.current_pose = (x, y, yaw)
        except Exception:
            return

        # --- FSM ---
        if self.state == 'NAV_TO_WAITING_ZONE':
            if in_waiting_zone(self.current_pose, self.waiting_zone):
                self.get_logger().info("Entered waiting zone")
                self.nav_client.cancel_all_goals_async()
                self.state = 'WAIT_FOR_ELEVATOR'

        elif self.state == 'WAIT_FOR_ELEVATOR':
            # 这里后续接 YOLO 门开判断
            self.get_logger().info("Waiting for elevator door (mock)")
            self.state = 'DONE'

        elif self.state == 'DONE':
            pass


def main():
    rclpy.init()
    node = ElevatorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()