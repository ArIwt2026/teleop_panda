#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from franka_teach_msgs.action import GoToCameraPose
from geometry_msgs.msg import PoseStamped
import sys

class TestCameraPoseClient(Node):
    def __init__(self):
        super().__init__('test_camera_pose_client')
        self._action_client = ActionClient(self, GoToCameraPose, '/franka_teach_executor/go_to_camera_pose')

    def send_goal(self, z_offset=0.1, plan_only=False):
        goal_msg = GoToCameraPose.Goal()
        
        target = PoseStamped()
        target.header.frame_id = 'camera_front_link'
        target.header.stamp = self.get_clock().now().to_msg()
        
        # Position: z_offset meters in front of the camera
        target.pose.position.x = 0.0
        target.pose.position.y = 0.0
        target.pose.position.z = z_offset
        
        # Orientation: Identity (pointing same way as camera)
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0
        
        goal_msg.target_pose = target
        goal_msg.velocity_scale = 0.1
        goal_msg.acceleration_scale = 0.1
        goal_msg.planning_time = 5.0
        goal_msg.goal_tolerance = 0.01
        goal_msg.plan_only = plan_only

        self.get_logger().info('Waiting for action server...')

        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: move to z={z_offset} in camera frame')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Result: SUCCESS')
        else:
            self.get_logger().error(f'Result: FAILED - {result.error_message}')
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    z_offset = 0.1
    plan_only = '--plan' in sys.argv
    for arg in sys.argv[1:]:
        if arg != '--plan':
            try:
                z_offset = float(arg)
                break
            except ValueError:
                pass

    client = TestCameraPoseClient()
    client.send_goal(z_offset, plan_only)

    rclpy.spin(client)

if __name__ == '__main__':
    main()
