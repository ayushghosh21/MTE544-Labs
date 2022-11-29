#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

from mte544_action_interfaces.action import Move2Goal

class AStarClient(Node):

    def __init__(self):
        super().__init__('a_star_action_client')
        self._action_client = ActionClient(self, Move2Goal, 'mte_544_a_star')
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.initial_pose = None
        self.goal_pose = None

    def initial_pose_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg
            self.get_logger().info(f"Initialize position: {self.initial_pose.pose.pose.position.x, self.initial_pose.pose.pose.position.y}")
            #print(self.initial_pose)

    def goal_pose_callback(self, msg):
        if self.goal_pose is None:
            self.goal_pose = msg
            self.get_logger().info(f"Goal position: {self.goal_pose.pose.position.x, self.goal_pose.pose.position.y}")
            
            #print(self.goal_pose)


    def send_goal(self, initial_x=None, initial_y=None, goal_x=None, goal_y=None):
        goal_msg = Move2Goal.Goal()

        if not (initial_x and initial_y and goal_x and goal_y):
            while True:
                if self.initial_pose is None:
                    rclpy.spin_once(self)
                else:
                    break
            while True:
                if self.goal_pose is None:
                    rclpy.spin_once(self)
                else:
                    break
            
            goal_msg.initial_pose = self.initial_pose
            goal_msg.goal_x = self.goal_pose.pose.position.x
            goal_msg.goal_y = self.goal_pose.pose.position.y

        else:
            goal_msg.initial_pose = PoseWithCovarianceStamped()
            goal_msg.initial_pose.pose.pose.position.x = initial_x
            goal_msg.initial_pose.pose.pose.position.y = initial_y
            goal_msg.goal_x = goal_x
            goal_msg.goal_y = goal_y
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_pose = None
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal Reached: {0}'.format(result.reached_goal))
        self.goal_pose = None
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = AStarClient()

    initial_x = 0
    initial_y = 0
    goal_x = 10 
    goal_y = 10
    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
