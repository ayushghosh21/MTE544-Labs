#!/usr/bin/env python3

import rclpy

import time

from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Duration
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Path
from mte544_a_star.a_star_skeleton1 import find_path
import numpy as np
from mte544_action_interfaces.action import Move2Goal
from skimage.transform import downscale_local_mean

class AStarActionServer(Node):

    def __init__(self):
        super().__init__('a_star_action_server')
        self._action_server = ActionServer(
            self,
            Move2Goal,
            'mte_544_a_star',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
        )
        
        # create the subscriber object
        self.global_map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Create path visualizer publisher 
        self.path_pub = self.create_publisher(Path, '/path_viz', 10)

        # Initialize map parameters to default values 
        self.origin = [0,0,0]
        self.map_res = 0.03

        # global costmap variable
        self.occupancy_map = None

        # List of cartisian points containing A* path to goal
        self.path_cart = None

    def goal_callback(self, goal_request):
        """Determine whether to accept or reject the goal"""

        # Access global map and pass to A* function
        while True:
            if self.occupancy_map is None:
                pass
            else:
                break
        initial_pose = goal_request.initial_pose

        start = self.coord2pixel((initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y))
        goal =  self.coord2pixel((goal_request.goal_x, goal_request.goal_y))

        # start = (250, 150)
        # goal =  (50, 150)
        # Run A*
        [path, cost] = find_path(start, goal, self.occupancy_map)
        
        if path.size == 0:
            # Failed to find a path to the goal
            self.get_logger().info(f"No path found")
            return GoalResponse.REJECT

        path_scale = path*self.map_res
        self.path_cart = path_scale + self.origin

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for pose in self.path_cart:
            point = PoseStamped()

            point.pose.position.x = pose[0]
            point.pose.position.y = pose[1]

            path_msg.poses.append(point)

        self.path_pub.publish(path_msg)
        dist = cost*self.map_res
        self.get_logger().info(f"Path distance:{dist}")
        self.get_logger().info(f"Number of points: {self.path_cart.shape[0]}")
        #plt.plot(self.path_cart[:, 0], self.path_cart[:, 1])
        #plt.show()
        return GoalResponse.ACCEPT

    def map_callback(self, msg):
        
        if self.occupancy_map is None:

            self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
            
            self.height = msg.info.height
            self.width = msg.info.width
            self.map_res = msg.info.resolution
            self.occupancy_map = np.reshape(msg.data, (self.height, self.width))
            self.occupancy_map = downscale_local_mean(self.occupancy_map, (2,2))
            self.map_res *= 2
            self.occupancy_map[self.occupancy_map < 65] = 0
            self.occupancy_map = np.transpose(self.occupancy_map).astype(bool)

            #maze_plot=np.transpose(np.nonzero(self.occupancy_map))
            #plt.plot(maze_plot[:,0], maze_plot[:,1], '.')
            #plt.show()


            ## KEEP IT FOR TESTING!!!

            # start = (177, 117)
            # goal =  (50, 150)
            # goal = [0, -0.08]
            
            # goal[0] -= self.origin[0]
            # goal[1] -= self.origin[1]

            # goal[0] /= self.map_res
            # goal[1] /= self.map_res
            
            # goal[0] = round(goal[0])
            # goal[1] = round(goal[1])

            # #print(goal)
            # [path, cost] = find_path(start, (goal[0], goal[1]), self.occupancy_map)

            # path_scale = path*self.map_res
            # path_cart = path_scale + self.origin

            # dist = cost*self.map_res
            # print(dist)
            # #print(path_cart.shape)
            # #plt.plot(path_cart[:, 0], path_cart[:, 1])
            # plt.show()

    def execute_callback(self, goal_handle):
        """Follow A* planned path using a P controller"""

        self.get_logger().info('Executing goal...') 
        
        # Use the points in path_cart variable for P controller. path_cart is list of cartestion points in plan

        # P controller pseduocode
        # P Controller should subscribe to the tf topic to determine current pose
        # calculate error between current pose and intermediate goal pose
        # generate control signals (velocities) required to reach intermediate goal pose
        # publish commanded velocities to drive the robot from its current position to the intermediate goal pose via /cmd_vel topic

        for point in self.path_cart:
            break # For now,
            self.get_logger().info(f"Going to: {point}")
            reached_goal = False
            while not reached_goal:
                #TODO: add P controller
                #TODO: run 1 iteration of P control

                feedback_msg = Move2Goal.Feedback()
                feedback_msg.current_pose.pose.position.x
                feedback_msg.current_pose.pose.position.y
                goal_handle.publish_feedback(feedback_msg)
                
                rclpy.spin_once(self, timeout_sec=0.1) # 10Hz spin
        
        #Have now reached the final goal
        self.get_logger().info('Reached goal')
        goal_handle.succeed()
        result = Move2Goal.Result()
        result.reached_goal = True

        # Else goal is unreachable for some reason while taversing
        # goal_handle.abort()
        # result = Move2Goal.Result()
        # result.reached_goal = False

        return result

    def coord2pixel(self, point):
        return (
            round((point[0] - self.origin[0])/(self.map_res)), 
            round((point[1] - self.origin[1])/(self.map_res))
        ) 

def main(args=None):
    rclpy.init(args=args)
    a_star_action_server = AStarActionServer()
    rclpy.spin(a_star_action_server)

if __name__ == '__main__':
    main()