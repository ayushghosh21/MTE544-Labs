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

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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

        # P-controller items
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # The P-controller runs in the global frame. 
        self.curr_pose = Pose() # holds current position of turtlebot.
        self.setpoint_pose = Pose() # defaults to 0 till we receive a new setpoint from external node
        self.vel_msg = Twist() # holds velocity command to send to turtlebot
        # Used for finding TF between base_footprint frame and map (i.e. robot position)
        # See https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html#write-the-listener-node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.reached_intermediate_goal = False

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
            self.occupancy_map = np.transpose(self.occupancy_map)

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

    def coord2pixel(self, point):
        return (
            round((point[0] - self.origin[0])/(self.map_res)), 
            round((point[1] - self.origin[1])/(self.map_res))
        ) 

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
            self.get_logger().info(f"Current Pose: [{self.curr_pose.x:.3f},{self.curr_pose.y:.3f},{self.curr_pose.theta:.3f}]")
            self.get_logger().info(f"Going to: {point}")
            self.setpoint_pose.x = point[0]
            self.setpoint_pose.y = point[1]
            self.reached_intermediate_goal = False
            while not self.reached_intermediate_goal:
                self.update_current_pose()
                self.run_control_loop_once() # run 1 iteration of P control
                feedback_msg = Move2Goal.Feedback()
                feedback_msg.current_pose.pose.position.x = self.curr_pose.x
                feedback_msg.current_pose.pose.position.y = self.curr_pose.y
                goal_handle.publish_feedback(feedback_msg)
                rclpy.spin_once(self, timeout_sec=0.1) # 10Hz spin
        
        #Have now reached the final goal
        self.get_logger().info('Reached goal')
        goal_handle.succeed()
        result = Move2Goal.Result()
        result.reached_goal = True

        return result
    
    #P-controller helper functions
    def update_current_pose(self):
        """Get current pose of Turtlebot. Find the transform between base_footprint and map"""

        from_frame_rel = 'base_footprint'
        to_frame_rel = 'map'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            self.curr_pose.x = t.transform.translation.x
            self.curr_pose.y = t.transform.translation.y
            base_map_rot = t.transform.rotation
        
            quaternion = (
                base_map_rot.x,
                base_map_rot.y,
                base_map_rot.z,
                base_map_rot.w
                )
    
            _, _, self.curr_pose.theta = self.euler_from_quaternion(quaternion)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # self.get_logger().info(f"Current Pose: [{self.curr_pose.x:.3f},{self.curr_pose.y:.3f},{self.curr_pose.theta:.3f}]")

    def euler_from_quaternion(self, quaternion):       
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        (x,y,z,w) = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

    def bound(self, val:float, bound:float) ->float:
        """Limit value to given symmetric range"""
        return min(max(val, -bound), bound)
    
    def get_position_error(self):
        """Calculate error in position between current pose and the setpoint pose.
        """
        # As we independantly have another P controller to turn towards the setpoint, we can 
        # think of the position error as a straight line away from our current position.
        # So, use Euclidean distance as the error function.
        return sqrt(pow((self.setpoint_pose.x - self.curr_pose.x), 2) +
                    pow((self.setpoint_pose.y - self.curr_pose.y), 2))

    def get_linear_velocity(self, Kp=1.2) -> float:
        """Proportional controller for Position"""
        # if we need to rotate more than 60 degrees (arbitrarily), we should first turn and then move forward later
        # This will allow robot to stay on course
        if abs(self.get_angle_error()) > math.pi/3.0:
            return 0.0
        # As turtlebot gets closer to the setpoint, we automatically decrease it's linear velocity
        return self.bound(Kp * self.get_position_error(), 0.3) # Max linear velocity from Turtlebot4 datasheet

    def get_required_theta(self):
        """Calculate angle needed for turtlebot to face the setpoint"""
        return atan2(self.setpoint_pose.y - self.curr_pose.y, self.setpoint_pose.x - self.curr_pose.x)

    def get_angle_error(self):
        '''Calculate change in angle needed for turtlebot to face the setpoint'''
        angle_error = self.get_required_theta() - self.curr_pose.theta
        # Accounts for discontinous jump at pi -> -pi
        # Add or subtract a full turn (2pi rad) when the angle resulting angle is outside the (-pi, pi) range.
        # e.g. if current theta is 3.14 and desired theta is -3.1415, the smallest rotation angle would be 0.0015 rad 'left' and the rotation don't need to be 1 whole round rotation : = -6.2 rad
        if angle_error < -math.pi:
            angle_error = angle_error + 2*math.pi
        elif (angle_error > math.pi):
            angle_error = angle_error - 2*math.pi
        return angle_error

    def get_angular_velocity(self, Kp=6.2) -> float:
        '''Proportional controller for Required orientation to move towards setpoint position'''
        # As turtlebot faces the setpoint position more, we automatically decrease angular velocity
        return self.bound(Kp * self.get_angle_error(), 1.9) # Max angular velocity from Turtlebot4 datasheet

    def run_control_loop_once(self):
        # If our control loop is still active when the robot is really close, then we will start seeing unnecessary osciliations
        # due to control inaccuracies. e.g. Turtlebot jittering back and forth. 
        # So, if turtlebot is closer than 0.1 to setpoint, temporarily disable control loop
        # This avoids oscillation
        if self.get_position_error() >= 0.1:
            self.notified_planner = False # will need to notify planner, once we have reached setpoint
            self.vel_msg.linear.x = self.get_linear_velocity() # move towards setpoint
            self.vel_msg.angular.z = self.get_angular_velocity() # orient towards setpoint
        else:
            self.reached_intermediate_goal = True
            # Stopping our robot after the movement is over.
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
        self.publisher_vel.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    a_star_action_server = AStarActionServer()
    rclpy.spin(a_star_action_server)

if __name__ == '__main__':
    main()