#!/usr/bin/env python

from mte544_action_interfaces.srv import SetPose, SetPoseResponse
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import pow, atan2, sqrt

class PController(Node):
    '''Contains all aspects of this ROS node
    - current pose, goal, publisher, subscriber, and service
    '''
    # Pose of the turtle bot will be controlled by 2 seperate Proportional controllers
    # 1 for linear Position, and 1 for angular orientation
    # This way, we can independantly 'face' the setpoint, and then move towards it

    def __init__(self):
        super().__init__('p_controller')  # Declare ROS node
        self.srv = self.create_service(SetPose, 'set_pose', self.handle_pose_setpoint) # Declare ROS Service
        # TODO: Callback-based listening on TurtleBot pose updates
        # TODO: get tf between base_footprint and map
        self.subscriber_pose = self.create_subscription(Pose, '/amcl_pose', self.handle_pose_update)
        self.subscriber_setpoint = self.create_subscription(Point, '/set_position', self.handle_pose_setpoint)
        self.publisher_vel = self.create_publisher(Twist,'/cmd_vel', queue_size=10)
        self.timer = self.create_timer(0.1, self.run_control_loop) # 10Hz
        self.curr_pose = Pose() # holds current position of turtlebot. Updated when we receive a new message
        self.setpoint_pose = Pose() # self.goal_pose defaults to 0 till we receive a new setpoint from external node
        self.notified_planner = False

        print("Ready to control turtlebot")

    def handle_pose_setpoint(self, req: SetPose, response: SetPoseResponse):
        '''Receive new pose setpoint'''
        # Save new setpoint
        self.setpoint_pose.x = req.x
        self.setpoint_pose.y = req.y
        response.x = req.x
        response.y = req.y
        self.get_logger().info(f"Received new setpoint {req.x} {req.y}")
        return response
    
    def handle_pose_update(self, pose: Pose):
        '''Save current pose of Turtlebot'''
        self.curr_pose = pose
        pass

    def get_position_error(self):
        """Calculate error in position between current pose and the setpoint pose.
        """
        # As we independantly have another P controller to turn towards the setpoint, we can 
        # think of the position error as a straight line away from our current position.
        # So, use Euclidean distance as the error function.
        return sqrt(pow((self.setpoint_pose.x - self.curr_pose.x), 2) +
                    pow((self.setpoint_pose.y - self.curr_pose.y), 2))

    def get_linear_velocity(self, Kp=1.2):
        '''Proportional controller for Position'''
        # As turtlebot gets closer to the setpoint, we automatically decrease it's linear velocity
        return Kp * self.get_position_error()

    def get_required_steering(self):
        '''Calculate change in angle needed for turtlebot to face the setpoint'''
        return atan2(self.setpoint_pose.y - self.curr_pose.y, self.setpoint_pose.x - self.curr_pose.x)

    def get_angle_error(self):
        '''Calculate change in angle needed for turtlebot to face the setpoint'''
        return self.get_required_steering() - self.curr_pose.theta

    def get_angular_velocity(self, Kp=6.2):
        '''Proportional controller for Required orientation to move towards setpoint position'''
        # As turtlebot faces the setpoint position more, we automatically decrease angular velocity
        return Kp * self.get_angle_error()

    def run_control_loop(self):
        velocity_message = Twist()
        # If our control loop is still active when the robot is really close, then we will start seeing unnecessary osciliations
        # due to control inaccuracies. e.g. Turtlebot jittering back and forth. 
        # So, if turtlebot is closer than 0.1 to setpoint, temporarily disable control loop
        # This avoids oscillation
        if self.get_position_error() >= 0.1:
            self.notified_planner = False # will need to notify planner, once we have reached setpoint
            velocity_message.linear.x = self.get_linear_velocity() # move towards setpoint
            velocity_message.linear.y = 0
            velocity_message.linear.z = 0
            velocity_message.angular.x = 0
            velocity_message.angular.y = 0
            velocity_message.angular.z = self.get_angular_velocity() # orient towards setpoint
            self.publisher_vel.publish(velocity_message)
        else:
            if not self.notified_planner:
                # notify navigation_server that I am ready for the next pose
                goal_message = Point()
                goal_message.x = self.setpoint_pose.x
                goal_message.y = self.setpoint_pose.y
                self.publisher_pose.publish(goal_message)
                self.notified_planner = True
            # Stopping our robot after the movement is over.
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0
        self.publisher_vel.publish(velocity_message)

def main(args=None):
    rclpy.init(args=args)

    p_controller = PController()
    p_controller.start_control_loop()

    rclpy.spin(p_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    p_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()