#!/usr/bin/env python

from beginner_tutorials.srv import SetPose,SetPoseResponse
import rospy

from geometry_msgs.msg import Twist
# rostopic info /turtle1/Pose shows it uses this type of message:
from turtlesim.msg import Pose 
from math import pow, atan2, sqrt

class PController:
    '''Contains all aspects of this ROS node
    - current pose, goal, publisher, subscriber, and service
    '''
    # Pose of the turtle bot will be controlled by 2 seperate Proportional controllers
    # 1 for linear Position, and 1 for angular orientation
    # This way, we can independantly 'face' the setpoint, and then move towards it

    def __init__(self):
        rospy.init_node('controller')  # Declare ROS node
        rospy.Service('set_pose', SetPose, self.handle_pose_setpoint) # Declare ROS Service
        # Callback-based listening on TurtleBot pose updates
        rospy.Subscriber('/turtle1/pose', Pose, self.handle_pose_update)
        # Hold reference to Publisher
        self.velocity_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Class variables
        self.curr_pose = Pose() # holds current position of turtlebot. Updated when we receive a new message
        self.setpoint_pose = Pose() # self.goal_pose defaults to 0 till we receive a new setpoint from external client
        self.rate = rospy.Rate(10) # Loop at 10Hz

        print("Ready to control turtlebot")

    def handle_pose_setpoint(self, req: SetPose):
        '''Receive new pose setpoint'''
        # Save new setpoint
        self.setpoint_pose.x = req.x
        self.setpoint_pose.y = req.y
        return SetPoseResponse(self.setpoint_pose.x, self.setpoint_pose.y)
    
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

    def start_control_loop(self):
        while not rospy.is_shutdown(): # run till ros is disconnected
            velocity_message = Twist()
            # If our control loop is still active when the robot is really close, then we will start seeing unnecessary osciliations
            # due to control inaccuracies. e.g. Turtlebot jittering back and forth. 
            # So, if turtlebot is closer than 0.01 to setpoint, temporarily disable control loop
            # This avoids oscillation
            while self.get_position_error() >= 0.01:

                velocity_message.linear.x = self.get_linear_velocity() # move towards setpoint
                velocity_message.linear.y = 0
                velocity_message.linear.z = 0
                velocity_message.angular.x = 0
                velocity_message.angular.y = 0
                velocity_message.angular.z = self.get_angular_velocity() # orient towards setpoint
                self.velocity_pub.publish(velocity_message)
                self.rate.sleep() # Publish at desired rate

            
            # Stopping our robot after the movement is over.
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0
            self.velocity_pub.publish(velocity_message)
            self.rate.sleep() # Publish at desired rate


if __name__ == "__main__":
    turtle = PController()
    turtle.start_control_loop()
