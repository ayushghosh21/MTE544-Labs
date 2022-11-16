import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
import math
from math import sqrt
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors as KNN
from sklearn.neighbors import KDTree
from scipy.ndimage import rotate

test = True

class Bagreader(Node):


    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('bagreader')
        # create the subscriber object
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        

        self.viz_pub = self.create_publisher(MarkerArray, 'viz_topic_array', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.laser_forward = LaserScan()
        self.NUM_PARTICLES = 5000
        
        self.origin = [0,0,0]
        self.map_res = 0.03

        self.occupancy_map = self.import_occupancy_map()

        self.map_max_w = self.occupancy_map.shape[1]
        self.map_max_h = self.occupancy_map.shape[0]
        
        self.ob_list = self.occupancy_to_list()

        # Create KDTree for finding the closest point on the map, in likli
        self.kdt=KDTree(self.ob_list)

        self.particles = self.initialize_particle_filter()
        
        
    def import_occupancy_map(self, f='map_maze_1.pgm', yaml_f="map_maze_1.yaml"):
        """Load provided map and convert to coordinates in meters"""

        import matplotlib.pyplot as plt

        with open(f, 'rb') as pgmf:
            im = plt.imread(pgmf)
        ob = ~im.astype(np.bool)
        #plt.imshow(ob, cmap='gray')
        #plt.show()


        import yaml
        with open(yaml_f, 'r') as stream:
            data_loaded = yaml.safe_load(stream)

        self.origin = data_loaded['origin']
        self.map_res = data_loaded['resolution']

        return ob

    def occupancy_to_list(self):

          # Convert Occupancy Map to Coordinates
        ob_rotated = rotate(self.occupancy_map, -90, reshape=True)

        ob = np.argwhere(ob_rotated == 1)*self.map_res #Map occupancy grid
        axis_offsets = np.array([self.origin[0], self.origin[1]])
        
        ob = ob + axis_offsets[None,:]

        return ob
    # need to setup the laser callback message

    def laser_callback(self, msg):
        self.laser_forward = msg
        #print(self.laser_forward)

        # t_base_map = self.tf_buffer.lookup_transform(
        #         'map',
        #         'base_link',
        #         rclpy.time.Time())
            
        # trans_base_map = t_base_map.transform.translation
        # rot_base_map = t_base_map.transform.rotation
        
        # quaternion = (
        #     rot_base_map.x,
        #     rot_base_map.y,
        #     rot_base_map.z,
        #     rot_base_map.w
        #     )
            
        # roll_base_map, pitch_base_map, yaw_base_map = self.euler_from_quaternion(quaternion)
        #robot_pose = [trans_base_map.x, trans_base_map.y, yaw_base_map]
        #lidar_cart = self.transform_laser(robot_pose)
        import time
        start_time = time.time()
        self.particle_filter()
        print("PF--- %s seconds ---" % (time.time() - start_time))
        
        import time
        start_time = time.time()

        self.vizualize_points(self.particles)
        print("--- %s seconds ---" % (time.time() - start_time))
        # self.get_average_position()

        # this takes the value at angle 359 (equivalent to angle 0)
    
    def transform_laser(self, robot_pose = [0, 0, 0]):
        from_frame_rel = 'rplidar_link'
        to_frame_rel = 'base_link'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            trans = t.transform.translation
            rot = t.transform.rotation
            
            quaternion = (
                rot.x,
                rot.y,
                rot.z,
                rot.w
                )
                
            roll, pitch, yaw = self.euler_from_quaternion(quaternion)

            #print("trans", t.transform.translation.x, t.transform.translation.y)
            #print("rotation", yaw)

            delta_base_lidar = [trans.x, trans.y, yaw]
            #print(delta_base_lidar)
            lidar_points = self.lidar_to_cartesian(robot_pose, delta_base_lidar)
            
            return lidar_points
            
            #self.vizualize_points(lidar_points)       
            #print(np.amax(lidar_points))

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
    
    def euler_from_quaternion(self, quaternion):

        (x,y,z,w) = quaternion
        
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
        
    def vizualize_points(self, points):
        
        markers = MarkerArray()
        
        
        for idx, val in enumerate(points):
            ellipse = Marker()
            ellipse.header.frame_id = 'map'
            ellipse.header.stamp = self.get_clock().now().to_msg()
            ellipse.type = Marker.ARROW
            ellipse.pose.position.x = val[0] 
            ellipse.pose.position.y = val[1]
            ellipse.pose.position.z = 0.0

            [qx, qy, qz, qw] = self.get_quaternion_from_euler(0, 0 , val[2])

            ellipse.pose.orientation.x = qx
            ellipse.pose.orientation.y = qy
            ellipse.pose.orientation.z = qz
            ellipse.pose.orientation.w = qw

            ellipse.scale.x = 0.1
            ellipse.scale.y = 0.1
            ellipse.scale.z = 0.1
            
            ellipse.color.a = 1.0
            ellipse.color.r = 0.0
            ellipse.color.g = 1.0
            ellipse.color.b = 0.0
            
            ellipse._id = idx
            markers.markers.append(ellipse)

        self.viz_pub.publish(markers)

    def lidar_to_cartesian(self, robot_pose, delta_base_lidar=[0, 0, 0]):
        
        # Current robot pose
        curr_x = robot_pose[0]
        curr_y = robot_pose[1]
        curr_yaw = robot_pose[2]  
        
        delta_x = delta_base_lidar[0]
        delta_y = delta_base_lidar[1]
        delta_yaw = delta_base_lidar[2]
        
        ranges_data = self.laser_forward.ranges
        num_points = len(ranges_data)
        point_angles = np.zeros(num_points)
        point_x = np.zeros(num_points)
        point_y = np.zeros(num_points)


        point_indexes = np.arange(len(ranges_data))
        point_angles_idx_increment = np.multiply(point_indexes, self.laser_forward.angle_increment)

        point_angles = np.add(self.laser_forward.angle_min, point_angles_idx_increment) + curr_yaw + delta_yaw

        point_x = np.multiply(ranges_data, np.cos(point_angles)) + curr_x - delta_x
        point_y = np.multiply(ranges_data, np.sin(point_angles)) + curr_y - delta_y
        
        point_x = point_x[np.isfinite(point_x)]
        point_y = point_y[np.isfinite(point_y)]
        
        
        lidar_points = np.stack((point_x, point_y), axis=-1)

        return lidar_points
        
    def likelihood_field(self, predicted_sample):
        
        """
        Measurement Model - Likelihood field. Lec 15 Slide 79

        Parameters:
        predicted_sample: the current particle's pose.
        lidar_points: zk_measurements, in sensor frame. We get this from the class impicitly
        """

        # distribution paramters
        # We have set it similar to example from class, where n1=1,n2=0,n3=0
        # The lidar datasheet shows an accuracy of 1% for a full scale range of 12m.
        # Assume that this accuracy corresponds to a full scale acceptable measurment range of +- 3 std dev. 
        # Thus the standard deviation is calculated as:
        lidar_standard_deviation = (0.01*12)*2/6

        #Measured lidar scan at pose predicted_sample
        lidar_points = self.transform_laser(predicted_sample)
        # lidar_points has all infinite beams filtered out
        # lidar_points is already converted to the global map frame
        
        # testing plotting of occupancy map vs transformed lidar points
        # global test
        # if(test):
        #     test2 = rotate(self.occupancy_map, -90, reshape=True)
        #     test_ob = np.argwhere(test2 == 1)*self.map_res
        #     plt.scatter(ob[:,0],ob[:,1],marker='.')
        #     plt.scatter(lidar_points[:,0],lidar_points[:,1],marker='.')
        #     plt.show()
        #     test = False

        # calculate distance between each lidar point and its respective point
        dist=self.kdt.query(lidar_points, k=1)[0][:]
        
        # probability of each point hitting - we ignore p_rand and p_max, and sum to find overall weight
        weight= np.sum (np.exp(-(dist**2)/(2*lidar_standard_deviation**2)))
        return weight

    def sample_motion_model(inputs:None, posterior_sample):
        """"Motion Model
        Robot is static, hence don't need a motion model
        """
        # New predicted sample = posterior sample as there is no motion
        return posterior_sample

    def particle_filter_loop(self, posterior_samples):
        """
        Particle Filter Algorithim. 
        Each particle contains a state (x,y,theta) = pose of the robot

        Lec 16 Slide 94. 
        Parameters:
        posterior_samples - list of samples, representing the previous belief. 
        inputs - u_k-1. Not used since no motion
        zk_measurements: list of points from lidar scan. Accessed internally, not passed in
        """

        # reset list
        predicted_samples = np.zeros((self.NUM_PARTICLES,3))
        resampled_samples = []
        weights = np.zeros((self.NUM_PARTICLES))
        
        import time
        start_time = time.time()

        for i in range(self.NUM_PARTICLES):
            posterior_sample_eta_k_minus1 = posterior_samples[i]
            # Pass sample i through motion model
            predicted_sample_eta_k = self.sample_motion_model(posterior_sample_eta_k_minus1)
            # evaluate sample according to sensor measurement model
            w_k = self.likelihood_field(predicted_sample_eta_k)
            weights[i] = w_k
            predicted_samples[i]= predicted_sample_eta_k

        print("Liklihood--- %s seconds ---" % (time.time() - start_time))
        # Normalize weights
        
        weights = weights/np.sum(weights)

        # resampling. Draw new samples according to importance weight wks
        # draw new sample from predicted_samples according to distribution of w_k
        resampled_samples = predicted_samples[np.random.choice(self.NUM_PARTICLES,size=self.NUM_PARTICLES,p=weights)]
            
        return resampled_samples
    
    def particle_filter(self):
        # Particle Filter applied to map
        self.particles = self.particle_filter_loop(self.particles)

    def get_average_position(self):
        """Get estimated position of robot by averaging all particles"""
        # Get mean of particles along each axis
        average_pose = np.mean(self.particles, axis=0)
        return average_pose

    def initialize_particle_filter(self):
        """"Initalize particle filter by creating a uniform list of particles"""
        # np.random.choice return a uniform distribution by default
        sel_index = np.random.choice(self.map_max_w*self.map_max_h, replace = False, size=self.NUM_PARTICLES)
        # Generate random x,y,theta uniformly over the entire map
        random_x, random_y = np.unravel_index(sel_index, (self.map_max_w, self.map_max_h))
        random_angle = np.random.uniform(0,2*math.pi,size=self.NUM_PARTICLES)

        # random_particles is array of size NUM_PARTICLES x 3
        random_particles = np.stack((random_x * self.map_res, random_y * self.map_res, random_angle),axis=-1)
        
        # Coordinate transform
        axis_offsets = np.array([self.origin[0], self.origin[1], 0])
        
        random_particles = random_particles + axis_offsets[None,:]
        
        return random_particles

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    bagreader = Bagreader()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(bagreader)
    # Explicity destroys the node
    bagreader.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
