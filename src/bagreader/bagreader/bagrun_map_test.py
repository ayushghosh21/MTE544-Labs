import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
import math
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
#from sklearn.neighbors import NearestNeighbors as KNN

class Bagreader(Node):


    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('bagreader')
        # create the subscriber object
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        

        self.viz_pub = self.create_publisher(MarkerArray, 'viz_topic_array', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ## TODO Define these variables from map input!!!!!!!
        self.map_max_x = 356
        self.map_max_y = 457
        self.map_res = 0.03

        self.laser_forward = LaserScan()
        self.laser_cart = []

        self.import_occupancy_map()


    def import_occupancy_map(self):

        import matplotlib.pyplot as plt
        f = 'map_maze_1.pgm'
        with open(f, 'rb') as pgmf:
            im = plt.imread(pgmf)
            
        print(np.unique(im))
        
        print(im.dtype)
        print(im.shape)
        
        
       # im[np.where(im == 205)] = 254
        
        print(~im.astype(np.bool))
        plt.imshow(~im.astype(np.bool), cmap='gray')
        
        plt.show()

        return ~im.astype(np.bool)


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
        # robot_pose = [trans_base_map.x, trans_base_map.y, yaw_base_map]

        robot_pose = [0, 0, 0]
        lidar_cart = self.transform_laser(robot_pose)

        #map_lidar_pts = self.cartesion_to_map(lidar_cart)
        

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
    

    def vizualize_points(self, lidar_data):
        
        markers = MarkerArray()
        
        
        for idx, val in enumerate(lidar_data):
            ellipse = Marker()
            ellipse.header.frame_id = 'map'
            ellipse.header.stamp = self.get_clock().now().to_msg()
            ellipse.type = Marker.CUBE
            ellipse.pose.position.x = val[0] 
            ellipse.pose.position.y = val[1]
            ellipse.pose.position.z = 0.0
            ellipse.scale.x = 0.02
            ellipse.scale.y = 0.02
            ellipse.scale.z = 0.02
            
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

    def cartesion_to_map(self, lidar_points):

        point_x = lidar_points[:,0]
        point_y = lidar_points[:,1]

        point_x = point_x[np.isfinite(point_x)]
        point_y = point_y[np.isfinite(point_y)]
        
        point_x = point_x/self.map_res
        point_y = point_y/self.map_res
        
        
        point_x = point_x.astype(int)
        point_y = point_y.astype(int)
        
        point_map_x = point_x + self.map_origin_pt[0]/self.map_res
        point_map_y = point_y + self.map_max_h - self.map_origin_pt[1]/self.map_res

        point_map_x[point_map_x > self.map_max_w] = np.NaN
        point_map_x[point_map_x < -self.map_max_w] = np.NaN

        point_map_y[point_map_y < -self.map_max_h] = np.NaN
        point_map_y[point_map_y > self.map_max_h] = np.NaN



        point_x = point_x[np.isfinite(point_x)]
        point_y = point_y[np.isfinite(point_y)]

        occupancy_map = np.zeros((self.map_max_h, self.map_max_w))

        
        
        for i, (x, y) in enumerate(zip(point_x, point_x)):
            occupancy_map[y, x] = 1    
        
        return occupancy_map

# Particle Filter applied to map
# TODO: on startup create an initial list of points
NUM_PARTICLES=1000

def sample_motion_model(inputs:None, posterior_sample:list) -> list:
    # Robot is static, hence don't need a motion model
    # Hence new predicted sample = posterior sample. Inputs u_k-1 are ignored
    return posterior_sample

def particle_filter_loop(posterior_samples:list, inputs: None):
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
    predicted_samples = []
    resampled_samples = []
    weights = []
    for i in range(NUM_PARTICLES):
        posterior_sample_eta_k_minus1 = posterior_samples[i]
        # Pass sample i through motion model
        predicted_sample_eta_k = sample_motion_model(inputs,posterior_sample_eta_k_minus1)
        # evaluate sample according to sensor measurement model
        w_k = likelihood_field(predicted_sample_eta_k)
        weights.append(w_k)
        predicted_samples.append(predicted_sample_eta_k)
    
    for i in range(NUM_PARTICLES):
        # resampling. Draw new samples according to importance weight wks
        # draw new sample from predicted_samples according to distribution of w_k
        resampled_sample = np.random.choice(predicted_samples, weights)
        resampled_samples.append(resampled_sample)
    return resampled_samples

def likelihood_field(self,predicted_sample):
    """
    Measurement Model - Likelihood field. Lec 15 Slide 79
    lidar_points: zk_measurements, in sensor frame. We get this from the class
    """

    # Get sensor measurement data and convert co-ord frame from sensor_frame to global_frame using predicted robot position
    lidar_points = self.transform_laser(predicted_sample)
    lidar_points = np.argwhere(self.occupancy_map == 1) #Lidar occupancy grid
    # We use numpy operations to perform a single compuation of distance of each lidar point to the closest object
    #p_rand_mult = 1.0/lidar_points.amax()
    #using knn to find closest point on the map
    ob = np.argwhere(self.occupancy_map == 1) #Map occupancy grid
    nbrs = KNN(n_neighbors=1, algorithm='ball_tree').fit(ob)

    lidar_map_points = lidar_points * self.map_res # Converting  to pixels
    lidar_map_points = lidar_map_points[self.M_idxs,:]
    _, indices = self.nbrs.kneighbors(lidar_map_points)
    z = ob[indices].reshape(lidar_map_points.shape)
    # distance between 2 points in pixels
    dist = np.linalg.norm(lidar_map_points-z)

    # probability of each point hitting
    pz = 1/(math.sqrt(2*math.pi)) * np.exp(-(0.5 * dist**2)) #+ p_rand_mult, ignore noise?

    q = np.prod(pz)

    return q

    


def draw_random_points(max_x, max_y, n):
    """"Initalize particle filter samples distribution"""
    random_image = np.random.random_sample(size=(max_x, max_y)) > 0.5
    sel_index = np.random.choice(np.argwhere(random_image.ravel()).ravel(), size=n)
    random_x, random_y = np.unravel_index(sel_index, (max_x, max_y))





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
