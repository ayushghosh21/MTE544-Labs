import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()
	home_path = os.getenv("HOME")
	
	# Map server
	map_server_config_path = os.path.join(home_path, 'MTE544-Labs/src/load_map_rviz', 'map_maze_1.yaml')

	lifecycle_nodes = ['map_server']
	use_sim_time = True
	autostart = True

	start_lifecycle_manager_cmd = Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager',
		output='screen',
		parameters=[{'use_sim_time': use_sim_time},
		            {'autostart': autostart},
		            {'node_names': lifecycle_nodes}])

	map_server_cmd = Node(
	    package='nav2_map_server',
	    executable='map_server',
	    output='screen',
	    parameters=[{'yaml_filename': map_server_config_path}]
	    )
	
	# This odom_map_tf node for testing only. Publishes static TF between odom and map and bypasses Particle Filter.
	# Remove corresponding ld.add_action when we want to run actual particle filter.
	
	# TF betwen Map and odom just zero
	#args = ["0", "0", "0", "0", "0", "0", "map", "odom"]
	
	# Point 2 Ground Truth, TF between map and odom
	args = ["0", "0", "0", "-0.55", "0", "0", "map", "odom"]
	
	a#rgs = ["0", "0", "0", "0", "0", "0", "map", "odom"]
	# Point 5 Ground Truth, TF between map and odom
	#args = ["0.05", "-0.05", "0", "-0.61", "0", "0", "map", "odom"]
	
	#args = ["0.05", "-2.5", "0", "1.61", "0", "0", "map", "odom"]
	
	# Static TF node
	odom_map_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = args)
                       
	ld.add_action(map_server_cmd)
	ld.add_action(start_lifecycle_manager_cmd)
	ld.add_action(odom_map_tf)
	
	return ld
