import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	
	# Map server
	map_server_config_path = os.path.join(get_package_share_directory("mte544_particle_filter"), 'launch', 'map_maze_1.yaml')

	lifecycle_nodes = ['map_server']
	use_sim_time = False
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
	
	rviz2_node = Node(
		package='rviz2',
		executable='rviz2',
		arguments=['-d', [os.path.join(get_package_share_directory("mte544_particle_filter"), 'launch', 'map_rviz.rviz')]]
	)

	particle_filter_node = Node(
	    package='mte544_particle_filter',
	    executable='run_particle_filter.py',
	    output='screen',
	)

	return LaunchDescription([
		rviz2_node,
		#map_server_cmd,
		#start_lifecycle_manager_cmd,
		TimerAction(
			period=1.0,
			actions=[map_server_cmd, start_lifecycle_manager_cmd],
		),
		TimerAction(
			period=1.5,
			actions=[particle_filter_node],
		),
	])
