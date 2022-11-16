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
	
	rviz2_node = Node(
		package='rviz2',
		executable='rviz2',
		arguments=['-d', [os.path.join(home_path, 'MTE544-Labs/src/load_map_rviz', 'map_rviz.rviz')]]
	)
        
	ld.add_action(rviz2_node)
	return ld
