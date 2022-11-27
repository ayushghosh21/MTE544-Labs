import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():

	lifecycle_nodes = [
		'map_server',
		# 'amcl',
		'planner_server'
	]

	map_file = os.path.join(get_package_share_directory("mte544_a_star"), 'launch', 'map.yaml')
	params_file = os.path.join(get_package_share_directory("mte544_a_star"), 'param', 'burger.yaml')
	
	return launch.LaunchDescription([
		
		# Start map_server and lifecycle node after 2 seconds 
		launch.actions.TimerAction(
			period=1.5,
			actions=[
				launch_ros.actions.Node(
					package='nav2_map_server',
					executable='map_server',
					name='map_server',
					output='screen',
					parameters=[{'yaml_filename': map_file}]
				),
			],
		),

        # launch_ros.actions.Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen'),
        #     #parameters=[{''}]),

        launch_ros.actions.Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{params_file}]
		),
		
		launch_ros.actions.Node(
			package = "tf2_ros", 
			executable = "static_transform_publisher",
			arguments = ["2", "0.5", "0", "0", "0", "0", "map", "odom"]
		),

        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            output='screen',
            parameters=[
				{'use_sim_time':False},
				{'autostart': True},
				{'node_names': lifecycle_nodes},
                {'params_file':params_file},
			]),
		launch_ros.actions.Node(
			package='rviz2',
			executable='rviz2',
			arguments=['-d', [os.path.join(get_package_share_directory("mte544_a_star"), 'launch', 'map_rviz.rviz')]]
		),
    ])