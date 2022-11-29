# MTE 544 Lab 3 - Path Planning and Control
> By Group 8, Ayush Ghosh, Nick Shaju, Abhinav Agrahari


## Setup
To setup the `mte544_a_star` package, build and source this workspace: 

```bash
pip install scikit-image
colcon build --symlink-install
source install/setup.bash
```

## Running

In 4 seperate terminals:
- `ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py`
- `ros2 launch mte544_a_star mte544_a_star.launch.py`
- `ros2 run mte544_a_star mte544_navigation_server.py`
- `ros2 run mte544_a_star mte544_navigation_client.py`

Then through RViz, give a `2D Pose estimate` and a `2D Goal Pose`
