# MTE 544 Lab 2 - Particle Filter Localization
> By Group 8, Ayush Ghosh, Nick Shaju, Abhinav Agrahari


## Setup
To setup the `mte544_particle_filter` package, build and source this workspace: 

```colcon build```

```source install/setup.bash```



## Point 2 Localization
Ensure the workspace is sourced in the terminal:

```source install/setup.bash```

1. Run the sample launch file:

```ros2 launch mte544_particle_filter particle_filter.launch.py```


2. Wait for the launch file to finish loading (it will take at least 2.5 seconds). A map will appear in Rviz2 and a message displaying "Ready!" will show in the terminal.

Note: If the map does not appear, try re-running the launch file.

Open a new terminal in the root of this workspace, and run the Point 2 bag file:

```ros2 bag play src/mte544_particle_filter/bag_files/point2```

Wait a few moments for the bag file to start sending data. The particles and super-imposed lidar scan will be updated for each iteration of the particle filter.

## Point 5 Localization
Ensure the workspace is sourced in the terminal:

```source install/setup.bash```

1. Run the sample launch file:

```ros2 launch mte544_particle_filter particle_filter.launch.py```


2. Wait for the launch file to finish loading (it will take at least 2.5 seconds). A map will appear in Rviz2 and a message displaying "Ready!" will show in the terminal.

Note: If the map does not appear, try re-running the launch file.

Open a new terminal in the root of this workspace, and run the Point 5 bag file:

```ros2 bag play src/mte544_particle_filter/bag_files/point5```

Wait a few moments for the bag file to start sending data. The particles and super-imposed lidar scan will be updated for each iteration of the particle filter.