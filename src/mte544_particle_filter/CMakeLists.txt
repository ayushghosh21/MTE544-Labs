cmake_minimum_required(VERSION 3.5)
project(mte544_particle_filter)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)


find_package(geometry_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_py REQUIRED)




install(DIRECTORY
  launch
  src
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/run_particle_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
