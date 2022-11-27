cmake_minimum_required(VERSION 3.5)
project(mte544_a_star)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)


find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(mte544_action_interfaces REQUIRED)


install(DIRECTORY
  launch
  src
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/mte544_navigation_server.py
  src/mte544_navigation_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
