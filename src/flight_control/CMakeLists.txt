cmake_minimum_required(VERSION 3.0.2)
project(flight_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs mavros_msgs common_msgs
)

# Make python scripts executable
catkin_install_python(PROGRAMS
  scripts/flight_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
