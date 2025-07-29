cmake_minimum_required(VERSION 3.0.2)
project(precision_landing)

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
  scripts/precision_landing_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
