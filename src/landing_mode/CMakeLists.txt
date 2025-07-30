cmake_minimum_required(VERSION 3.0.2)
project(landing_mode)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs mavros_msgs
)

catkin_install_python(PROGRAMS
  scripts/landing_mode_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
