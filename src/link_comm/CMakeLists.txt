cmake_minimum_required(VERSION 3.0.2)
project(link_comm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/link_comm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
