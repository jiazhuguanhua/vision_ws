cmake_minimum_required(VERSION 3.0.2)
project(animal_detect)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge
)

catkin_install_python(PROGRAMS
  scripts/animal_detect_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
