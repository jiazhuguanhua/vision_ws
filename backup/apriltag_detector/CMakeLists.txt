cmake_minimum_required(VERSION 3.0.2)
project(apriltag_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs cv_bridge image_transport common_msgs
)

# Make python scripts executable
catkin_install_python(PROGRAMS
  scripts/apriltag_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
