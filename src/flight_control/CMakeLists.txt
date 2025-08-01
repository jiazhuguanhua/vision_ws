cmake_minimum_required(VERSION 3.0.2)
project(flight_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  waypoint_planner
  sensor_msgs
  cv_bridge
  animal_detect
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs mavros_msgs waypoint_planner sensor_msgs cv_bridge animal_detect common_msgs
)

catkin_install_python(PROGRAMS
  scripts/offb_node_multiwpt.py
  scripts/px4_flight_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
