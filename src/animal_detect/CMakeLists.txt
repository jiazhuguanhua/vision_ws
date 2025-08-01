cmake_minimum_required(VERSION 3.0.2)
project(animal_detect)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LaserControl.msg
  AnimalDetection.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge message_runtime
)

catkin_install_python(PROGRAMS
  scripts/animal_detect_node.py
  scripts/test_animal_detect.py
  scripts/demo_message_format.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
