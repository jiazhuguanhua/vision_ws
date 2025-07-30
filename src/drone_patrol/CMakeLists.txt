cmake_minimum_required(VERSION 3.0.2)
project(drone_patrol)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AnimalDetection.msg
  LinkStatus.msg
  FlightStatus.msg
  LandingCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    mavros_msgs 
    tf2_ros 
    tf2_geometry_msgs 
    cv_bridge 
    message_runtime
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/link_comm.py
  scripts/flight_control.py
  scripts/animal_detect.py
  scripts/landing_mode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/drone_patrol.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
