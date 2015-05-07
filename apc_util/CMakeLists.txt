cmake_minimum_required(VERSION 2.8.3)
project(apc_util)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib
  tf
)


catkin_python_setup()

catkin_package(
  # LIBRARIES
  # INCLUDE_DIRS
  #   include
  # CATKIN_DEPENDS
)

install(DIRECTORY bin/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)