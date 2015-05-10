cmake_minimum_required(VERSION 2.8.3)
project(task_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib
  tf
  smach_ros
  apc_util
)

# add_message_files(
#   FILES
#   APCItem.msg
#   WorkOrder.msg
#   )

catkin_python_setup()

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
  # LIBRARIES
  # INCLUDE_DIRS
  #   include
  # CATKIN_DEPENDS
)

install(DIRECTORY bin/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)