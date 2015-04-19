cmake_minimum_required(VERSION 2.8.3)
project(motoman_moveit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules std_msgs message_generation)

find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
    FILES
    move_group_server.srv
    convert_trajectory_server.srv
)

generate_messages(
    DEPENDENCIES
    motoman_msgs
    std_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motoman_moveit
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    sensor_msgs
    std_msgs
    message_runtime
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(planning)
