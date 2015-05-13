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

find_package(Boost 1.46 REQUIRED COMPONENTS filesystem)

## Find PCL
find_package(PCL 1.7 REQUIRED common features kdtree search filters segmentation)

catkin_python_setup()

add_service_files(
    FILES
    PublishPointcloudCollision.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
)

catkin_package(
  # LIBRARIES
  # INCLUDE_DIRS
  #   include
  # CATKIN_DEPENDS
)

add_definitions(${PCL_DEFINITIONS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(publish_pointcloud_collision_server src/publish_pointcloud_collision_server.cpp)
add_dependencies(publish_pointcloud_collision_server ${PROJECT_NAME}_gencpp)

target_link_libraries(publish_pointcloud_collision_server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY bin/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)