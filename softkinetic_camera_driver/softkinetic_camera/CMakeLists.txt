cmake_minimum_required(VERSION 2.8.3)
project(softkinetic_camera)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pcl_ros cv_bridge image_transport camera_calibration_parsers resource_retriever)

## Find the DepthSense SDK
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
find_package(DepthSenseSDK)

## Catkin package
catkin_package(CATKIN_DEPENDS roscpp std_msgs pcl_ros cv_bridge image_transport camera_calibration_parsers resource_retriever DEPENDS system_lib DepthSenseSDK)

## Setup include paths
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${DEPTHSENSESDK_INCLUDE_DIRS})

## Only build driver if we have the DepthSense SDK
if(DEPTHSENSESDK_FOUND)
  message(STATUS "DepthSense SDK found. Building SoftKinetic camera driver.")
  ## Enable optimizations
  set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")
  ## Build executable
  add_executable(softkinetic_driver_node src/softkinetic_driver_node.cpp)
  add_dependencies(softkinetic_driver_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(softkinetic_driver_node ${catkin_LIBRARIES} ${DEPTHSENSESDK_LIBRARIES})
  ## Install the node
  install(TARGETS softkinetic_driver_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(WARNING "DepthSense SDK not found. SoftKinetic camera driver will not be built.")
endif()

