cmake_minimum_required(VERSION 2.8.3)
project(softkinetic_description)

find_package(catkin)

catkin_package()

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
