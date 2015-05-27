add_executable(convert_trajectory_server src/convert_trajectory_server.cpp)
target_link_libraries(convert_trajectory_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS convert_trajectory_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
