
add_executable(move_group_target_pose src/move_group_target_pose.cpp)
target_link_libraries(move_group_target_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_target_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_listen src/move_group_listen.cpp)
target_link_libraries(move_group_listen ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_listen DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
