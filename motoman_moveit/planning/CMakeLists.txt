add_executable(move_group_server src/move_group_server.cpp)
target_link_libraries(move_group_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(get_glue src/get_glue.cpp)
target_link_libraries(get_glue ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS get_glue DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_listen src/move_group_listen.cpp)
target_link_libraries(move_group_listen ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_listen DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(right_pose src/right_pose.cpp)
target_link_libraries(right_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS right_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(left_pose src/left_pose.cpp)
target_link_libraries(left_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS left_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
