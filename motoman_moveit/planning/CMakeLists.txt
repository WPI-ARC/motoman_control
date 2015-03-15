add_executable(move_group_server src/move_group_server.cpp)
target_link_libraries(move_group_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(get_glue src/get_glue.cpp)
target_link_libraries(get_glue ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS get_glue DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
