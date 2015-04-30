add_executable(convert_trajectory_server src/convert_trajectory_server.cpp)
target_link_libraries(convert_trajectory_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS convert_trajectory_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(get_tray_server src/get_tray_server.cpp)
target_link_libraries(get_tray_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS get_tray_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(attach_tray_server src/attach_tray_server.cpp)
#target_link_libraries(attach_tray_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS attach_tray_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(get_tray_test1 src/get_tray_test1.cpp)
target_link_libraries(get_tray_test1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS get_tray_test1 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
