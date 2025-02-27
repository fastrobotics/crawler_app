catkin_add_gtest(test_TankDriveNodeProcess test_TankDriveNodeProcess.cpp)
target_link_libraries(test_TankDriveNodeProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger TankDriveNodeProcess)
add_dependencies(test_TankDriveNodeProcess eros_generate_messages_cpp)

add_rostest_gtest(test_TankDriveNode
    test_TankDriveNode.test
    test_TankDriveNode.cpp
)
add_dependencies(test_TankDriveNode tankdrive_node)
target_link_libraries(test_TankDriveNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)
