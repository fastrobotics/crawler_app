add_library(TankDriveNodeProcess TankDriveNodeProcess.cpp)
target_link_libraries(TankDriveNodeProcess ${catkin_LIBRARIES} baseNodeProcess)

add_executable(tankdrive_node TankDriveNode.cpp) 
target_link_libraries(tankdrive_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} TankDriveNodeProcess baseNode )
add_dependencies(tankdrive_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
