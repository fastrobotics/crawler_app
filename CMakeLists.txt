cmake_minimum_required(VERSION 3.8)
project(crawler_app)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robot_config/robot_bringup/launch/
        DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY robot_config/robot_bringup/config/
        DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/playback_bag.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
