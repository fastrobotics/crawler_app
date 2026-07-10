cmake_minimum_required(VERSION 3.0.2)
project(crawler_app)

set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage -fpermissive ${WARNING_FLAGS}"
)
if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DARCHITECTURE_ARMV7L")
endif()
add_definitions(-std=c++14 -Wall -Wextra -Wno-switch -Werror)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs
                                        geometry_msgs message_generation genmsg)
find_package(GTest REQUIRED)
find_package(rostest REQUIRED)
