cmake_minimum_required(VERSION 3.0.2)
project(crawler_app)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage -fpermissive ${WARNING_FLAGS}")
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DARCHITECTURE_ARMV7L")
endif()
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  eros
  std_msgs
  sensor_msgs
)
find_package(GTest REQUIRED)
find_package(rostest REQUIRED)

catkin_package(
  CATKIN_DEPENDS eros
)
include_directories(
  ${catkin_INCLUDE_DIRS}

)

add_subdirectory(nodes)

# look for Doxygen package
find_package(Doxygen)