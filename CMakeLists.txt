cmake_minimum_required(VERSION 2.8.3)
add_definitions(-D__PX4_ROS)
add_definitions(-D__EXPORT=)
project(bjos)

# set required dependencies
find_package(px4 REQUIRED) 
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
    cmake_modules
    gazebo_msgs 
    sensor_msgs
    mav_msgs
    libmavconn
    tf
)

# set compiler and flags
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra --std=c++11 ${CMAKE_CXX_FLAGS}")

# set catkin settings
catkin_package(
  LIBRARIES active_safety
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs libmavconn
  DEPENDS system_lib
)

# include the headers
include_directories(include/)

# build the bjos library
add_library(bjos SHARED
    src/bjos/bjos.cpp
)

# build the test loader
add_executable(test_loader
    src/loader/test.cpp
)
# build the test executable
add_executable(test_test
    test/test.cpp
)

# link libraries
target_link_libraries(test_loader bjos ${catkin_LIBRARIES})
target_link_libraries(test_test bjos ${catkin_LIBRARIES})