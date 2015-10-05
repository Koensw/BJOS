cmake_minimum_required(VERSION 2.8.3)
project(bjos)

# set compiler and flags
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -pedantic --std=c++11 ${CMAKE_CXX_FLAGS}")

# include the headers
include_directories(include/)

# build the bjos library
add_library(bjos SHARED
    src/bjos/bjos.cpp
    src/bjos/helpers/process.cpp
)
target_link_libraries(bjos pthread rt)

# build the test loader
add_executable(test_loader
    src/loader/test.cpp
)
# build the test executable
add_executable(test_test
    test/test.cpp
)

# example controller
add_executable(bjos_example
    docs/ExampleController.cpp
)
target_link_libraries(bjos_example bjos)

# link libraries
target_link_libraries(test_loader bjos)
target_link_libraries(test_test bjos)