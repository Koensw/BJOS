cmake_minimum_required(VERSION 2.8.3)
project(bjos)
find_package(Boost COMPONENTS system thread REQUIRED)

# set compiler and flags
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -pedantic --std=c++11 ${CMAKE_CXX_FLAGS}")

# include the headers
include_directories(include/)

#### LIBS ####
# build the bjos library
add_library(bjos SHARED
    src/bjos/bjos.cpp
    src/bjos/helpers/error.cpp
    src/bjos/helpers/process.cpp
    src/controllers/SonarController.cpp
    src/controllers/FlightController.cpp
    src/controllers/flight/serial_port.cpp
    src/controllers/test_gripper.cpp
    src/libs/i2c.cpp
    src/libs/geometry.cpp
)
target_link_libraries(bjos ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} pthread rt wiringPi)

#### LOADERS ####
# build the test loader
add_executable(test_loader
    src/loader/test.cpp
)
target_link_libraries(test_loader bjos)

# flight loader
add_executable(flight_loader
    src/loader/test_flight.cpp
)
target_link_libraries(flight_loader ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} bjos)

# sonar loader
add_executable(sonar_loader
    src/controllers/sonar/DevantechSonarInterface.cpp
    src/loader/test_sonar.cpp
)
target_link_libraries(sonar_loader ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} bjos)

add_executable(default_loader
    src/controllers/sonar/DevantechSonarInterface.cpp
    src/loader/test_default.cpp
)
target_link_libraries(default_loader ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} bjos)

#### TEST ####
# build the test offboard executable
add_executable(test_offboard
    test/test_offboard.cpp
)
target_link_libraries(test_offboard bjos)

# build the test offboard interactive executable
add_executable(test_offboard_interactive
    test/test_offboard_interactive.cpp
)
target_link_libraries(test_offboard_interactive bjos)

# example controller
add_executable(bjos_example
    docs/ExampleController.cpp
)
target_link_libraries(bjos_example bjos)

# test_gripper
add_executable(test_gripper
    src/controllers/test_gripper.cpp
)
target_link_libraries(test_gripper bjos)




#### INSTALL ####
install(TARGETS bjos
        RUNTIME DESTINATION /usr/local/bin
        LIBRARY DESTINATION /usr/local/lib
        ARCHIVE DESTINATION /usr/local/lib/static)
install(DIRECTORY include/ DESTINATION /usr/local/include/bjos)

# TODO: BETTER INSTALL PLACE
# install(DIRECTORY libs/ DESTINATION /usr/local/include/bjos/libs)

