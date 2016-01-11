cmake_minimum_required(VERSION 2.8.3)
project(bjos)
find_package(Boost COMPONENTS system thread REQUIRED)

# load eigen3
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

# set compiler and flags
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -pedantic --std=c++11 ${CMAKE_CXX_FLAGS}")

# include the headers
include_directories(include/)

#### LIBS ####
# build the bjos library
add_library(bjos SHARED
    src/bjos/bjos.cpp
    src/bjos/controller.cpp
    src/bjos/helpers/error.cpp
    src/bjos/helpers/process.cpp
    src/controllers/SonarController.cpp
    src/controllers/FlightController.cpp
    src/controllers/flight/serial_port.cpp
    src/libs/i2c.cpp
)
target_link_libraries(bjos ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} pthread rt bjcomm zmq) #FIXME: zmq should not need an explicit specification

#### LOADERS ####
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

### TEST ###

# build the test offboard interactive executable
add_executable(test_offboard_interactive
    test/test_offboard_interactive.cpp
)
target_link_libraries(test_offboard_interactive bjos)

# test logger controller
add_executable(test_receiver
    test/test_receiver.cpp
)
target_link_libraries(test_receiver bjos)

# example controller
add_executable(bjos_example
    docs/ExampleController.cpp
)
target_link_libraries(bjos_example bjos)

#### INSTALL ####
install(TARGETS bjos
        RUNTIME DESTINATION /usr/local/bin
        LIBRARY DESTINATION /usr/local/lib
        ARCHIVE DESTINATION /usr/local/lib/static)
install(DIRECTORY include/ DESTINATION /usr/local/include/bjos)

# TODO: BETTER INSTALL PLACE
# install(DIRECTORY libs/ DESTINATION /usr/local/include/bjos/libs)

