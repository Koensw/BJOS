cmake_minimum_required(VERSION 2.8.3)
project(bjos)
find_package(Boost COMPONENTS system thread REQUIRED)

# set compiler and flags
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -pedantic --std=c++11 ${CMAKE_CXX_FLAGS}")

# include the headers
include_directories(include/)
include_directories(libs/)

#### LIBS ####
# build temporary libs
add_library(geometry SHARED
    libs/geometry.cpp
)
add_library(i2c SHARED
    libs/i2c.cpp
)

# build the bjos library
add_library(bjos SHARED
    src/bjos/bjos.cpp
    src/bjos/helpers/error.cpp
    src/bjos/helpers/process.cpp
    src/controllers/SonarController.cpp
    src/controllers/FlightController.cpp
    src/controllers/flight/serial_port.cpp
)
target_link_libraries(bjos ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} pthread rt geometry i2c)

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

#### TEST ####
# build the test executable
add_executable(test_test
    test/test.cpp
)
target_link_libraries(test_test bjos)

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
