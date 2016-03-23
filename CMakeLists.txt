cmake_minimum_required(VERSION 2.8.3)
project(bjos)
find_package(Boost COMPONENTS atomic filesystem system thread REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_path(BJCOMM_INCLUDE_DIR NAMES bjcomm/interface.h PATHS /usr/include /usr/local/include)
find_library(BJCOMM_LIBRARY libbjcomm.so)
include_directories(SYSTEM ${BJCOMM_INCLUDE_DIR})

# load eigen3
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS} )

# set compiler and flags
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
    src/controllers/GripperController.cpp
    src/controllers/EyesController.cpp
    src/controllers/RGBEyesController.cpp
    src/controllers/flight/serial_port.cpp
    src/libs/i2c.cpp
    src/libs/log.cpp
    src/libs/dma.c
    src/libs/mailbox.c
    src/libs/pwm.c
    src/libs/rpihw.c
    src/libs/ws2811.c
)
target_link_libraries(bjos ${Boost_LIBRARIES} pthread rt wiringPi ${BJCOMM_LIBRARY} zmq) #FIXME: zmq should not need an explicit specification

#### LOADERS ####
# flight loader
add_executable(flight_loader
    src/loader/test_flight.cpp
)
target_link_libraries(flight_loader bjos)

# sonar loader
add_executable(sonar_loader
    src/controllers/sonar/DevantechSonarInterface.cpp
    src/loader/test_sonar.cpp
)
target_link_libraries(sonar_loader bjos)

# gripper loader
add_executable(gripper_loader
    src/loader/test_gripper.cpp
)
target_link_libraries(gripper_loader bjos)

# rgbeyes loader
add_executable(rgbeyes_loader
    src/loader/test_RGBeyes.cpp
)
target_link_libraries(rgbeyes_loader bjos)

# default loader
add_executable(default_loader
    src/controllers/sonar/DevantechSonarInterface.cpp
    src/loader/test_default.cpp
)
target_link_libraries(default_loader bjos)

### TEST ###
# build the test offboard interactive executable
add_executable(test_offboard_interactive
    test/test_offboard_interactive.cpp
)
target_link_libraries(test_offboard_interactive bjos)

# build the test sync vision executable
add_executable(test_sync_vision
    test/test_sync_vision.cpp
)
target_link_libraries(test_sync_vision bjos)

# test logger controller
add_executable(test_receiver
    test/test_receiver.cpp
)
target_link_libraries(test_receiver bjos)

# test_gripper
add_executable(test_gripper
    test/test_gripper.cpp
)
target_link_libraries(test_gripper bjos)

# align
add_executable(align_frames
    test/align_frames.cpp
)
target_link_libraries(align_frames bjos)

# test estimate
add_executable(estimate_simulator
    test/estimate_simulator.cpp
)
target_link_libraries(estimate_simulator bjos)

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

