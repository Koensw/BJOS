#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>

#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"

#include "controllers/FlightController.h"
#include "controllers/GripperController.h"

/*
 *  Test offboard
 */

using namespace bjos;

float VEL = 0.5;
float TAKEOFF = 2.0;
float ZVEL = 0.5;
float YAWR = 0.2;

#define M_EPS 1e-7

std::tuple<Eigen::Vector3d, Eigen::Vector3d, uint16_t, uint8_t> handle_input(char c)
{
	static Eigen::Vector3d setv_old;
    static Eigen::Vector3d setav_old;
    static uint16_t tm_old;
    static uint8_t gripAction_old;
    Eigen::Vector3d setv(0,0,0);
    Eigen::Vector3d setav(0,0,0);
    uint16_t tm = SET_TARGET_VELOCITY;
    uint8_t gripAction = 0;

	switch (c) {
	case 'w':
		std::cout << "Going forward" << std::endl;
		setv[0] = VEL;
		break;

	case 's':
		std::cout << "Going backward" << std::endl;
		setv[0] = -VEL;
		break;

	case 'a':
		std::cout << "Going left" << std::endl;
		setv[1] = VEL;
		break;

	case 'd':
		std::cout << "Going right" << std::endl;
		setv[1] = -VEL;
		break;

	case 'e':
		std::cout << "Turning left" << std::endl;
        setav[2] = YAWR;
		setv = setv_old;
        tm &= SET_TARGET_YAW_RATE;
		break;

	case 'r':
		std::cout << "Turning right" << std::endl;
        setav[2] = -YAWR;
		setv = setv_old;
        tm &= SET_TARGET_YAW_RATE;
		break;

	case 'g':
		std::cout << "Landing" << std::endl;
        tm = SET_TARGET_LAND;
		break;

    case 't':
		std::cout << "Takeoff!" << std::endl;
        tm = SET_TARGET_TAKEOFF;
		break;

	case 'o':
		std::cout << "Going upward" << std::endl;
		setv[2] = ZVEL;
		break;

	case 'l':
		std::cout << "Going downward" << std::endl;
		setv[2] = -ZVEL;
		break;

    case 'n':
        std::cout << "Gripper closing" << std::endl;
        gripAction = 1;
        break;

    case 'm':
        std::cout << "Gripper opening" << std::endl;
        gripAction = 2;
        break;

	case 'q':
		std::cout << "Quitting... :(" << std::endl;
		break;

	case '\n':
		setv = setv_old;
        setav = setav_old;
        tm = tm_old;
        gripAction = gripAction_old;
		break;

	case 'h':
	default:
		std::cout << "Holding..." << std::endl;
		break;
	}
    setv_old = setv;
    setav_old = setav;
    tm_old = tm;
    gripAction_old = gripAction;

    return std::make_tuple(setv, setav, tm, gripAction);
}

int main(){
    Process::installSignalHandler();
    //BJOS::init();
    BJOS *bjos = BJOS::getOS();
    if(bjos == nullptr){
        std::cout << "BJOS is not available..." << std::endl;
        return 0;
    }
    
    FlightController flight;
    bjos->getController("flight", &flight);
    if(!flight.isAvailable()){
        std::cout << "Failed to retrieve flight controller" << std::endl;
        return 0;
    }

    GripperController gripper;
    bjos->getController("gripper", &gripper);
    if (!gripper.isAvailable()) {
        std::cout << "Failed to retrieve gripper controller" << std::endl;
        return 0;
    }

	char ret;
	std::cout << "Interactive Offboard Tester!\n----------------------------" << std::endl;
	do {
		ret = std::cin.get();
		if (!Process::isActive()) ret = 'q';

        auto action = handle_input(ret);
        flight.setTargetCF(std::get<2>(action), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), std::get<0>(action), std::get<1>(action));

        switch (std::get<3>(action)) {
            case 1: //close gripper
                gripper.pickup();
                break;
            case 2: //open gripper
                gripper.release();
                break;
            case 0: //do nothing
            default:
                break; 
        }

	} while (ret != 'q');

	std::cout << "Byebye!";
}
