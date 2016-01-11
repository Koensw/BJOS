#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>

#include <chrono>
#include <thread>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"

#include "controllers/FlightController.h"

/*
 *  Test offboard
 */

using namespace bjos;

float VEL = 0.5;
float TAKEOFF = 2.0;
float ZVEL = 0.5;
float YAWR = 0.2;

std::pair<uint16_t, Heading> handle_input(char c)
{
	static Heading setp_old;
    static uint16_t tm_old;
	Heading setp = Heading();
    uint16_t tm = SET_TARGET_VELOCITY;
	switch (c) {
	case 'w':
		std::cout << "Going forward" << std::endl;
		setp.velocity.vx = VEL;
		setp.velocity.vy = setp.velocity.vz = 0;
		break;

	case 's':
		std::cout << "Going backward" << std::endl;
		setp.velocity.vx = -VEL;
		setp.velocity.vy = setp.velocity.vz = 0;
		break;

	case 'a':
		std::cout << "Going left" << std::endl;
		setp.velocity.vy = VEL;
		setp.velocity.vx = setp.velocity.vz = 0;
		break;

	case 'd':
		std::cout << "Going right" << std::endl;
		setp.velocity.vy = -VEL;
		setp.velocity.vx = setp.velocity.vz = 0;
		break;

	case 'e':
		std::cout << "Turning left" << std::endl;
        setp = setp_old;
		setp.angular_velocity.vy = -YAWR;
        tm &= SET_TARGET_YAW_RATE;
		break;

	case 'r':
		std::cout << "Turning right" << std::endl;
        setp = setp_old;
		setp.angular_velocity.vy = YAWR;
        tm &= SET_TARGET_YAW_RATE;
		break;

	case 'o':
		std::cout << "Going upward" << std::endl;
		setp.velocity.vz = ZVEL;
		setp.velocity.vx = setp.velocity.vy = 0;
		break;

	case 'l':
		std::cout << "Landing" << std::endl;
		setp.velocity.vz = -ZVEL;
		setp.velocity.vx = setp.velocity.vy = 0;
        tm = SET_TARGET_LAND;
		break;

    case 't':
		std::cout << "Takeoff!" << std::endl;
		setp.velocity.vz = TAKEOFF;
		setp.velocity.vx = setp.velocity.vy = 0;
        tm = SET_TARGET_TAKEOFF;
		break;

	case 'q':
		std::cout << "Quitting... :(" << std::endl;
        setp.velocity.vz = -ZVEL;
		setp.velocity.vx = setp.velocity.vy = 0;
		break;

	case '\n':
		setp = setp_old;
        tm = tm_old;
		break;

	case 'h':
	default:
		std::cout << "Holding..." << std::endl;
		setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
		break;
	}

	setp_old = setp;
    tm_old = tm;
	return std::pair<uint16_t, Heading>(tm, setp);
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

	char ret;
	Heading action;
	std::cout << "Interactive Offboard Tester!\n----------------------------" << std::endl;
	do {
		ret = std::cin.get();
		if (!Process::isActive()) ret = 'q';
		std::pair<uint16_t, Heading> action = handle_input(ret);
		flight.setTargetCF(action.first, Pose(), action.second);
	} while (ret != 'q');

	std::cout << "Byebye!";
}
