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

float VEL = 1.5;

Heading handle_input(char c)
{
	static Heading setp_old;
	Heading setp = Heading();
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
		setp.velocity.vy = -VEL;
		setp.velocity.vx = setp.velocity.vz = 0;
		break;

	case 'd':
		std::cout << "Going right" << std::endl;
		setp.velocity.vy = VEL;
		setp.velocity.vx = setp.velocity.vz = 0;
		break;

	case 'o':
		std::cout << "Going upward" << std::endl;
		setp.velocity.vz = VEL;
		setp.velocity.vx = setp.velocity.vy = 0;
		break;

	case 'l':
		std::cout << "Going downward" << std::endl;
		setp.velocity.vz = -VEL;
		setp.velocity.vx = setp.velocity.vy = 0;
		break;

	case 'q':
		std::cout << "Quitting... :(" << std::endl;
		setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
		break;

	case '\n':
		setp = setp_old;
		break;

	case 'h':
	default:
		std::cout << "Holding..." << std::endl;
		setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
		break;
	}

	setp_old = setp;
	return setp;
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
	std::cout << "Interactive Offboard Tester!\n----------------------------" << std::endl;
	do {
		ret = std::cin.get();
		if (!Process::isActive()) ret = 'q';
		flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), handle_input(ret));
	} while (ret != 'q');

	std::cout << "Byebye!";
}
