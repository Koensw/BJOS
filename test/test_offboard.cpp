#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>

#include <chrono>
#include <thread>
#include <ctime>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"
#include <unistd.h>

#include "controllers/FlightController.h"

/*
 *  Test offboard
 */

using namespace bjos;

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
        
    Pose pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
    
    std::cout << "Press enter to start hold..." << std::endl;
    std::cin.get();
    Heading setp = Heading();
    setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should hold now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
    
    std::cout << "Press enter to go upwards..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vx = setp.velocity.vy = 0;
    setp.velocity.vz = 3;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should start moving now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;

    std::cout << "Press enter to hold again..." << std::endl;
    std::cin.get();
	double speed = 3.0;
	clock_t start_t, current_t;
	start_t = clock();
	while(speed>0)
	{
		setp = Heading();
		setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = speed;
		flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
		current_t = clock();
		double s = (double)(current_t - start_t) / CLOCKS_PER_SEC;
		speed = 3.0 - s*3.0; //in 1000ms is gaat hij naar 0;
		usleep(50000); // kan gebruikt worden om 50ms te slapen.
	}
    setp = Heading();
	setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should hold now..." << std::endl;

    std::cout << "Press enter to go forward..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vy = setp.velocity.vz = 0;
    setp.velocity.vx = 1.5;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should start moving now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;

    std::cout << "Press enter to hold again..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should hold now..." << std::endl;

    std::cout << "Press enter to go to the left..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vz = setp.velocity.vx = 0;
    setp.velocity.vy = 1.5;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should start moving now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
    
    std::cout << "Press enter to hold again..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should hold now..." << std::endl;

    std::cout << "Press enter to go backward..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vy = setp.velocity.vz = 0;
    setp.velocity.vx = -1.5;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should start moving now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;

    std::cout << "Press enter to hold again..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should hold now..." << std::endl;

    std::cout << "Press enter to go to the right..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vz = setp.velocity.vx = 0;
    setp.velocity.vy = -1.5;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should start moving now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
    
    std::cout << "Press enter to hold again..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should hold now..." << std::endl;

    std::cout << "Press enter to go downwards..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vx = setp.velocity.vy = 0;
    setp.velocity.vz = -0.5;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should start moving now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;

    std::cout << "Press enter to hold again..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);
    std::cout << "Should hold now...finishing" << std::endl;
    
    /*std::cout << "Press enter to go up 1 meter and hold there..." << std::endl;
    std::cin.get();
    Pose seps = Pose();
    seps.position.x = seps.position.y = seps.position.z = 0;
    flight.setTargetCF(SET_TARGET_POSITION, seps, Heading());
    std::cout << "Should hold now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
 
    std::cout << "Press enter to hold again..." << std::endl;
    std::cin.get();
    setp = Heading();
    setp.velocity.vx = setp.velocity.vy = setp.velocity.vz = 0;
    flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), setp);

    std::cout << "Should hold now... finishing" << std::endl;*/
}
