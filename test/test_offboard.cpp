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
    Pose setp = Pose();
    setp.position.x = setp.position.y = setp.position.z = 0;
    flight.setTargetCF(SET_TARGET_POSITION, setp, Heading());
    std::cout << "Should hold now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
    
    std::cout << "Press enter to go up 1 meter..." << std::endl;
    std::cin.get();
    setp = flight.getPoseNED();
    setp.position.x = setp.position.y = 0;
    setp.position.z = 0.5;
    flight.setTargetCF(SET_TARGET_POSITION, setp, Heading());
    std::cout << "Should go up half a meter now..." << std::endl;
    pose = flight.getPoseNED();
    std::cout << "Current position is: " << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
    
    std::cout << "Finishing" << std::endl;
}
