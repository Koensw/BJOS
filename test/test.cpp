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
 *  General test file
 * 
 *  TODO: implement separate testcases
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
    
    FlightController test;
    bjos->getController("flight", &test);
    if(!test.isAvailable()){
        std::cout << "failed" << std::endl;
        return 0;
    }
        
    
    Heading head;
    head.velocity.vz = -1;
    test.setTarget(MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY, Pose(), head);
    
    while(Process::isActive()){
        Pose pose = test.getPose();
        std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //sleep(1);
    }
    
    //mutex.unlock();
}
