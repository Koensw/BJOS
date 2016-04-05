#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <string>
#include <utility>

#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"

#include "controllers/FlightController.h"

/*
 *  Write parameter
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

    int type;
    do {
        std::cout << "Arm = 1, disarm = 0, quit = -1: ";
        std::cin >> type;
        
        if(type != -1 && type != 0 && type != 1)
            std::cout << "That ain't -1, 0 or 1. Dumbass." << std::endl;
        else if(type != -1)
            flight.armDisarm((bool)type);

    } while(type != -1 && Process::isActive());

    std::cout << "Byebye!" << std::endl;
    
    return 1;
}
