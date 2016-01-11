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

#include "controllers/GripperController.h"

/*
 *  Test gripper
 */

using namespace bjos;

int main(){    
    Process::installSignalHandler();
    
    //get a link to the BJOS
    BJOS *bjos = BJOS::getOS();
    if(bjos == nullptr){
        std::cout << "BJOS is not available..." << std::endl;
        return 0;
    }

	GripperController gripper;
	bjos->getController("gripper", &gripper);
    if(!gripper.isAvailable()){
        std::cout << "Failed to retrieve gripper controller" << std::endl;
        return 0;
    }
    
	printf("CODE is running! Press CTRL+C to quit.\n");

	int pwm = 0;
    //read user input until ctrl+c
    while(Process::isActive()){
		std::cout << "Gripper position (0-4095): " << std::endl;
		std::cin >> pwm;
		gripper.gripperClosePWM(pwm);
    }
}