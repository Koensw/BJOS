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

#include "controllers/RGBEyesController.h"

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

	RGBEyesController RGBeyes;
    printf("loading..\n");
	bjos->getController("RGBeyes", &RGBeyes);
    if(!RGBeyes.isAvailable()) {
        std::cout << "Failed to retrieve RGBEyes controller" << std::endl;
        return 0;
    }
    
	printf("CODE is running! Press CTRL+C to quit.\n");

	int red = 0;
	int green = 0;
	int blue = 0;
    //read user input until ctrl+c
    while(Process::isActive()){
		std::cout << "RGB color (0-255): " << std::endl;
		std::cin >> red;
		std::cin >> green;
		std::cin >> blue;
		RGBeyes.rgbsollid(red, green, blue);
    }
}