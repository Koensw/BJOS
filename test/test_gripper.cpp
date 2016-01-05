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

	wiringPiSetup(); // Initialize wiringPi -- using Broadcom pin numbers
	gripper.fd = wiringPiI2CSetup(0x40);
	pinMode(gripper.gripPin, PWM_OUTPUT); // Set PWM as PWM output
	pinMode(gripper.armPin, PWM_OUTPUT); // Set PWM as PWM output
	pinMode(gripper.ch7, INPUT);     // Set regular as INPUT
	pinMode(gripper.ch8, INPUT);      // Set regular as INPUT
	pullUpDnControl(gripper.ch7, PUD_UP); // Enable pull-up resistor -. weet niet zeker of dit nodig is !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
	pullUpDnControl(gripper.ch8, PUD_UP); // Enable pull-up resistor on 
	printf("CODE is running! Press CTRL+backslash to quit.\n");
	gripper.reset();
	gripper.setPWMFreq(1000);

	int pwm = 0;
    //do things until we got a request to stop the program
    while(Process::isActive()){
		std::cout << "Gripper position (0-4000): " << std::endl;
		std::cin >> pwm;
		gripper.gripper_close_pwm(pwm);
    }
}