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

void mainProcess(){
    //reset BJOS (ALERT: this should only be done in the loader, because this will delete all other registered controllers ...)
    //BJOS::init();
    
    //most likely you want to install the signal handler to make sure that we can exit properly on SIGTERM etc.
    Process::installSignalHandler();
    
    //get a link to the BJOS
    BJOS *bjos = BJOS::getOS();
    
    //make an unitialized controller
	GripperController *gripper = new GripperController;
	
	wiringPiSetup(); // Initialize wiringPi -- using Broadcom pin numbers
	gripper->fd = wiringPiI2CSetup(0x40);
	pinMode(gripper->gripPin, PWM_OUTPUT); // Set PWM as PWM output
	pinMode(gripper->armPin, PWM_OUTPUT); // Set PWM as PWM output
	pinMode(gripper->ch7, INPUT);     // Set regular as INPUT
	pinMode(gripper->ch8, INPUT);      // Set regular as INPUT
	pullUpDnControl(gripper->ch7, PUD_UP); // Enable pull-up resistor --> weet niet zeker of dit nodig is !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
	pullUpDnControl(gripper->ch8, PUD_UP); // Enable pull-up resistor on 
	printf("CODE is running! Press CTRL+backslash to quit.\n");
	gripper->reset();
	gripper->setPWMFreq(1000);
    //initialize the controller
    bjos->initController(gripper);
	int pwm = 0;
    //do things until we got a request to stop the program
    while(Process::isActive()){
		printf("Lower arm(mm): ");
		std::cin >> pwm;
		printf("Lowering with %i mm.", pwm);
		gripper->lower_arm_mm(pwm);
		printf("Gripper position (0-4000): ");
		std::cin >> pwm;
		gripper->gripper_close_pwm(pwm);
        //std::cout << gripper->getInt() << std::endl;
        sleep(1);
    }
    
    //wait for other clients to unload (the main process should unload the latest)
    while(!gripper->canFinalize()){
        std::cout << "waiting for clients..." << std::endl;
        sleep(1);
    }
    
    //unload the controller
    delete gripper;
    
    //finish BJOS (ALERT: this should only be called in the loader that should remain active until all registered controllers are deregistered ...)
    BJOS::finalize();
}

int main(){
    //use threads here to simulate the two processes here
    std::thread mainThd(mainProcess);
        
    //wait to both are finished
    mainThd.join();
}