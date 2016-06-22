#include <iostream>

#include <thread>
#include <chrono>

#include "libs/i2c.h"
#include "libs/log.h"

#include "bjos/bjos.h"
#include "bjos/helpers/error.h"
#include "bjos/helpers/process.h"

#include "controllers/FlightController.h"
#include "controllers/GripperController.h"
#include "controllers/EyesController.h"

/*
 * Default loader for BJOS
 */

using namespace bjos;

FlightController *flight;
GripperController *gripper;
EyesController *eyes;

/* Initialize the OS */
void OSInit(){
    try{
        Log::info("DefaultLoader", "Starting loader %s", "test_default");
        //FIXME: fix state after segfault! 
        //if(BJOS::getState() != BJOS::UNINITIALIZED) Log::warn("DefaultLoader", "BJOS already running! Expecting invalid shutdown so continuing...");
        
        //init the OS
        BJOS::init();
        Process::installSignalHandler();
        BJOS *bjos = BJOS::getOS();

        //start wiring pi
        wiringPiSetupSys();
        
        //load the flight controller
        flight = new FlightController();
        bjos->initController(flight);

        //enable writing estimates to the pixhawk by default
        flight->toggleWriteEstimate(true);
        
        //load the gripper controller
        gripper = new GripperController(0x40, 0);
        bjos->initController(gripper);        
        
        //reset gripper
        gripper->gripperClosePWM(4095);
        
        //load eyes
        eyes = new EyesController(0x40, 2);
        bjos->initController(eyes);
        
        //set the eyes default off
        eyes->setEnabled(false);
    }catch(ControllerInitializationError &init_err){
        Log::fatal(init_err.getControllerName(), init_err.what());
        std::exit(0);
    }
}

void OSFinalize(){   
    BJOS *bjos = BJOS::getOS();
    
    //request shutdown for the OS clients
    bjos->shutdown();
    
    //wait for finalizing clients (or 5 seconds past)
    int time = 0;
    while((!gripper->canFinalize() || !flight->canFinalize())){
        if(++time >= 50) break;
        Log::info("DefaultLoader", "Waiting for %d clients to finish...", bjos->getControllerCount("gripper")-1 + bjos->getControllerCount("flight")-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    //delete pointers
    delete eyes;
    delete flight;
    delete gripper;
    
    //stop os
    BJOS::finalize();
    Log::info("DefaultLoader", "Successfull shutdown!");
}

int main(){
    // init
    OSInit();
    
    // wait until finished
    while(Process::isActive()){            
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    //finalize
    OSFinalize();
}
