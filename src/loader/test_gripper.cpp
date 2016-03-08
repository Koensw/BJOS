#include <iostream>

#include <thread>
#include <chrono>

#include "libs/i2c.h"
#include "libs/log.h"

#include "bjos/bjos.h"
#include "bjos/helpers/error.h"
#include "bjos/helpers/process.h"

#include "controllers/GripperController.h"

/*
 * Gripper loader for BJOS
 */

using namespace bjos;

GripperController *gripper;

/* Initialize the OS */
void OSInit(){
    try{
        Log::info("GripperLoader", "Starting loader %s", "test_gripper");
        //if(BJOS::getState() != BJOS::UNINITIALIZED) Log::warn("BJOS already running... expecting incorrect shutdown so will continue.");
        
        //init the OS
        BJOS::init();
        Process::installSignalHandler();
        BJOS *bjos = BJOS::getOS();

        //start the wiring pi library
        wiringPiSetupSys();
        
        gripper = new GripperController(0x40, 1);
        bjos->initController(gripper);
        
        gripper->gripperClosePWM(0);
    }catch(ControllerInitializationError &init_err){
        Log::fatal(init_err.getControllerName(), init_err.what());
        std::exit(0);
    }
}

void OSFinalize(){   
    BJOS *bjos = BJOS::getOS();
    
    //request shutdown for the OS clients
    bjos->shutdown();
    
    //wait for finalizing clients
    while(!gripper->canFinalize()){
        Log::info("GripperLoader", "Waiting for %d clients to finish...", bjos->getControllerCount("gripper")-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //delete pointers
    delete gripper;
    
    //stop os
    BJOS::finalize();
    Log::info("GripperLoader", "Successfull shutdown!");
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
