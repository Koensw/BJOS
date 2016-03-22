#include <iostream>

#include <thread>
#include <chrono>


#include "bjos/bjos.h"
#include "bjos/helpers/error.h"
#include "bjos/helpers/process.h"

#include "controllers/RGBEyesController.h"

/*
 * Gripper loader for BJOS
 */

using namespace bjos;

RGBEyesController *rgbeyes;

/* Initialize the OS */
void OSInit(){
    try{
        Log::info("RGBeyesLoader", "Starting loader %s", "test_RGBeyes");
        //if(BJOS::getState() != BJOS::UNINITIALIZED) Log::warn("BJOS already running... expecting incorrect shutdown so will continue.");
        
        //init the OS
        BJOS::init();
        Process::installSignalHandler();
        BJOS *bjos = BJOS::getOS();
        
        rgbeyes = new RGBEyesController();
        bjos->initController(rgbeyes);
        
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
    while(!rgbeyes->canFinalize()){
        Log::info("RGBEyesLoader", "Waiting for %d clients to finish...", bjos->getControllerCount("rgbeyes")-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //delete pointers
    delete rgbeyes;
    
    //stop os
    BJOS::finalize();
    Log::info("RGBeyesLoader", "Successfull shutdown!");
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
