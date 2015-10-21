#include <iostream>

#include <thread>
#include <chrono>

#include "i2c.h"
#include "log.h"

#include "bjos/bjos.h"
#include "bjos/helpers/error.h"
#include "bjos/helpers/process.h"

#include "controllers/FlightController.h"
#include <mavlink/v1.0/common/mavlink.h>

/*
 * Test loader for BJOS
 */

using namespace bjos;

FlightController *flight;

/* Initialize the OS */
void OSInit(){
    try{
        Log::info("flight_loader", "Starting loader %s", "test_flight");
        //if(BJOS::getState() != BJOS::UNINITIALIZED) Log::warn("BJOS already running... expecting incorrect shutdown so will continue.");
        
        //init the OS
        BJOS::init();
        Process::installSignalHandler();
        BJOS *bjos = BJOS::getOS();

        flight = new FlightController();
        
        bjos->initController(flight);
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
    while(!flight->canFinalize()){
        Log::info("flight_loader", "Waiting for %d clients to finish...", bjos->getControllerCount("flight")-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //delete pointers
    delete flight;
    
    //stop os
    BJOS::finalize();
    Log::info("flight_loader", "Successfull shutdown!");
}

int main(){
    // init
    OSInit();
    
    // wait until finished
    while(Process::isActive()){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "pre OSFinalize()" << std::endl;
    //finalize
    OSFinalize();
}
