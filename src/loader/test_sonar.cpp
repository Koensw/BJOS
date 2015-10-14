#include <iostream>

#include <thread>
#include <chrono>

#include "i2c.h"
#include "log.h"

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"

#include "controllers/SonarController.h"
#include "controllers/sonar/DevantechSonarInterface.h"

/*
 * Test loader for BJOS
 */

using namespace bjos;

SonarController *sonar;

/* Initialize the OS */
void OSInit(){
    Log::info("Starting loader %s", "test_sonar");
    //if(BJOS::getState() != BJOS::UNINITIALIZED) Log::warn("BJOS already running... expecting incorrect shutdown so will continue.");
    
    //init the OS
    BJOS::init();
    Process::installSignalHandler();
    BJOS *bjos = BJOS::getOS();

    //start i2c
    I2C::start("/dev/i2c-1");
    
    sonar = new SonarController;
    unsigned char address[3] = {0x70, 0x71, 0x72};
    for(int i=0; i<3; ++i){
        SonarInterface *interface = new DevantechSonarInterface(address[i]);
        sonar->registerInterface(interface);
    }
    
    bjos->initController(sonar);
    sonar->setUpdateTime(1);
}

void OSFinalize(){   
    BJOS *bjos = BJOS::getOS();
    
    //request shutdown for the OS clients
    bjos->shutdown();
    
    //wait for finalizing clients
    while(!sonar->canFinalize()){
        Log::info("Waiting for %d clients to finish...", bjos->getControllerCount("sonar")-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //delete pointers
    delete sonar;
    
    //stop i2c
    I2C::stop();
    
    //stop os
    BJOS::finalize();
    Log::info("Successfull shutdown!");
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
