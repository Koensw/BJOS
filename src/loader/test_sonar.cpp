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

SonarInterface *interface;

SonarController *sonar;

/* Initialize the OS */
void OSInit(){
    Log::info("Starting loader %s", "test_sonar");
    if(BJOS::getState() != BJOS::UNINITIALIZED) Log::warn("BJOS already running... expecting incorrect shutdown so will continue.");
    
    //init the OS
    BJOS::init();
    Process::installSignalHandler();    
    BJOS *bjos = BJOS::getOS();

    //start i2c
    I2C::start("/dev/i2c-1");
    
    interface = new DevantechSonarInterface(0x70);
    
    sonar = new SonarController;
    sonar->registerInterface(interface);
    
    bjos->initController(sonar);
    sonar->setUpdateTime(1);
}

void OSFinalize(){   
    //request shutdown for the OS clients
    BJOS::getOS()->shutdown();
    
    //wait for finalizing clients
    while(!sonar->canFinalize()){
        Log::info("Waiting for clients to finish...");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //delete pointers
    delete sonar;
    delete interface;
    
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
        Log::info("Update ...");
	std::vector<SonarData> data = sonar->getData();
        for(size_t i=0; i<data.size(); ++i){
            Log::info(" %d: %f", i, data[i].distance);
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    //finalize
    OSFinalize();
}
