#include <iostream>

#include <thread>
#include <chrono>

#include "libs/i2c.h"
#include "libs/log.h"

#include "bjos/bjos.h"
#include "bjos/helpers/error.h"
#include "bjos/helpers/process.h"

#include "controllers/SonarController.h"
#include "controllers/sonar/DevantechSonarInterface.h"
#include "controllers/sonar/MaxbotixSonarInterface.h"

/*
 * Test loader for BJOS
 */

using namespace bjos;

SonarController *sonar;

/* Initialize the OS */
void OSInit(){
    try{
        Log::info("SonarLoader", "Starting loader %s", "test_sonar");
        //if(BJOS::getState() != BJOS::UNINITIALIZED) Log::warn("BJOS already running... expecting incorrect shutdown so will continue.");
        
        //init the OS
        BJOS::init();
        Process::installSignalHandler();
        BJOS *bjos = BJOS::getOS();

        //start i2c
        I2C::start("/dev/i2c-1");
        
        sonar = new SonarController(false);
        unsigned char address[4] = {0x71, 0x72, 0x73, 0x74};
        double yaw[4] = {0, 0, 0, 0};
        for(int i=0; i<4; ++i){
            SonarInterface *interface = new MaxbotixSonarInterface(address[i]);
            Pose pose;
            pose.position = Eigen::Vector3d::Zero();
            pose.orientation = Eigen::Vector3d(0, 0, yaw[i]);
            sonar->registerInterface(interface, pose, (i % 2));
        }
        
        bjos->initController(sonar);
        sonar->setUpdateTime(0.1);
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
    while(!sonar->canFinalize()){
        Log::info("SonarLoader", "Waiting for %d clients to finish...", bjos->getControllerCount("sonar")-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //delete pointers
    delete sonar;
    
    //stop i2c
    I2C::stop();
    
    //stop os
    BJOS::finalize();
    Log::info("SonarLoader", "Successfull shutdown!");
}

int main(){
    // init
    OSInit();
    
    // wait until finished
    while(Process::isActive()){            
	Log::info("SonarLoader", "DISTANCES: ");
	std::vector<SonarData> data = sonar->getData();
	for(size_t i=0; i<data.size(); ++i){
		Log::info("SonarLoader", "%f", data[i].distance);
	}

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    //finalize
    OSFinalize();
}
