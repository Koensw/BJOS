#include <iostream>

#include <thread>
#include <chrono>

#include "libs/i2c.h"
#include "libs/log.h"

#include "bjos/bjos.h"
#include "bjos/helpers/error.h"
#include "bjos/helpers/process.h"

#include "controllers/SonarController.h"
#include "controllers/FlightController.h"
#include "controllers/GripperController.h"

#include "controllers/sonar/DevantechSonarInterface.h"

/*
 * Test loader for BJOS
 */

using namespace bjos;

SonarController *sonar;
FlightController *flight;
GripperController *gripper;

/* Initialize the OS */
void OSInit(){
    try{
        Log::info("default_loader", "Starting loader %s", "test_sonar");
        //if(BJOS::getState() != BJOS::UNINITIALIZED) Log::warn("BJOS already running... expecting incorrect shutdown so will continue.");
        
        //init the OS
        BJOS::init();
        Process::installSignalHandler();
        BJOS *bjos = BJOS::getOS();

        //start i2c
        I2C::start("/dev/i2c-1");
        
        //load the sonar controller
        sonar = new SonarController(true);
        unsigned char address[3] = {0x70, 0x71, 0x72};
        //double yaw[3] = {-1.57079632679, 3.14159265, 1.57079632679};
        double yaw[3] = {0, 1.57079632679, 3.14159265};
        for(int i=0; i<3; ++i){
            SonarInterface *interface = new DevantechSonarInterface(address[i]);
            Pose pose;
            pose.orientation.y = yaw[i];
            //sonar->registerInterface(interface, pose, (i == 0));
        }
        
        bjos->initController(sonar);
        sonar->setUpdateTime(0.1);
        
        //load the flight controller
        flight = new FlightController();
        bjos->initController(flight);

        //load the gripper controller
        gripper = new GripperController();
        bjos->initController(gripper);        
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
    while(!sonar->canFinalize() || !flight->canFinalize() || !gripper->canFinalize()){
        Log::info("default_loader", "Waiting for %d clients to finish...", bjos->getControllerCount("sonar")+bjos->getControllerCount("flight")+bjos->getControllerCount("gripper")-3);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //delete pointers
    delete sonar;
    delete flight;
    delete gripper;
    
    //stop i2c
    I2C::stop();
    
    //stop os
    BJOS::finalize();
    Log::info("default_loader", "Successfull shutdown!");
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
