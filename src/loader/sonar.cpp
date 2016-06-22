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
#include "controllers/EyesController.h"

#include "controllers/sonar/DevantechSonarInterface.h"
#include "controllers/sonar/MaxbotixSonarInterface.h"

/*
 * Test loader for BJOS
 */

using namespace bjos;

SonarController *sonar;
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
        
        //start i2c
        I2C::start("/dev/i2c-1");
        
        //load the sonar controller
        //TODO: separate the config from the loader
        sonar = new SonarController(false);
        unsigned char address[4] = {0x71, 0x72, 0x73, 0x74};
        double yaw[4] = {-1.1780972451, -0.39269908169, 0.39269908169, 1.1780972451};
        for(int i=0; i<4; ++i){
            SonarInterface *interface = new MaxbotixSonarInterface(address[i]);
            Pose pose;
            pose.position = Eigen::Vector3d::Zero();
            pose.orientation = Eigen::Vector3d(0, 0, yaw[i]);
            sonar->registerInterface(interface, pose, (i % 2));
        }
        bjos->initController(sonar);
        sonar->setUpdateTime(0.1);
                
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
    while(!sonar->canFinalize() || (!gripper->canFinalize() || !flight->canFinalize())){
        if(++time >= 50) break;
        Log::info("DefaultLoader", "Waiting for %d clients to finish...", bjos->getControllerCount("sonar")-1 + bjos->getControllerCount("gripper")-1 + bjos->getControllerCount("flight")-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    //delete pointers
    delete sonar;
    delete eyes;
    delete flight;
    delete gripper;
    
    //stop i2c
    I2C::stop();
    
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
