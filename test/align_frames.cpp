#include <iostream>

#include <cstdlib>
#include <cstddef>
#include <cassert>
#include <utility>

#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"
#include "bjos/libs/log.h"

#include "controllers/FlightController.h"
#include "controllers/GripperController.h"

using namespace bjos;

int main(){
    Process::installSignalHandler();
    BJOS *bjos = BJOS::getOS();
    if(bjos == nullptr){
        Log::info("AlignFrame", "BJOS is not available...");
        return 0;
    }
    
    FlightController flight;
    bjos->getController("flight", &flight);
    if(!flight.isAvailable()){
        Log::info("AlignFrame", "Failed to retrieve flight controller");
        return 0;
    }
    
    if(flight.isWFDefined()){
        Log::info("AlignFrame", "Script should be run before sync is done");
        return 0;
    }
    
    //disable writing estimate so we can estimate both things apart
    flight.toggleWriteEstimate(false);
    
    Log::info("AlignFrame", "Waiting for sync...");
    while(!flight.isWFDefined() && Process::isActive()){
        usleep(10000);
    }
    if(!Process::isActive()) return 0;
    Log::info("AlignFrame", "WF is defined!");
    
    Log::info("AlignFrame", "Move in a rectangle and press enter in each corner... ");
    for(int i=0; i<4; ++i){
        std::cin.get();
        Log::info("AlignFrame", "New point saved");
        
        Eigen::Vector3d posEst = flight.getPositionEstimateWF();
        double yawEst = flight.getYawEstimateWF();
        std::cout << "estimate " << posEst[0] << " " << posEst[1] << " " << posEst[2] << " " << yawEst << std::endl;
        
        Eigen::Vector3d pos = flight.getPositionWF();
        double yaw = flight.getOrientationWF()[2];
        std::cout << "position " << pos[0] << " " << pos[1] << " " << pos[2] << " " << yaw << std::endl;
    }
    
    Log::info("AlignFrame", "Done!");
}