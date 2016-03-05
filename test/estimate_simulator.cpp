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
#include "libs/log.h"

#include "controllers/FlightController.h"
#include "controllers/GripperController.h"

using namespace bjos;

int main(){
    BJOS *bjos = BJOS::getOS();
    if(bjos == nullptr){
        std::cout << "BJOS not available" << std::endl;
        return 0;
    }
    
    FlightController flight;
    if(!bjos->getController("flight", &flight)){
        std::cout << "Current loader has no FlightController" << std::endl;
        return 0;
    }
    
    Process::installSignalHandler();
    flight.toggleWriteEstimate(true);
    flight.syncVision(Eigen::Vector3d(0,0,0), 0);
    while(Process::isActive()){
        double x, y, z, yaw;
        std::cin >> x >> y >> z >> yaw;
        
        int cnt = 0;
        while(Process::isActive() && cnt < 1000){ 
            flight.setYawEstimateWF(yaw);
            flight.setPositionEstimateWF(Eigen::Vector3d(x,y,z));
            
            Eigen::Vector3d est = flight.getPositionEstimateWF();
            double eyaw = flight.getYawEstimateWF();
            
            Eigen::Vector3d pos = flight.getPositionWF();
            double pyaw = flight.getOrientationWF().z();
            
            std::cout << "estimate (" << est.x() << "," << est.y() << "," << est.z() << "," << eyaw << ") - pos(" << pos.x() << "," << pos.y() << "," << pos.z() << "," << pyaw << std::endl;
            
            usleep(10000);
            ++cnt;
        }
    }
}