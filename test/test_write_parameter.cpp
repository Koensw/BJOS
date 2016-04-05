#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <string>
#include <utility>
#include <cmath>

#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"

#include "controllers/FlightController.h"
#include "controllers/GripperController.h"
#include "controllers/EyesController.h"

/*
 *  Write parameter
 */

using namespace bjos;

int main(){
    Process::installSignalHandler();
    //BJOS::init();
    BJOS *bjos = BJOS::getOS();
    if(bjos == nullptr){
        std::cout << "BJOS is not available..." << std::endl;
        return 0;
    }
    
    FlightController flight;
    bjos->getController("flight", &flight);
    if(!flight.isAvailable()){
        std::cout << "Failed to retrieve flight controller" << std::endl;
        return 0;
    }               
  
    while(Process::isActive()){
        std::cout << "Write a parameter" << std::endl;
        
        std::string param;
        std::cout << "Name: ";
        std::cin >> param;    
        
        std::pair<float, uint8_t> current = flight.readParameter(param.c_str());
        if(std::isnan(current.first)){
            std::cout << "Parameter not received, probably it does not exist!" << std::endl;
            continue;
        }
                
        std::cout << "Value ";
        switch(current.second){
            case MAV_PARAM_TYPE_INT32:
                std::cout << *reinterpret_cast<int*>(&current.first) << std::endl;
                break;
            case MAV_PARAM_TYPE_REAL32:
                std::cout << current.first << std::endl;
                break;
            default:
                std::cout << "??" << std::endl;
                std::cout << "Parameter type not yet supported!" << std::endl;
                return 1;
        }
        
        std::cout << "New: ";
        
        int val_int32;
        float val_float;
        
        int ret = 0;
        switch(current.second){
            case MAV_PARAM_TYPE_INT32:
                std::cin >> val_int32;
                if(std::cin.eof()) return 1;

                ret = flight.writeParameter(param.c_str(), *reinterpret_cast<float*>(&val_int32), MAV_PARAM_TYPE_INT32);
                break;
            case MAV_PARAM_TYPE_REAL32:
                std::cin >> val_float;
                
                if(std::cin.eof()) return 1;
                ret = flight.writeParameter(param.c_str(), val_float, MAV_PARAM_TYPE_REAL32);
                break;
            default:
                return 1;
        }
        if (ret)
            std::cout << "Succesfully wrote parameter " << param << "!" << std::endl;
        else{
            std::cout << "Oops! The write failed :(" << std::endl;
            return 1;
        }
    }
}
