#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <string>
#include <utility>

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
  
    std::cout << "Type (1 = int32, 2 = float): ";
    int type;
    std::cin >> type;
    std::string param;
    std::cout << "Name: ";
    std::cin >> param;
    std::cout << "Value: ";
    
    int val_int32;
    float val_float;
    
    int ret = 0;
    switch(type){
        case 1:
            std::cin >> val_int32;
            ret = flight.writeParameter(param.c_str(), *reinterpret_cast<float*>(&val_int32), MAV_PARAM_TYPE_INT32);
            break;
        case 2:
            std::cin >> val_float;
            ret = flight.writeParameter(param.c_str(), val_float, MAV_PARAM_TYPE_REAL32);
            break;
        default:
            ret = false;
            return 1;
    }
    
    if (ret)
        std::cout << "Succesfully wrote parameter " << param << "!" << std::endl;
    else
        std::cout << "Oops! The write failed :(" << std::endl;
}
