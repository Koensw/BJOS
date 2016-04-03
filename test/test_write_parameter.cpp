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
  
    std::cout << "Write parameter!\nNumber: ";
    std::string param;
    float value;
    std::cin >> param;
    std::cout << "Value: ";
    std::cin >> value;

    if (flight.writeParameter(param.c_str(), value, MAV_PARAM_TYPE_UINT8))
        std::cout << "Succesfully wrote parameter " << param << " with value " << value << "!" << std::endl;
    else
        std::cout << "Oops! The write failed :(" << std::endl;

    std::cout << "Byebye!";
}
