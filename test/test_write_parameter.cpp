#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
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
    float number, value;
    std::cin >> number;
    std::cout << "Value: ";
    std::cin >> value;

    if (flight.writeParameter(number, value))
        std::cout << "Succesfully wrote parameter " << number << " with value " << value << "!" << std::endl;
    else
        std::cout << "Oops! The write failed :(" << std::endl;

    std::cout << "Byebye!";
}
