#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <string>
#include <utility>
#include <cmath>
#include <iomanip>

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

uint64_t get_time_msec(clockid_t clk_id)
{
    struct timespec _time_stamp;
    clock_gettime(clk_id, &_time_stamp);
    return _time_stamp.tv_sec * 1000ULL + _time_stamp.tv_nsec / 1000000ULL;
}

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
  
    uint64_t start = get_time_msec(CLOCK_MONOTONIC);
    while(Process::isActive()){
        uint64_t cur = get_time_msec(CLOCK_MONOTONIC);
        std::cout << "[" << std::fixed << std::setprecision(2) << (cur - start)/1e3 << "] BATT: " << flight.getBatteryPercentage() << std::endl;
        sleep(1);
    }
}
