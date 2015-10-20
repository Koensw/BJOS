#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"

#include "test_controller.h"

/*
 *  General test file
 * 
 *  TODO: implement separate testcases
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
    
    FlightController test;
    bjos->getController("flight", &test);
    if(!test.isAvailable()){
        std::cout << "failed" << std::endl;
        return 0;
    }
        
    while(Process::isActive()){
        sleep(1);
    }
    
    //mutex.unlock();
}