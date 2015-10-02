#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>

#include "bjos/bjos.h"
#include "test_controller.h"

/*
 *  General test file
 * 
 *  TODO: implement separate testcases
 */

using namespace boost::interprocess;

int main(){
    BJOS::installSignalHandler();
    BJOS *bjos = BJOS::getOS();
    
    //named_mutex mutex;
    //named_mutex mutex(open_or_create, "testtestest", 644);
    //mutex.unlock();
    //scoped_lock<named_mutex> lock(mutex);
    
    std::cout << "lock succeeded" << std::endl;
    
    TestController test;
    bjos->getController("flight", &test);
    if(!test.isAvailable()){
        std::cout << "failed" << std::endl;
        return 0;
    }
    *test._id = 2;
    
    std::cout << test.getID() << std::endl;
        
    sleep(10);
    
    //mutex.unlock();
}