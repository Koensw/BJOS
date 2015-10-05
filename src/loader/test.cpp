#include <iostream>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>
#include <csignal> 

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"
#include "../../test/test_controller.h"

/*
 * Test loader for BJOS
 */

using namespace bjos;

TestController *test;

/* Initialize the OS */
void OSInit(){
    BJOS::init();
    Process::installSignalHandler();    
    BJOS *bjos = BJOS::getOS();

    test = new TestController;
    bjos->initController(test);
    *test->_id = 1;
    std::cout << "(1) " << test->getID() << std::endl;
}

/* Do several things just for testing ... */
void OSTest(){
    BJOS *bjos = BJOS::getOS();
    
    TestController test;
    bool c = bjos->getController("flight", &test);
    if(c == false) return;
    
    int cnt = 0;
    while(Process::isActive() && ++cnt < 5){
        std::cout << "(2) " << test.getID() << std::endl;
        sleep(1);
    }
}

void OSFinalize(){   
    //shutdown the OS client
    BJOS::getOS()->shutdown();
    
    while(!test->canFinalize()){
        std::cout << "waiting for clients" << std::endl;
        sleep(1);
    }
    delete test;

    BJOS::finalize();
}

int main(){
    // init
    OSInit();
    
    //test
    OSTest();
    
    OSFinalize();
}