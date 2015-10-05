#ifndef _BLUEJAY_TEST_H
#define _BLUEJAY_TEST_H

#include <iostream>
#include <mutex>

#include "bjos/controller/controller.h"

/*
 * Controller that is used for testing purposes
 */

//WARNING: dont put this in header
using namespace bjos;

class TestController: public bjos::Controller{
public:
    TestController(): _id(0) {}
    ~TestController(){
        Controller::finalize<int>();
    }
    
    int getID(){
        std::lock_guard<BJOS::Mutex> lock(*mutex);
        int id = *_id;
        return id;
    }
    
    int *_id;
private:
    /* init module is only called for the main node */
    void init(BJOS *bjos){
        Controller::init(bjos, "flight", _id);
        //TODO: what shall we do with return value?
        //check if init succeeded
    }
    /* load node is called for all childeren */
    void load(BJOS *bjos){
        Controller::load(bjos, "flight", _id);
    }
};

#endif