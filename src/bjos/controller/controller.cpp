#include "bjos/controller/controller.h"

#include "libs/log.h"
#include <boost/thread/thread.hpp>

using namespace bjos;
using namespace bjcomm;

void Controller::update_state(){
    Publisher state_pub("status");
    bool ret = state_pub.start();
    
    if(!ret){
        Log::error(_controller_name, "Cannot start the state publisher");
        _state_thrd_running = false;
        return;
    }
    
    while(_state_thrd_running){
        try{
            Message msg;
            std::string data = getState();
            msg.setData(data);
            msg.setType("controllers/"+_controller_name);
            
            state_pub.send(msg);
            
            boost::this_thread::sleep_for(boost::chrono::milliseconds(static_cast<int>(1000.0/_state_update_rate)));
        }catch(boost::thread_interrupted){
            //interrupt: we should stop now
            return;
        }
    }
}