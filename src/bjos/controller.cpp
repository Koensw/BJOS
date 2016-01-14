#include "bjos/controller.h"

#include "libs/log.h"
#include <boost/thread/thread.hpp>

using namespace bjos;
using namespace bjcomm;

void Controller::send_state_message(Message msg){
    //check if properly initialized
    if(!_state_pub) return;
        
    //send the message over the channel
    _state_pub->send(msg);
}
