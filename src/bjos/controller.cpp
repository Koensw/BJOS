#include "bjos/controller.h"

#include "libs/log.h"
#include <boost/thread/thread.hpp>

using namespace bjos;
using namespace bjcomm;

void Controller::send_state_message(Message msg){
    //check if properly initialized
    if(!_state_pub) return;
        
    //convert the message to a global state message
    Message final_msg;
    msg.setType(_controller_name);
    msg.setData(msg.getType()+" "+msg.getData());
    
    //send the message over the channel
    _state_pub->send(msg);
}