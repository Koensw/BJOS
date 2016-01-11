#include <iostream>
#include <sstream>

#include <bjcomm/subscriber.h>
#include <bjcomm/message.h>
#include <bjcomm/poller.h>

using namespace bjcomm;

/* logger receiving client */
int main(){
    Subscriber sub("debug");
    bool ret = sub.start();
    if(!ret) return 1;
        
    Poller poller;
    int SUBSCRIBER = poller.add(&sub);
    
    while(true){
        poller.poll();        
        
        if(poller.hasMsg(SUBSCRIBER)){
            Message msg;
            msg = sub.receive();
    
            if(msg.getType() == "info"){
                std::cout << "[INFO] " << msg.getData() << std::endl;
            }else if(msg.getType() == "warn"){
                std::cout << "[WARN] " << msg.getData() << std::endl;
            }else if(msg.getType() == "error"){
                std::cout << "[ERROR] " << msg.getData() << std::endl;
            }else if(msg.getType() == "fatal"){
                std::cout << "[FATAL] " << msg.getData() << std::endl;
            }else{
                std::cout << "INCORRECT MESSAGE" << std::endl;
            }
        }else{
            std::cout << "INCORRECT POLL" << std::endl;
        }
    }
}
