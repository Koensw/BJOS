#include "bjos/helpers/process.h"

#include "bjos/bjos.h"

#include <atomic>
#include <csignal>

using namespace bjos;

std::atomic_bool Process::_active(true);

void (*Process::getSignalHandler())(int){
    return signal_handler;
}
void Process::installSignalHandler(){
    struct sigaction act;
    act.sa_handler = signal_handler;
    sigfillset(&act.sa_mask);
    act.sa_flags = 0;
    
    sigaction(SIGINT, &act, NULL);
    sigaction(SIGTERM, &act, NULL);
    //sigaction(SIGSTOP, &act, NULL);
    sigaction(SIGHUP, &act, NULL); //TODO: reload something on sighup ?
    sigaction(SIGABRT, &act, NULL);
}

void Process::stop(){
    _active = false;
}

bool Process::isActive(){
    return _active && BJOS::getState() == BJOS::ACTIVE;
}

void Process::signal_handler(int){
    _active = false;
}