#include <cstdlib>
#include <atomic>
#include <csignal>
#include <mutex>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/map.hpp>

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/offset_ptr.hpp>

#include "bjos/bjos.h"
#include "bjos/controller/controller.h"

using namespace boost::interprocess;

/*
 * Implementation of the BJOS
 */

std::atomic_bool _running;

const boost::interprocess::open_only_t BJOS::mutex_open_only = boost::interprocess::open_only;

//TODO: can we find out that the shared memory is actually old ?
void BJOS::init(){
    _running = false;
    
    //remove shared memory and mutex that still exists because of earlier sudden stops
    shared_memory_object::remove("BJOS_SHARED_MEMORY");
    named_mutex::remove("BJOS_MUTEX");
}
void BJOS::finalize(){
    if(_running){
        //check if properly closed everything
        BJOS *bjos = BJOS::getOS();
        
        if(!bjos->_controller_map->empty()){
            bjos->fatal_error("Finishing while not everything is properly closed");
        }
    }
    
    //remove shared memory
    shared_memory_object::remove("BJOS_SHARED_MEMORY");
    named_mutex::remove("BJOS_MUTEX");
}

void BJOS::handleSignal(int){
    //set the running flag to false
    _running = false;
}

//TODO: use sigaction instead of std::signal
void (*BJOS::installSignalHandler())(int){
    
    std::signal(SIGINT, BJOS::handleSignal);
    std::signal(SIGQUIT, BJOS::handleSignal);
    std::signal(SIGABRT, BJOS::handleSignal);
    std::signal(SIGTERM, BJOS::handleSignal);
    return BJOS::handleSignal;
}

BJOS *BJOS::getOS(){
    //NOTE: only thread safe in C++11
    static BJOS instance;
    _running = true;
    return &instance;
}

//ALERT: this opens the memory segment or creates it; we should be absolutely be sure that is properly cleaned before using the first BJOS instance!!!
//WARNING: we should tweak the size...
BJOS::BJOS(): 
    _memory_segment(open_or_create,"BJOS_SHARED_MEMORY", 65536),
    _controller_map(0),
    _mutex(open_or_create, "BJOS_MUTEX")
{
    ControllerAllocator Controller(_memory_segment.get_segment_manager());
    
    std::pair<ControllerMap*, managed_shared_memory::size_type> _check_controller = _memory_segment.find<ControllerMap>("bjos_controller_map");
    if(_check_controller.first == 0){
        //we are the first loader...
        
        //load the map
        _controller_map = _memory_segment.construct<ControllerMap>("bjos_controller_map")      //object name
                                (std::less<interprocess::char_string>(), Controller);    
    }else _controller_map = _check_controller.first;
}

void BJOS::unloadController(std::string name){
    //lock mutex
    std::lock_guard<BJOS::Mutex> lock(_mutex);
    
    //convert to shared memory string
    interprocess::char_allocator string_allocator(_memory_segment.get_segment_manager());
    interprocess::char_string ipc_name(name.c_str(), string_allocator);
    ControllerMap::iterator iter = _controller_map->find(ipc_name);
    if(iter != _controller_map->end() && iter->second){
        //decrease the count and check if main is still running
        --iter->second;
        if(iter->second <= 0){
            //ALERT: unloading after master node or multiple times
            fatal_error("Unload a controller multiple times or unloading a node that is already deregistered!");
        }
    }else{
        //ALERT: unloading an controller that is never loaded... this should be impossible
        fatal_error("Trying to unload an controller that is not loaded!");
    }
}

void BJOS::initController(Controller *Controller){
    //init the Controller
    Controller->init(this);
}

bool BJOS::getController(std::string name, Controller *Controller){    
    //convert to shared memory string
    interprocess::char_allocator string_allocator(_memory_segment.get_segment_manager());
    interprocess::char_string ipc_name(name.c_str(), string_allocator);
    
    ControllerMap::iterator iter = _controller_map->find(ipc_name);
    
    if(iter == _controller_map->end()){
        //Controller not loaded
        return false;
    }else{
        //Controller is available, load its shared memory now
        Controller->load(this);
        return true;
    }
}

bool BJOS::isRunning(){
    return _running;
}

int BJOS::getControllerCount(std::string name){
    //lock mutex
    std::lock_guard<Mutex> lock(_mutex);
    
    //convert to shared memory string
    interprocess::char_allocator string_allocator(_memory_segment.get_segment_manager());
    interprocess::char_string ipc_name(name.c_str(), string_allocator);
    ControllerMap::iterator iter = _controller_map->find(ipc_name);
    
    if(iter != _controller_map->end() && iter->second){
        return iter->second;
    }else return 0;
}