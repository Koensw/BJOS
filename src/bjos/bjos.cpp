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
#include "bjos/helpers/error.h"

using namespace boost::interprocess;
using namespace bjos;

/*
 * Implementation of the BJOS
 */

const boost::interprocess::open_only_t BJOS::mutex_open_only = boost::interprocess::open_only;

//TODO: can we find out that the shared memory is actually old ?
void BJOS::init(){    
    //TODO: use a check loaded file on the filesystem!
    
    //remove shared memory and mutex that still exists because of earlier sudden stops
    shared_memory_object::remove("BJOS_SHARED_MEMORY");
    named_mutex::remove("BJOS_MUTEX");
    
    //acquire the bjos mutex and lock it
    Mutex mutex(create_only, "BJOS_MUTEX");
    std::lock_guard<BJOS::Mutex> lock(mutex);
    
    //set the state to available
    managed_shared_memory shared_memory(create_only, "BJOS_SHARED_MEMORY", BJOS_SHARED_MEM_SIZE);
    State *current = shared_memory.construct<State>("bjos_state")();
    *current = ACTIVE;
}
void BJOS::finalize(){        
    const char *error = nullptr;
    //check if properly closed everything (if we are even active...)
    BJOS *bjos = BJOS::getOS();
    if(bjos == nullptr) {
        throw BJOSError("Finalizing OS that is not even initialized");
        return;
    }
    
    //acquire the bjos mutex and lock it
    Mutex mutex(open_only, "BJOS_MUTEX");
    std::lock_guard<BJOS::Mutex> lock(mutex);
    
    if(!bjos->_controller_map->empty()) error = "Finishing while not everything is properly closed";
    
    //remove shared memory
    shared_memory_object::remove("BJOS_SHARED_MEMORY");
    named_mutex::remove("BJOS_MUTEX");
    
    //print an error if someone went wrong
    if(error != nullptr) throw BJOSError(error);
}

BJOS::State BJOS::getState(){
    try{
        //check shared memory available
        managed_shared_memory check_shared_memory(open_only,"BJOS_SHARED_MEMORY");
        
        //acquire the bjos mutex and lock it
        Mutex mutex(open_only, "BJOS_MUTEX");
        std::lock_guard<BJOS::Mutex> lock(mutex);
        
        //return the state pointer
        State *current = check_shared_memory.find<State>("bjos_state").first;
        return *current;
    }catch(boost::interprocess::interprocess_exception &e){
        //when an error happens the OS is not initialized
        return UNINITIALIZED;
    }
}

void BJOS::shutdown(){
    std::lock_guard<BJOS::Mutex> lock(_mutex);
    
    State *state = _memory_segment.find<State>("bjos_state").first;
    *state = SHUTDOWN;
}

BJOS *BJOS::getOS(){
    //check if os is active
    if(getState() == UNINITIALIZED) return nullptr;

    //NOTE: only thread safe in C++11
    static BJOS instance;
    return &instance;
}

//NOTE: we only open the shared memory and the mutex, it should have already been checked that it actually initialized
//WARNING: we should tweak the size...
BJOS::BJOS(): 
    _memory_segment(open_only,"BJOS_SHARED_MEMORY"),
    _controller_map(0),
    _mutex(open_only, "BJOS_MUTEX")
{
    ControllerAllocator controller_allocator(_memory_segment.get_segment_manager());
        
    std::pair<ControllerMap*, managed_shared_memory::size_type> _check_controller = _memory_segment.find<ControllerMap>("bjos_controller_map");
    if(_check_controller.first == 0){
        //we are the first loader...
        
        //load the map
        _controller_map = _memory_segment.construct<ControllerMap>("bjos_controller_map")      //object name
                                (std::less<interprocess::char_string>(), controller_allocator);    
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
            throw BJOSError("Unload a controller multiple times or unloading a node that is already deregistered!");
        }
    }else{
        //ALERT: unloading an controller that is never loaded... this should be impossible
        throw BJOSError("Trying to unload an controller that is not loaded!");
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