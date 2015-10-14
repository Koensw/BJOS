#ifndef _BLUEJAY_BJOS_H
#define _BLUEJAY_BJOS_H

#include <functional>
#include <utility>
#include <iostream>
#include <mutex>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/offset_ptr.hpp>

/**
 *  Controller to the OS
 */

#define BJOS_SHARED_MEM_SIZE 65536

namespace bjos{
    namespace interprocess{
        using namespace boost::interprocess;
        
        typedef managed_shared_memory::segment_manager                       segment_manager_t;
        typedef allocator<char, segment_manager_t>                           char_allocator;
        typedef basic_string<char, std::char_traits<char>, char_allocator>   char_string;
    }
    
    struct Controller;
    
    
    class BJOS{
        friend class Controller;
    public:
        /* Init and finalize the OS layer
        * WARNING: this should be called only once before starting an instance and after using the last instance else errors WILL occur
        */
        static void init();
        static void finalize();
        
        /* Get the state of the OS */
        enum State { UNINITIALIZED = 0, ACTIVE, SHUTDOWN };
        static State getState();
        
        /* Request all its client to disconnect
         * WARNING: normally only the loader should invoke this
         */
        void shutdown();
        
        /* Get the OS */
        static BJOS *getOS();
        
        /* Init Controller (should only be done once */
        void initController(Controller *Controller);
        
        /* Get memory segmenter if needs own memory
           WARNING: check exclicitly for bad_alloc
        */
        boost::interprocess::managed_shared_memory &getMemoryManager(){
            return _memory_segment;
        }
        
        /* Get Controller for clients 
        * NOTE: only get one time for a class, doing it another time has no effect
        */
        bool getController(std::string name, Controller *Controller);
        
        /* Utility functions for amount of controllers */
        int getControllerCount(std::string name);
        
        /* TYPEDEFS */
        //TODO: we are using a boost named_mutex but it is currently giving an unitialised value error, so we maybe want to fix this by an extra layer?
        //NOTE: using new seems to fix this?
        typedef boost::interprocess::named_mutex Mutex;
        static const boost::interprocess::open_only_t mutex_open_only;
        
        /* ERROR FUNCTIONS */
        //call this when an status is reached that should be impossible
        //TODO: switch to throwing exception instead
        inline static void fatal_error(std::string msg){
            //TODO: integrate with logger
            std::cout << "[BJOS] PANIC: Reaching state that should never be possible..." << std::endl;
            std::cout << "[BJOS] MESSAGE: " << msg << std::endl;
            std::cout << "[BJOS] EXITING..." << std::endl;
            std::exit(127);
        }
        
    private:  
        /* PREVENT CONSTRUCT AND COPY */
        BJOS();
        ~BJOS(){}
        
        BJOS(BJOS const&);              
        void operator=(BJOS const&);
        
        
        /* REGISTER, LOAD, UNLOAD AND DEREGISTER ControllerS */
        /* Register an Controller in the OS */
        template <typename Type> bool registerController(std::string name, Type*& type){ 
            //lock mutex
            std::lock_guard<Mutex> lock(_mutex);
            
            //convert to shared memory string
            interprocess::char_allocator string_allocator(_memory_segment.get_segment_manager());
            interprocess::char_string ipc_name(name.c_str(), string_allocator);
            if(_controller_map->find(ipc_name)!=_controller_map->end()){
                //WARNING: Controller already exists, this should normally not be possible
                //TODO Provide a way to delete an old specific instance for controller that are loaded outside the general loader (CURRENTLY UNSUPPORTED)
                return false;
            }
            (*_controller_map)[ipc_name] = 1;
            
            //load the data
            try{
                type = _memory_segment.construct<Type>(name.c_str())();
            }catch(boost::interprocess::bad_alloc &e){
                //WARNING: no memory left, this also should normally not happen
                return false;
            }
            
            //remove the mutex if it is kept from earlier run and create new
            Mutex::remove(name.c_str());
            Mutex mutex(boost::interprocess::create_only, name.c_str());
            
            return true;
            
        }
        /* Loads the Controller in the OS */
        template <typename Type> void loadController(std::string name, Type*& type){
            //lock mutex
            std::lock_guard<Mutex> lock(_mutex);
            
            //convert to shared memory string
            interprocess::char_allocator string_allocator(_memory_segment.get_segment_manager());
            interprocess::char_string ipc_name(name.c_str(), string_allocator);
            ControllerMap::iterator iter = _controller_map->find(ipc_name);
            if(iter != _controller_map->end() && iter->second) {
                //increase the counter
                iter->second++;
                
                //Controller is found, load its type now
                std::pair<Type*, boost::interprocess::managed_shared_memory::size_type> res;
                res = _memory_segment.find<Type>(name.c_str());
                
                if(res.first == 0){
                    //ALERT: Controller is registered but its handler is not found... this should never be possible
                    fatal_error("Shared memory belonging to Controller is not available!");
                }
                
                //check if mutex is available
                try{
                    Mutex mutex(mutex_open_only, name.c_str());
                }catch(boost::interprocess::interprocess_exception e){
                    //ALERT: mutex is not avaible... this should never be possible
                    fatal_error("Mutex belonging to Controller not available!");
                }
                            
                //load the Controller
                type = res.first;
            }else{
                //ALERT: trying to load a Controller that is not registered... this should never be possible
                fatal_error("Trying to load an Controller that is not registered first!");
            }
        }
        /* Unloads an Controller from the OS */
        void unloadController(std::string name);
        /* Deregister an Controller from the OS */
        template <typename Type> void deregisterController(std::string name){
            //lock mutex
            std::lock_guard<Mutex> lock(_mutex);
            
            //convert to shared memory string
            interprocess::char_allocator string_allocator(_memory_segment.get_segment_manager());
            interprocess::char_string ipc_name(name.c_str(), string_allocator);
            ControllerMap::iterator iter = _controller_map->find(ipc_name);
            if(iter != _controller_map->end() && iter->second){
                //decrease the count and check if last
                --iter->second;
                if(iter->second != 0){
                    //ALERT: unloading after master node or multiple times
                    fatal_error("Deregistering a node multiple times or deregistering before all other instances are unloaded!");
                }
                
                //unload the loaded memory
                _memory_segment.destroy<Type>(name.c_str());
                
                //delete the mutex
                Mutex::remove(name.c_str());
                
                //delete from map
                _controller_map->erase(ipc_name);
            }else{
                //ALERT: unloading an Controller that is never loaded... this should be impossible
                fatal_error("Trying to deregister an Controller that is not registered!");
            }
        }
        
        /* SHARED MEMORY */
        boost::interprocess::managed_shared_memory _memory_segment;
        
        //NOTE: a set would be enough at the moment, but this is more flexible to store data later
        typedef boost::interprocess::allocator<std::pair<const interprocess::char_string, int>, interprocess::segment_manager_t> ControllerAllocator;
        typedef boost::interprocess::map<interprocess::char_string, int, std::less<interprocess::char_string>, ControllerAllocator> ControllerMap;
        ControllerMap *_controller_map;
        
        Mutex _mutex;
    };
}

#endif