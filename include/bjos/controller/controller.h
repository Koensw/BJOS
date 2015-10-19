#ifndef _BLUEJAY_CONTROLLER_H
#define _BLUEJAY_CONTROLLER_H

#include "../bjos.h"

namespace bjos{
    class Controller{
        friend class BJOS;
    public:
        Controller(): _bjos_instance(0), _main_instance(false) {}
        
        /* Init module is only called for the main node */
        virtual void init(BJOS *bjos) = 0;
        /* Load node is called for all childeren */
        virtual void load(BJOS *bjos) = 0;
        
        /* Check if controller is available (loaded) 
        * NOTE: subclasses can override this, but still have to check first if properly initialized by calling this method!
        */
        virtual bool isAvailable(){
            return (_bjos_instance != 0);
        }
        /* Check if this module can finalize
        * NOTE: normally this is always possible, only the main process should wait for it clients */
        bool canFinalize(){
            if(_main_instance) return _bjos_instance->getControllerCount(_controller_name) == 1;
            else return true;
        }
        
        /* Virtual destructor */
        virtual ~Controller() {}
    protected:
        /* Register the controller */
        template <typename Type> bool init(BJOS *bjos, std::string name, Type *&mem){
            //check if already inited (WARNING: this should normally not be possible)
            if(_bjos_instance != 0) return false;
            
            _controller_name = name;
            _bjos_instance = bjos;
            _main_instance = true;
            bool ret = bjos->registerController(name, mem);
            if(ret){
                //acquire mutex
                shared_data_mutex = new BJOS::Mutex(BJOS::mutex_open_only, name.c_str());
                
                return true;
            }else return false;
        }
        /* Loads a new instance of the controller */
        template <typename Type> void load(BJOS *bjos, std::string name, Type *&mem){
            //check if already loaded then stop
            if(_bjos_instance != 0) return;
            
            //load
            _controller_name = name;
            _bjos_instance = bjos;
            _main_instance = false;
            bjos->loadController(name, mem);
        
            //create the mutex
            shared_data_mutex = new BJOS::Mutex(BJOS::mutex_open_only, name.c_str());
        }
        /* Finish this controller */
        template <typename Type> void finalize(){
            //check if Controller is loaded
            if(_bjos_instance == 0) return;
            
            //remove the mutex
            delete shared_data_mutex;
            
            if(_main_instance) _bjos_instance->deregisterController<Type>(_controller_name);
            else _bjos_instance->unloadController(_controller_name);
        }
        
        /* Check if this is a main instance */
        bool isMainInstance(){
            return _main_instance;
        }
        std::string getControllerName(){
            return _controller_name;
        }
        
        BJOS::Mutex *shared_data_mutex;
    private:
        std::string _controller_name;
        BJOS *_bjos_instance;
        bool _main_instance;
    };
}

#endif