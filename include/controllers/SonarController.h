#ifndef _BLUEJAY_SONAR_CONTROLLER_H
#define _BLUEJAY_SONAR_CONTROLLER_H

#include <cstdlib>
#include <mutex>
#include <atomic>

#include <boost/thread/thread.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>

#include "../bjos/bjos.h"
#include "../bjos/controller.h"
#include "../bjos/helpers/error.h"

#include "sonar/SonarInterface.h"

#include "../libs/geometry.h"

namespace bjos{
    struct SonarData{
        int id;

        Pose pose;
        
        double field_of_view;
        double min_range;
        double max_range;
        
        double distance;
    };

    struct SharedSonarControllerData{
        SharedSonarControllerData(): sonar_size(0), update_time(0.1) {}
        
        //SET MAXIMUM AMOUNT OF SONARS ON COMPILE TIME
        //TODO: support custom amount of sonars, but this is tricky
        //(either we should find a fix or break our bjos system) 
        static const unsigned int SONAR_SIZE = 12;
        SonarData sonars[SONAR_SIZE];
        unsigned sonar_size;
        
        double update_time;
    };

    class SonarController : public Controller{
    public:
        /* Initialize the sonar controller (global variable only used by main instance) */
        SonarController(bool global = false)
            : _data(nullptr), _thrd_running(false), _global_read(global) {}
        
        /* Register a new sonar
        WARNING: all should be added before init
        WARNING: the controller will delete those interfaces when finalizing
        */
        int registerInterface(SonarInterface *, Pose pose, bool global = false);
        
        /* Get / set global flag (WARNING: only recognized by main instance) */
        void setGlobalRead(bool global){
            _global_read = global;
        }
        bool getGlobalRead(){
            return _global_read;
        }
        
        /* Return data for a registered sonar */
        SonarData getData(int id);
        std::vector<SonarData> getData();
        
        void setUpdateTime(double time){
            std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
            _data->update_time = time;
        }
        double getUpdateTime(){
            std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
            return _data->update_time;
        }
        
        /* Finalize this controller */
        ~SonarController(){
            if(isMainInstance()){
                //stop thread, wait for finish
                _thrd_running = false;
                _thrd.interrupt();
                _thrd.join();
                
                //delete interfaces
                for(size_t i=0; i<_interfaces.size(); ++i){
                    delete _interfaces[i].first;
                }
            }
            
            Controller::finalize<SharedSonarControllerData>();
        }

    private:
        //NOTE: only used by main instance
        /* Thread to update the sonars */
        void update_sonars();
        
        /* Initialize the main instance */ 
        void init(bjos::BJOS *bjos);
        /* load node is called for all childeren */
        void load(bjos::BJOS *bjos);
        
        SharedSonarControllerData *_data;
        
        //NOTE: only used by main instance
        std::vector<std::pair<SonarInterface*, bool> > _interfaces;
        std::vector<Pose> _poses;
        boost::thread _thrd;
        std::atomic_bool _thrd_running;
        bool _global_read;
    };
}

#endif
