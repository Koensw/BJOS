#ifndef _BLUEJAY_SONAR_CONTROLLER_H
#define _BLUEJAY_SONAR_CONTROLLER_H

#include <cstdlib>
#include <mutex>

#include <boost/thread/thread.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>

#include "geometry.h"

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"
#include "bjos/controller/controller.h"

struct SonarData{
    Pose pose;
    
    double field_of_view;
    double min_range;
    double max_range;
    
    double distance;
};

struct SharedSonarControllerData{
    SharedSonarControllerData(): sonar_size(0), update_time(0.1) {}
    
    //SET MAXIMUM AMOUNT OF SONARS ON COMPILE TIME
    //TODO: support custom amount of sonars, but this is tricky (either we should find a fix or break our bjos system) 
    static const unsigned int SONAR_SIZE = 12;
    SonarData sonars[SONAR_SIZE];
    unsigned sonar_size;
    
    double update_time;
};

struct SonarInterface;

class SonarController : public bjos::Controller{
public:
    /* Initialize the controller for the main instance */
    SonarController(): _data(nullptr), _thrd_running(false) {}
    
    /* Register a new sonar
       WARNING: all should be added before init
     */
    int registerInterface(SonarInterface *);
    
    /* Return data for a registered sonar */
    SonarData getData(int id);
    std::vector<SonarData> getData();
    
    /* Get / set the update time for the sonars */
    void setUpdateTime(double time){
        std::lock_guard<bjos::BJOS::Mutex> lock(*mutex);
        _data->update_time = time;
    }
    double getUpdateTime(){
        std::lock_guard<bjos::BJOS::Mutex> lock(*mutex);
        return _data->update_time;
    }
    
    /* Finalize this controller */
    ~SonarController(){
        if(isMainInstance()){
            //stop thread, wait for finish
            _thrd_running = false;
            _thrd.interrupt();
            _thrd.join();
        }
        
        Controller::finalize<SharedSonarControllerData>();
    }

private:
    /* Thread to update the sonars */
    void update_sonars();
    
    /* Initialize the main instance */ 
    void init(bjos::BJOS *bjos);
    /* load node is called for all childeren */
    void load(bjos::BJOS *bjos);
    
    SharedSonarControllerData *_data;
    
    //NOTE: only used by main instance
    std::vector<SonarInterface*> _interfaces;
    boost::thread _thrd;
    std::atomic_bool _thrd_running;
};

#endif