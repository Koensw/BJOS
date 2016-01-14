#include "controllers/SonarController.h"
 
#include <chrono>
#include <thread>
#include <stdexcept>

#include "controllers/sonar/SonarInterface.h"
#include "bjos/helpers/error.h"

using namespace bjos;

int SonarController::registerInterface(SonarInterface *interface, Pose pose, bool global){
    if(_interfaces.size() == SharedSonarControllerData::SONAR_SIZE)
        throw std::out_of_range(
        "Registering SonarInterface not possible, limit reached! Recompile with larger SONAR_SIZE.");
    
    _interfaces.push_back(std::make_pair(interface, global));
    _poses.push_back(pose);
    return _interfaces.size()-1;
}

SonarData SonarController::getData(int id){
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->sonars[id];
}

std::vector<SonarData> SonarController::getData(){
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    std::vector<SonarData> res;
    for(size_t i=0; i<_data->sonar_size; ++i){
        res.push_back(_data->sonars[i]);
    }
    return res;
}


void SonarController::init(BJOS *bjos){
    bool ret = Controller::init(bjos, "sonar", _data);
    
    if(!ret){
        //controller cannot be initialized...
        throw ControllerInitializationError(this, "Cannot initialize controller"); 
    }
    
    //set the size of all the available interfaces
    _data->sonar_size = std::move(_interfaces.size());
    //TODO: log a warning if no interfaces attached
    
    //init interfaces
    for(size_t i=0; i<_interfaces.size(); ++i){
        _data->sonars[i].id = i;
        _data->sonars[i].distance = _interfaces[i].first->getMaxRange();
        
        _data->sonars[i].max_range = _interfaces[i].first->getMaxRange();
        _data->sonars[i].min_range = _interfaces[i].first->getMinRange();
        _data->sonars[i].field_of_view = _interfaces[i].first->getFieldOfView();
        _data->sonars[i].pose = _poses[i];
    }
    
    //start update thread
    _thrd_running = true;
    _thrd = boost::thread(&SonarController::update_sonars, this);    
}

void SonarController::load(bjos::BJOS *bjos){
    Controller::load(bjos, "sonar", _data);
}

//TODO: this need to be extended
/*std::string SonarController::getState(){
    std::ostringstream state;
    state << "distance ";
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    std::vector<SonarData> res;
    for(size_t i=0; i<_data->sonar_size; ++i){
        state << _data->sonars[i].distance;
    }
    state << std::endl;
    return state.str();
}*/

void SonarController::update_sonars(){
    bool frst = true;
    
    //FIXME: interrupted over whole block
    while(_thrd_running){
        //TODO: handle sonars that are not active
        try{
            if(!frst){
                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                
                //update data
                for(size_t i=0; i<_interfaces.size(); ++i){
                    _data->sonars[i].distance = _interfaces[i].first->getDistance();
                }
            }
            frst = false;
            
            //trigger new read
            for(size_t i=0; i<_interfaces.size(); ++i){
                if(!_global_read) _interfaces[i].first->readDistance();
                else if(_interfaces[i].second) _interfaces[i].first->globalReadDistance();
            }
            
            boost::this_thread::sleep_for(boost::chrono::milliseconds(
                        static_cast<int>(1000*_data->update_time)));
        }catch(boost::thread_interrupted){
            //if interrupt, stop and let the controller finish resources
            return;
        }
    }
}
