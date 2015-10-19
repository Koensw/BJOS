#include "controllers/SonarController.h"

#include <chrono>
#include <thread>

#include "controllers/sonar/SonarInterface.h"

using namespace bjos;

int SonarController::registerInterface(SonarInterface *interface, bool global){
    if(_interfaces.size() == SharedSonarControllerData::SONAR_SIZE) BJOS::fatal_error("Registering SonarInterface not possible, limit reached! Recompile with large SONAR_SIZE.");
    
    _interfaces.push_back(std::make_pair(interface, global));
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
        std::cout << "Cannot initialize controller " << getControllerName() << std::endl;
        std::exit(0);
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
        
        //TODO: set pose
        //_data->sonars[i].pose;
    }
    
    //start update thread
    _thrd_running = true;
    _thrd = boost::thread(&SonarController::update_sonars, this);    
}

void SonarController::load(bjos::BJOS *bjos){
    Controller::load(bjos, "sonar", _data);
}

void SonarController::update_sonars(){
    bool frst = true;
    while(_thrd_running){
        //TODO: handle sonars that are not active
        
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
        
        try{
            boost::this_thread::sleep_for(boost::chrono::milliseconds(static_cast<int>(1000*_data->update_time)));
        }catch(boost::thread_interrupted){
            //if interrupt, stop and let the controller finish resources
            return;
        }
    }
}
