#include "controllers/SonarController.h"

#include <chrono>
#include <thread>

#include "controllers/sonar/SonarInterface.h"

using namespace bjos;

int SonarController::registerInterface(SonarInterface *interface){
    if(_interfaces.size() == SharedSonarControllerData::SONAR_SIZE) BJOS::fatal_error("Registering SonarInterface not possible, limit reached! Recompile with large SONAR_SIZE.");
    
    _interfaces.push_back(interface);
    return _interfaces.size()-1;
}

SonarData SonarController::getData(int id){
    std::lock_guard<bjos::BJOS::Mutex> lock(*mutex);
    return _data->sonars[id];
}

std::vector<SonarData> SonarController::getData(){
    std::lock_guard<bjos::BJOS::Mutex> lock(*mutex);
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
    for(size_t i=0; i<_data->sonar_size; ++i){
        _data->sonars[i].distance = _interfaces[i]->getMaxRange();
        //TODO: set pose
        //_data->sonars[i].pose
    }
    
    //start update thread
    _thrd_running = true;
    _thrd = boost::thread(&SonarController::update_sonars, this);    
}

void SonarController::load(bjos::BJOS *bjos){
    Controller::load(bjos, "sonar", _data);
}

void SonarController::update_sonars(){
    while(_thrd_running){
        {
            std::lock_guard<bjos::BJOS::Mutex> lock(*mutex);
        
            
        }
        
        try{
            boost::this_thread::sleep_for(boost::chrono::milliseconds(static_cast<int>(1000*_data->update_time)));
        }catch(boost::thread_interrupted){
            //if interrupt, stop and let the controller finish resources
            return;
        }
    }
}