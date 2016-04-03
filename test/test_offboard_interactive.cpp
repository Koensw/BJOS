#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>

#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"

#include "controllers/FlightController.h"
#include "controllers/GripperController.h"
#include "controllers/EyesController.h"

/*
 *  Test offboard
 */

using namespace bjos;

float VEL = 1.0;
float TAKEOFF = 2.0;
float ZVEL = 0.5;
float YAWR = 0.2;

bool doReboot = false;

#define M_EPS 1e-7

std::tuple<Eigen::Vector3d, Eigen::Vector3d, uint16_t, bool, int, bool, int> handle_input(char c)
{
    static int gripPWM = 600;
    static int eyesEnabled = false;
    
    static Eigen::Vector3d setv_old;
    static Eigen::Vector3d setav_old;
    static uint16_t tm_old;
    static bool gripActivate_old;
    static bool eyesActivate_old;
    Eigen::Vector3d setv(0,0,0);
    Eigen::Vector3d setav(0,0,0);
    uint16_t tm = SET_TARGET_VELOCITY;
    bool gripActivate = false;
    bool eyesActivate = false;
    
    switch (c) {
        case 'w':
            std::cout << "Going forward" << std::endl;
            setv[0] = VEL;
            break;
            
        case 's':
            std::cout << "Going backward" << std::endl;
            setv[0] = -VEL;
            break;
            
        case 'a':
            std::cout << "Going left" << std::endl;
            setv[1] = VEL;
            break;
            
        case 'd':
            std::cout << "Going right" << std::endl;
            setv[1] = -VEL;
            break;
            
        case 'e':
            std::cout << "Turning left" << std::endl;
            setav[2] = YAWR;
            setv = setv_old;
            tm &= SET_TARGET_YAW_RATE;
            break;
            
        case 'r':
            std::cout << "Turning right" << std::endl;
            setav[2] = -YAWR;
            setv = setv_old;
            tm &= SET_TARGET_YAW_RATE;
            break;
            
        case 'g':
            std::cout << "Landing" << std::endl;
            tm = SET_TARGET_LAND;
            break;
            
        case 't':
            std::cout << "Takeoff!" << std::endl;
            tm = SET_TARGET_TAKEOFF;
            break;
            
        case 'o':
            std::cout << "Going upward" << std::endl;
            setv[2] = ZVEL;
            break;
            
        case 'l':
            std::cout << "Going downward" << std::endl;
            setv[2] = -ZVEL;
            break;
            
        case 'n':
            std::cout << "Gripper closing" << std::endl;
            gripActivate = true;
            gripPWM = 600;
            break;
            
        case 'm':
            std::cout << "Gripper opening" << std::endl;
            gripActivate = true;
            gripPWM = 4095;
            break;
            
        case ',':
            std::cout << "Gripper tightening" << std::endl;
            gripActivate = true;
            gripPWM -= 200;
            break;
            
        case '.':
            std::cout << "Gripper loosening" << std::endl;
            gripActivate = true;
            gripPWM += 200;
            break;
            
        case '1':
            std::cout << "Going forward: 0.5 m/s" << std::endl;
            setv[0] = 0.5;
            break;

        case '2':
            std::cout << "Going forward: 1.0 m/s" << std::endl;
            setv[0] = 1.0;
            break;

        case '3':
            std::cout << "Going forward: 1.5 m/s" << std::endl;
            setv[0] = 1.5;
            break;

        case '4':
            std::cout << "Going forward: 2.0 m/s" << std::endl;
            setv[0] = 2.0;
            break;

        case '5':
            std::cout << "Going forward: 2.5 m/s" << std::endl;
            setv[0] = 2.5;
            break;

        case '6':
            std::cout << "Going forward: 3.0 m/s" << std::endl;
            setv[0] = 3.0;
            break;

        case '7':
            std::cout << "Going forward: 3.5 m/s" << std::endl;
            setv[0] = 3.5;
            break;

        case '8':
            std::cout << "Going forward: 4.0 m/s" << std::endl;
            setv[0] = 4.0;
            break;

        case '9':
            std::cout << "Going forward: 4.5 m/s" << std::endl;
            setv[0] = 4.5;
            break;
            
        case 'b':
            std::cout << "Execute reboot" << std::endl;
            doReboot = true;
            break;

        case '+':
            std::cout << "Eyes are on" << std::endl;
            eyesActivate = true;
            eyesEnabled = true;
            break;
            
        case '-':
            std::cout << "Eyes are off" << std::endl;
            eyesActivate = true;
            eyesEnabled = false;
            break;
        
        case 'q':
            std::cout << "Quitting... :(" << std::endl;
            break;
            
        case '\n':
            setv = setv_old;
            setav = setav_old;
            tm = tm_old;
            gripActivate = gripActivate_old;
            eyesActivate = eyesActivate_old;
            break;
            
        case 'h':
        default:
            std::cout << "Holding..." << std::endl;
            break;
    }
    setv_old = setv;
    setav_old = setav;
    tm_old = tm;
    gripActivate_old = gripActivate;
    eyesActivate_old = eyesActivate;
    
    return std::make_tuple(setv, setav, tm, gripActivate, gripPWM, eyesActivate, eyesEnabled);
}

int main(){
    Process::installSignalHandler();
    //BJOS::init();
    BJOS *bjos = BJOS::getOS();
    if(bjos == nullptr){
        std::cout << "BJOS is not available..." << std::endl;
        return 0;
    }
    
    FlightController flight;
    bjos->getController("flight", &flight);
    if(!flight.isAvailable()){
        std::cout << "Failed to retrieve flight controller" << std::endl;
        return 0;
    }
    
    GripperController gripper;
    bjos->getController("gripper", &gripper);
    if (!gripper.isAvailable()) {
        std::cout << "Failed to retrieve gripper controller!" << std::endl;
        //return 0;
    }
    
    EyesController eyes;
    bjos->getController("eyes", &eyes);
    if (!eyes.isAvailable()) {
        std::cout << "Failed to retrieve eyes controller!" << std::endl;
        //return 0;
    }
    
    char ret;
    std::cout << "Interactive Offboard Tester!\n----------------------------" << std::endl;
    do {
        ret = std::cin.get();
        if (!Process::isActive()) ret = 'q';
        
        if (ret == '\\') {
            std::cout << "KILLING MOTORS" << std::endl;
            flight.killMotors();
            ret = 'q';
        }else if (ret == '!'){
            std::cout << "FORCING FAILSAFE" << std::endl;
            flight.forceFailsafe();
            ret = 'q';
        }
        
        auto action = handle_input(ret);
        
        if(doReboot){
            std::cout << "Reboot in progress... (please wait)" << std::endl;
            flight.reboot();
            doReboot = false;
            std::cout << "Reboot completed!" << std::endl;
        }
        
        flight.setTargetCF(std::get<2>(action), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), std::get<0>(action), std::get<1>(action));
        
        if(gripper.isAvailable() && std::get<3>(action)) {
            gripper.gripperClosePWM(std::get<4>(action));
        }
        if(eyes.isAvailable() && std::get<5>(action)){
            eyes.setEnabled(std::get<6>(action));
        }
        
    } while (ret != 'q');
    
    std::cout << "Byebye!" << std::endl;
}
