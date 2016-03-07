/**
 * @file GripperController.cpp
 *
 * @brief Gripper interface definitions
 *
 * Functions for controller the gripper
 *
 * @author Koen Wolters <koen@bluejayeindhoven.nl>
 */

#include "controllers/EyesController.h"

#include <chrono>

using namespace bjos;

EyesController::EyesController(): _address(-1), _fd(-1), _data(0) {}
EyesController::EyesController(int addr, int chan): _address(addr), _channel(chan), _fd(-1), _data(0) {}
EyesController::~EyesController(){
    if(!Controller::isAvailable()) return;
    
    Controller::finalize<SharedEyesData>();
}

void EyesController::init(BJOS *bjos){        
    bool ret = Controller::init(bjos, "eyes", _data);
        
    if(!ret){
        //controller cannot be initialized...
        throw ControllerInitializationError(this, "Cannot initialize controller"); 
    }
        
    //load i2c (TODO: all I2C operations should take only place in loader and only in main thread?)
    if(_address < 0){
        //controller cannot be initialized...
        throw ControllerInitializationError(this, "Address not given to controller");
    }
    
    //initialize I2C
    _fd = wiringPiI2CSetup(_address);
    
    //reset device
    reset();
    set_pwm_freq(EYES_PWM_FREQ);
    
    //default the eyes to off
    setEnabled(false);
    
    //save the address
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    _data->address = _address;
    _data->channel = _channel;
}

void EyesController::load(BJOS *bjos){    
    Controller::load(bjos, "eyes", _data);
        
    int addr = 0;
    shared_data_mutex->lock();
    addr = _data->address;
    _channel = _data->channel;
    shared_data_mutex->unlock();
        
    //load i2c (TODO: all I2C operations should take only place in loader and only in main thread?)
    _fd = wiringPiI2CSetup(addr);
}

bool EyesController::areEnabled(){
    int pwm = get_pwm(_channel);
    return (pwm > (EYES_PWM_ON+EYES_PWM_OFF)/2.0);
}

void EyesController::setEnabled(bool eyes){
    Log::info("EyesController", "%d %d %d", _channel, eyes, areEnabled());
    if(eyes) set_pwm(_channel, EYES_PWM_ON);
    else set_pwm(_channel, EYES_PWM_OFF);
    Log::info("EyesController", "%d %d %d", _channel, eyes, areEnabled());
}

void EyesController::reset() {
    Log::info("EyesController", "Reset I2C device");
    wiringPiI2CWriteReg8(_fd, MODE1, 0x00); //Normal mode
    wiringPiI2CWriteReg8(_fd, MODE2, 0x04); //Totem pole
}

void EyesController::set_pwm_freq(int freq) {
    Log::info("EyesController", "Set I2C device pwm frequency", freq);
    uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq) - 1;
    wiringPiI2CWriteReg8(_fd, MODE1, 0x10); //Sleep
    wiringPiI2CWriteReg8(_fd, PRE_SCALE, prescale_val); // Multiplyer for PWM frequency
    wiringPiI2CWriteReg8(_fd, MODE1, 0x80); //Restart
    wiringPiI2CWriteReg8(_fd, MODE2, 0x04); //Totem pole (default)
}

void EyesController::set_pwm(uint8_t device, int off_value) {
    int on_value = 0;
    wiringPiI2CWriteReg8(_fd, LED0_ON_L + LED_MULTIPLYER * (device), on_value & 0xFF);
    wiringPiI2CWriteReg8(_fd, LED0_ON_H + LED_MULTIPLYER * (device), on_value >> 8);
    wiringPiI2CWriteReg8(_fd, LED0_OFF_L + LED_MULTIPLYER * (device), off_value & 0xFF);
    wiringPiI2CWriteReg8(_fd, LED0_OFF_H + LED_MULTIPLYER * (device), off_value >> 8);
}

int EyesController::get_pwm(uint8_t device) {
    int retval = 0;
    retval = wiringPiI2CReadReg8(_fd, LED0_OFF_H + LED_MULTIPLYER * (device));
    retval = retval & 0xf;
    retval <<= 8;
    retval += wiringPiI2CReadReg8(_fd, LED0_OFF_L + LED_MULTIPLYER * (device));
    return retval;
}