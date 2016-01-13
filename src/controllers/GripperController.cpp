/**
 * @file GripperController.cpp
 *
 * @brief Gripper interface definitions
 *
 * Functions for controller the gripper
 *
 * @author Joep Linssen,	   <joep.linssen@bluejayeindhoven.nl>
 * @author Daniël Pijnenborg,  <daniel.pijnenborg@bluejayeindhoven.nl>
 */

#include "controllers/GripperController.h"

#include <chrono>

using namespace bjos;

GripperController::GripperController(): _address(-1), _fd(-1), _data(0) {}
GripperController::GripperController(int addr): _address(addr), _fd(-1), _data(0) {}
GripperController::~GripperController(){
    if(!Controller::isAvailable()) return;
    
    Controller::finalize<SharedGripperData>();
}

void GripperController::init(BJOS *bjos){        
    bool ret = Controller::init(bjos, "gripper", _data);
        
    if(!ret){
        //controller cannot be initialized...
        throw ControllerInitializationError(this, "Cannot initialize controller"); 
    }
        
    //load i2c (TODO: all I2C operations should take only place in loader and only in main thread?)
    if(_address < 0){
        //controller cannot be initialized...
        throw ControllerInitializationError(this, "Address not given to controller");
    }
    _fd = wiringPiI2CSetup(_address);
    
    //set pin modes and config
    pinMode(GRIPPER_CH7, INPUT);     // Set regular as INPUT
    pinMode(GRIPPER_CH8, INPUT);      // Set regular as INPUT
    pullUpDnControl(GRIPPER_CH7, PUD_UP); // Enable pull-up resistor -. weet niet zeker of dit nodig is !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
    pullUpDnControl(GRIPPER_CH8, PUD_UP); // Enable pull-up resistor on 
    
    //reset gripper
    reset();
    set_pwm_freq(GRIPPER_PWM_FREQ);
    
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    _data->address = _address;
}

void GripperController::load(BJOS *bjos){    
    Controller::load(bjos, "gripper", _data);
        
    int addr = 0;
    shared_data_mutex->lock();
    addr = _data->address;
    shared_data_mutex->unlock();
        
    //load i2c (TODO: all I2C operations should take only place in loader and only in main thread?)
    _fd = wiringPiI2CSetup(addr);
}

void GripperController::reset() {
    printf("reset\n");
    wiringPiI2CWriteReg8(_fd, MODE1, 0x00); //Normal mode
    wiringPiI2CWriteReg8(_fd, MODE2, 0x04); //totem pole
}

void GripperController::set_pwm_freq(int freq) {
    printf("setPWMFreqn\n");
    uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq) - 1;
    wiringPiI2CWriteReg8(_fd, MODE1, 0x10); //sleep
    wiringPiI2CWriteReg8(_fd, PRE_SCALE, prescale_val); // multiplyer for PWM frequency
    wiringPiI2CWriteReg8(_fd, MODE1, 0x80); //restart
    wiringPiI2CWriteReg8(_fd, MODE2, 0x04); //totem pole (default)
}

void GripperController::set_pwm(uint8_t device, int off_value) {
    int on_value = 0;
    wiringPiI2CWriteReg8(_fd, LED0_ON_L + LED_MULTIPLYER * (device), on_value & 0xFF);
    wiringPiI2CWriteReg8(_fd, LED0_ON_H + LED_MULTIPLYER * (device), on_value >> 8);
    wiringPiI2CWriteReg8(_fd, LED0_OFF_L + LED_MULTIPLYER * (device), off_value & 0xFF);
    wiringPiI2CWriteReg8(_fd, LED0_OFF_H + LED_MULTIPLYER * (device), off_value >> 8);
}

int GripperController::get_pwm(uint8_t led) {
    printf("getPWM\n");
    int ledval = 0;
    ledval = wiringPiI2CReadReg8(_fd, LED0_OFF_H + LED_MULTIPLYER * (led));
    ledval = ledval & 0xf;
    ledval <<= 8;
    ledval += wiringPiI2CReadReg8(_fd, LED0_OFF_L + LED_MULTIPLYER * (led));
    return ledval;
}

/* Arduino functions
 % *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 */
long GripperController::map(long x, long in_min, long in_max, long out_min, long out_max)
{
    printf("map\n");
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int GripperController::pulse_in(int pin, int level)
{
    printf("pulseIn\n");
    int timeout = 10000;
    struct timeval tn, t0, t1;
    long micros;
    
    gettimeofday(&t0, NULL);
    
    micros = 0;
    
    while (digitalRead(pin) != level)
    {
        gettimeofday(&tn, NULL);
        
        if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
        micros += (tn.tv_usec - t0.tv_usec);
        
        if (micros > timeout) return 0;
    }
    
    gettimeofday(&t1, NULL);
    
    while (digitalRead(pin) == level)
    {
        gettimeofday(&tn, NULL);
        
        if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
        micros = micros + (tn.tv_usec - t0.tv_usec);
        
        if (micros > timeout) return 0;
    }
    
    if (tn.tv_sec > t1.tv_sec) micros = 1000000L; else micros = 0;
    micros = micros + (tn.tv_usec - t1.tv_usec);
    
    return micros;
}

/*Gripper functions
 $ *$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
 */
void GripperController::gripperClosePWM(int pwm)
{
    printf("gripperClosePWM\n");
    if (pwm>4000)
        pwm = 4000;
    else if (pwm<0)
        pwm = 0;
    set_pwm(0, pwm);
}

void GripperController::pickup()
{
    gripperClosePWM(DEMO_CUP_PWM);
}

void GripperController::release()
{
    gripperClosePWM(4095);
}

void GripperController::gripperCloseForce(int limit)
{
    printf("gripper_close_force\n");
    int pwm = 800;
    int force = 100;
    //int force=(analogRead(forcePin))-614;
    while (force<limit)
    {
        pwm--;
        gripperClosePWM(pwm);
        delay(30);
        //force=0.70*force+0.30*((analogRead(forcePin))-614);
        /*Serial.print("Force:  ");
         *	Serial.print(force);
         *	Serial.print(" , PWM: ");
         *	Serial.println(pwm);*/
        if (pwm == 0)
            break;
    }
}

void GripperController::gripperCloseObject(char* object)
{
    printf("gripper_close_object\n");
    if (strcmp("Apple", object))
        gripperCloseForce(35);
    else if (strcmp("Can", object))
        gripperCloseForce(50);
    else
        printf("Object not recognized.\n");
    //Serial.println("Object not recognized");
}

/* RC functions 
 & *&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 */
bool GripperController::check_RC()//RC override
{
    printf("check_RC\n");
    delay(100);
    int RC1 = pulse_in(GRIPPER_CH7, HIGH);
    delay(100);
    int RC2 = pulse_in(GRIPPER_CH8, HIGH);
    printf("RC1: %i, RC2: %i\n", RC1, RC2);
    if (RC2>1500)
    {
        gripperClosePWM(4095);
        return(true);
    }
    else if (RC1>1500) {
        gripperClosePWM(0);
        return(true);
    }
    else {
        return(false);
    }
    
}