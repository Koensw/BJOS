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

GripperController::GripperController(): _data(0) {}
GripperController::~GripperController(){
        if(isMainInstance()){
            //WARNING: in most cases you want to check that you really need to delete things, because we cannot be certain that this controller is registered!
            if(!Controller::isAvailable()) return;
            
            //do own destruction for the main instance here...
        }else{
            //WARNING: in most cases you want to check that you really need to delete things, because we cannot be certain that this controller is loaded!
            if(!Controller::isAvailable()) return;
            
            //do own destruction for all other instances here...
        }
        
        //ALERT: always call the finalize method in the super class at the end of the destructor (unless the controller is not available)
        //ALERT: pass the datatype of the shared example data
        Controller::finalize<SharedGripperData>();
    }

void GripperController::init(BJOS *bjos){
    //ALERT: always call the controller init method at the start of this method and pass the name of the controller and a reference to the shared data struct
    bool ret = Controller::init(bjos, "gripper", _data);
    
    //check if properly initialized
    if(!ret){
        //if not then the controller is not usable
        //most probably you just want to print an error and quit...
        std::cout << "Cannot initialize controller " << getControllerName() << std::endl;
        std::exit(0);
    }
    
    //initialize the main process... (dont forget to lock the mutex if you access the shared memory)
    //NOTE: you maybe want to start a thread here if you want to sync things periodically (make sure that you dont hang in this function)
}

void GripperController::load(BJOS *bjos){
    Controller::load(bjos, "gripper", _data);
}

void GripperController::reset() {
		printf("reset\n");
		wiringPiI2CWriteReg8(fd, MODE1, 0x00); //Normal mode
		wiringPiI2CWriteReg8(fd, MODE2, 0x04); //totem pole
	}

void GripperController::setPWMFreq(int freq) {
	printf("setPWMFreqn");
	uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq) - 1;
	wiringPiI2CWriteReg8(fd, MODE1, 0x10); //sleep
	wiringPiI2CWriteReg8(fd, PRE_SCALE, prescale_val); // multiplyer for PWM frequency
	wiringPiI2CWriteReg8(fd, MODE1, 0x80); //restart
	wiringPiI2CWriteReg8(fd, MODE2, 0x04); //totem pole (default)
}

void GripperController::setPWM(uint8_t device, int off_value) {
	int on_value = 0;
	wiringPiI2CWriteReg8(fd, LED0_ON_L + LED_MULTIPLYER * (device), on_value & 0xFF);
	wiringPiI2CWriteReg8(fd, LED0_ON_H + LED_MULTIPLYER * (device), on_value >> 8);
	wiringPiI2CWriteReg8(fd, LED0_OFF_L + LED_MULTIPLYER * (device), off_value & 0xFF);
	wiringPiI2CWriteReg8(fd, LED0_OFF_H + LED_MULTIPLYER * (device), off_value >> 8);
}



int GripperController::getPWM(uint8_t led) {
	printf("getPWM\n");
	int ledval = 0;
	ledval = wiringPiI2CReadReg8(fd, LED0_OFF_H + LED_MULTIPLYER * (led));
	ledval = ledval & 0xf;
	ledval <<= 8;
	ledval += wiringPiI2CReadReg8(fd, LED0_OFF_L + LED_MULTIPLYER * (led));
	return ledval;
}

/* Arduino functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
long GripperController::map(long x, long in_min, long in_max, long out_min, long out_max)
{
	printf("map\n");
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int GripperController::pulseIn(int pin, int level)
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
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
*/
void GripperController::gripper_close_pwm(int pwm)
{
	printf("gripper_close_pwm\n");
	if (pwm>4000)
		pwm = 4000;
	else if (pwm<0)
		pwm = 0;
	setPWM(0, pwm);
}

void GripperController::pickup()
{
	gripper_close_pwm(DEMO_CUP_PWM);
}

void GripperController::release()
{
	gripper_close_pwm(4095);
}

void GripperController::gripper_close_force(int limit)
{
	printf("gripper_close_force\n");
	int pwm = 800;
	int force = 100;
	//int force=(analogRead(forcePin))-614;
	while (force<limit)
	{
		pwm--;
		gripper_close_pwm(pwm);
		delay(30);
		//force=0.70*force+0.30*((analogRead(forcePin))-614);
		/*Serial.print("Force:  ");
		Serial.print(force);
		Serial.print(" , PWM: ");
		Serial.println(pwm);*/
		if (pwm == 0)
			break;
	}
}

void GripperController::gripper_close_object(char* object)
{
	printf("gripper_close_object\n");
	if (strcmp("Apple", object))
		gripper_close_force(35);
	else if (strcmp("Can", object))
		gripper_close_force(50);
	else
		printf("Object not recognized.\n");
	//Serial.println("Object not recognized");
}

/* RC functions 
&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
*/
bool GripperController::check_RC()//RC override
{
	printf("check_RC\n");
	delay(100);
	int RC1 = pulseIn(ch7, HIGH);
	delay(100);
	int RC2 = pulseIn(ch8, HIGH);
	printf("RC1: %i, RC2: %i\n", RC1, RC2);
	if (RC2>1500)
	{
		gripper_close_pwm(4095);
		return(true);
	}
	else if (RC1>1500) {
		gripper_close_pwm(0);
		return(true);
	}
	else {
		return(false);
	}

}