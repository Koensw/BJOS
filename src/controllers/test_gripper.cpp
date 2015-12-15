/* Example implementation of a controller 
 *
 * NOTE: all methods are defined in the .cpp here but in the real controller you also want a header of course
 */

#include <cstdlib>
#include <mutex>
#include <thread>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"
#include "bjos/controller/controller.h"


#include <stdio.h>    // Used for printf() statements
#include <wiringPi.h> // Include WiringPi library!
#include <wiringPiI2C.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 //#include "PCA9685.h"
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <fcntl.h>
#include <syslog.h>		
#include <inttypes.h>
#include <errno.h>
#include <math.h>



//defines for the I2C interface with the pwm board
#define MODE1 0x00			//Mode  register  1
#define MODE2 0x01			//Mode  register  2
#define SUBADR1 0x02		//I2C-bus subaddress 1
#define SUBADR2 0x03		//I2C-bus subaddress 2
#define SUBADR3 0x04		//I2C-bus subaddress 3
#define ALLCALLADR 0x05     //LED All Call I2C-bus address
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels
#define ALLLED_ON_L 0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB	//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC	//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD	//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE		//prescaler for output frequency
#define CLOCK_FREQ 25000000.0 //25MHz default osc clock

//especially this line should not be in a header...
using namespace bjos;

//WARNING: do not store pointers or STL objects here if not absolutely necessary
//NOTE: if you need to store a pointer use boost::offset_ptr (note that arrays can also be pointers...)
//NOTE: if you need to store a STL structure here use the boost::interprocess variants
struct gripperData{
    int armheight;
    //add all other members here
};



class Test_Gripper : public Controller{
public:
    /* Initialize the class 
     * ALERT: dont do anything here that need shared data, mutexes or should only be done in the main process
     * NOTE: normally you only want to set your unitialized values (like pointers) to zero here
     */
	Test_Gripper(): _data(0) {}


	// Pin number declarations. We're using the Broadcom chip pin numbers.
	const int gripPin = 26; // PWM output
	const int armPin = 23; // PWM output
	const int ch7 = 3; // RC input
	const int ch8 = 4; // RC input
	int gripperOffset = 0; // correction for gripper movement
	int fd = 0; // I2C device
	int maxarmlength = 300; // length of arm when fully extended
	int maxactuatorlenght = 50; // length of gripper actuator

    /* IMPLEMENT YOUR OWN FUNCTIONS HERE 
     * WARNING: you can assume that these methods are only called after initialization
     */
	



	 /* Functions for the I2C connection with the pwm board
	 ################################################################################################################
	 */
	void reset() {
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		wiringPiI2CWriteReg8(fd, MODE1, 0x00); //Normal mode
		wiringPiI2CWriteReg8(fd, MODE2, 0x04); //totem pole
	}

	void setPWMFreq(int freq) {
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq) - 1;
		wiringPiI2CWriteReg8(fd, MODE1, 0x10); //sleep
		wiringPiI2CWriteReg8(fd, PRE_SCALE, prescale_val); // multiplyer for PWM frequency
		wiringPiI2CWriteReg8(fd, MODE1, 0x80); //restart
		wiringPiI2CWriteReg8(fd, MODE2, 0x04); //totem pole (default)
	}

	void setPWM2(uint8_t led, int on_value, int off_value) {
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		wiringPiI2CWriteReg8(fd, LED0_ON_L + LED_MULTIPLYER * (led), on_value & 0xFF);
		wiringPiI2CWriteReg8(fd, LED0_ON_H + LED_MULTIPLYER * (led), on_value >> 8);
		wiringPiI2CWriteReg8(fd, LED0_OFF_L + LED_MULTIPLYER * (led), off_value & 0xFF);
		wiringPiI2CWriteReg8(fd, LED0_OFF_H + LED_MULTIPLYER * (led), off_value >> 8);
	}

	void setPWM(uint8_t led, int value) {
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		setPWM2(led, 0, value);
	}



	int getPWM(uint8_t led) {
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		int ledval = 0;
		ledval = wiringPiI2CReadReg8(fd, LED0_OFF_H + LED_MULTIPLYER * (led));
		ledval = ledval & 0xf;
		ledval <<= 8;
		ledval += wiringPiI2CReadReg8(fd, LED0_OFF_L + LED_MULTIPLYER * (led));
		return ledval;
	}

	/* End of functions for the I2C connection with the pwm board
	################################################################################################################
	*/

	/* Arduino functions
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	*/
	long map(long x, long in_min, long in_max, long out_min, long out_max)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	int pulseIn(int pin, int level)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
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

	/* End of arduino functions
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	*/

	/* fucntions for control of the arm
	&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	*/
	bool lower_arm_mm(int mm)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		_data->armheight = _data->armheight + mm;
		if (_data->armheight + gripperOffset>maxarmlength || _data->armheight + gripperOffset<0)
		{
			_data->armheight = _data->armheight - mm;
			return false;
		}
		else {
			int pwm = map((_data->armheight + gripperOffset), 0, maxarmlength, 0, 4095);
			setPWM(1, pwm);
			return true;
		}
	}

	bool lower_arm_to_mm(int mm)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		_data->armheight = mm;
		if (_data->armheight + gripperOffset>maxarmlength || _data->armheight + gripperOffset<0)
		{
			_data->armheight = _data->armheight - mm;
			return false;
		}
		else {
			int pwm = map((_data->armheight + gripperOffset), 0, maxarmlength, 0, 4095);
			setPWM(1, pwm);
			return true;
		}
	}

	int get_blob_size()
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		//communicatie met BJOS voor blobsize
		return 0;
	}

	bool lower_to_object(int px_obj)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		//obtain the blob pixel count from pieter
		int px_blob = get_blob_size();
		if (px_blob<px_obj*1.05 && px_blob>px_obj*0.95)
			return true;
		else if (px_blob<px_obj*0.95)
		{
			while (px_blob<px_obj*0.95)
			{
				if (!lower_arm_mm(2))
					break;
				px_blob = get_blob_size();
				delay(30);
			}
			return true;
		}
		else if (px_blob>px_obj*1.05)
		{
			while (px_blob>px_obj*1.05)
			{
				if (!lower_arm_mm(-2))
					break;
				px_blob = get_blob_size();
				delay(30);
			}
			return true;
		}
		return false;
	}

	/* End of fucntions for control of the arm
	&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	*/

	/*Gripper functions
	$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	*/
	void gripper_close_pwm(int pwm)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		if (pwm>4000)
			pwm = 4000;
		else if (pwm<0)
			pwm = 0;
		setPWM(0, pwm);
		gripperOffset = map(pwm, 0, 4095, maxactuatorlenght, 0);
		lower_arm_mm(0);
	}


	void gripper_close_force(int limit)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
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

	void gripper_close_object(char* object)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		if (strcmp("Apple", object))
			gripper_close_force(35);
		else if (strcmp("Can", object))
			gripper_close_force(50);
		else
			printf("Object not recognized.\n");
		//Serial.println("Object not recognized");
	}

	void pickup(int px_obj, int force)
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		if (lower_to_object(px_obj))
			gripper_close_force(force);
		else
			printf("Something went wrong lowering the arm!\n");
	}
	/*End of gripper functions
	$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	*/


	bool check_RC()//RC override
	{
		std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
		delay(100);
		int RC1 = pulseIn(ch7, HIGH);
		delay(100);
		int RC2 = pulseIn(ch8, HIGH);
		printf("RC1: %i, RC2: %i\n", RC1, RC2);
		if (RC2>1500)
		{
			gripper_close_pwm(800);
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
	/*
    int getInt(){
        //WARNING: for anything that modifies or reads shared data the mutex needs to be locked first
        //NOTE: you can also do mutex->lock() and mutex->unlock manually if needed, but normally you want to use a lock_guard (this also works with exceptions and guarentees unlocking)
        std::lock_guard<BJOS::Mutex> lock(*shared_data_mutex);
        
        //do something with the data
        return _data->int_data;
        
        //NOTE: the mutex is automatically unlocked when this function is left
    }
    void setInt(int int_data){
        //NOTE: example of manual lock and unlock (that you normally dont want to to do)
        shared_data_mutex->lock();
        _data->armheight = int_data;
        shared_data_mutex->unlock();
    }
    */
    /* If necessary you can overload the isAvailable method of the superclass */
    bool isAvailable(){
        //ALERT: first call the super class method to make sure the general interface is available because this function should be safe to call also if not initialized
        bool chk = Controller::isAvailable();
        if(!chk) return false;
        
        //do your own check if this interface is available and ready to use (so it methods are actually doing something real)
        
        //NOTE: if you do nothing here you can delete this method and just use the overloaded version
        return true;
    }
    
    /* Finalize this controller */
    ~Test_Gripper(){
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
        Controller::finalize<gripperData>();
    }

//NOTE: these members are private because they should not be called directly
private:


    /* Initialize the main instance */ 
    void init(BJOS *bjos){
        //ALERT: always call the controller init method at the start of this method and pass the name of the controller and a reference to the shared data struct
        bool ret = Controller::init(bjos, "example", _data);
        
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
    /* load node is called for all childeren */
    void load(BJOS *bjos){
        //ALERT: always call the controller load method at the start of this method and pass the name of the controller and a reference to the shared data struct
        Controller::load(bjos, "example", _data);
        
        //load anything for a normal node if necessary
    }
    
	gripperData *_data;
};

void mainProcess(){
    //reset BJOS (ALERT: this should only be done in the loader, because this will delete all other registered controllers ...)
    BJOS::init();
    
    //most likely you want to install the signal handler to make sure that we can exit properly on SIGTERM etc.
    Process::installSignalHandler();
    
    //get a link to the BJOS
    BJOS *bjos = BJOS::getOS();
    
    //make an unitialized controller
	Test_Gripper *example = new Test_Gripper;
    
    //initialize the controller
    bjos->initController(example);
	int pwm = 0;
    //do things until we got a request to stop the program
    while(Process::isActive()){
		printf("Lower arm(mm): ");
		scanf("%d", &pwm);
		lower_arm_mm(pwm);
		printf("Gripper position(0-4000): ");
		scanf("%d", &pwm);
		gripper_close_pwm(pwm);
        //std::cout << example->getInt() << std::endl;
        sleep(1);
    }
    
    //wait for other clients to unload (the main process should unload the latest)
    while(!example->canFinalize()){
        std::cout << "waiting for clients..." << std::endl;
        sleep(1);
    }
    
    //unload the controller
    delete example;
    
    //finish BJOS (ALERT: this should only be called in the loader that should remain active until all registered controllers are deregistered ...)
    BJOS::finalize();
}

void otherProcess(){
    //most likely you want to install the signal handler to make sure that we can exit properly on SIGTERM etc.
    Process::installSignalHandler();
    
    //get a link to the BJOS
    BJOS *bjos = BJOS::getOS();

    //make an unitialized controller
	Test_Gripper example;
    
    //try to retrieve an instance of a controller
    //ALERT: you should make sure that the controller you try to load is indeed the specified type
    bool ret = bjos->getController("example", &example);
    if(!ret){
        //cannot get the controller (the main process of this controller is not running)
        std::cout << "failed to get controller" << std::endl;
        return;
    }
    
    //do things ...
    std::cout << "setting the example controller to 5" << std::endl;
    example.setInt(5);
    
    //NOTE: the controller is automatically unloaded because its destructor is called when going out of scope (and you can always directly finalize as normal instance)
}

int main(){
    //use threads here to simulate the two processes here
    std::thread mainThd(mainProcess);
    
    //wait two seconds to allow the main process to start...
    sleep(2);
    
    std::thread otherThd(otherProcess);
    
    //wait to both are finished
    mainThd.join();
    otherThd.join();
}