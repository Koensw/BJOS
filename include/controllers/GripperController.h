/**
 * @file GripperController.h
 *
 * @brief Gripper interface definitions
 *
 * Functions for controller the gripper
 *
 * @author Joep Linssen,	   <joep.linssen@bluejayeindhoven.nl>
 * @author Daniël Pijnenborg,  <daniel.pijnenborg@bluejayeindhoven.nl>
 */

#ifndef _BLUEJAY_GRIPPER_CONTROLLER_H
#define _BLUEJAY_GRIPPER_CONTROLLER_H

#include <cstdlib>
#include <mutex>
#include <thread>

#include "../bjos/bjos.h"
#include "../bjos/helpers/process.h"
#include "../bjos/controller.h"

#include "../libs/log.h"

#include <stdio.h>    // Used for printf() statements
#include <softPwm.h>
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
#define MODE1 0x00          //Mode  register  1
#define MODE2 0x01          //Mode  register  2
#define SUBADR1 0x02        //I2C-bus subaddress 1
#define SUBADR2 0x03        //I2C-bus subaddress 2
#define SUBADR3 0x04        //I2C-bus subaddress 3
#define ALLCALLADR 0x05     //LED All Call I2C-bus address
#define LED0 0x6            //LED0 start register
#define LED0_ON_L 0x6       //LED0 output and brightness control byte 0
#define LED0_ON_H 0x7       //LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8      //LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9      //LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4    // For the other 15 channels
#define ALLLED_ON_L 0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB    //load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC   //load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD   //load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE      //prescaler for output frequency
#define CLOCK_FREQ 25000000.0 //25MHz default osc clock

#define DEMO_CUP_PWM 600   //PWM for the gripper that ensures the demo cup is gripped

//FIXME: this should be a param that can be given to the loader of the controller!
#define GRIPPER_PIN 23
#define GRIPPER_CH7 3
#define GRIPPER_CH8 4
#define GRIPPER_PWM_FREQ 1000

namespace bjos {
    struct SharedGripperData{
        int address;
        int armheight;
    };
    
    class GripperController : public Controller{
    public:
        GripperController();
        GripperController(int address);
        virtual ~GripperController();

        /* Gripper functions
        $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        */
        void gripperClosePWM(int pwm);
        void pickup();
        void release();

        // redundant functions that use force feedback
        void gripperCloseForce(int limit);
        void gripperCloseObject(char* object);

        /* RC functions 
        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        */
        // TODO: implement RC override
        bool check_RC();

    private:
        /* Functions for the I2C connection with the pwm board
         ################################################################################################################
        */
        void reset();
        void set_pwm_freq(int freq);
        void set_pwm(uint8_t device, int off_value);
        int get_pwm(uint8_t led);

        /* Arduino functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        */
        long map(long x, long in_min, long in_max, long out_min, long out_max);
        int pulse_in(int pin, int level);

        /* Initialize the main instance */ 
        void init(BJOS *bjos);
        /* load node is called for all childeren */
        void load(BJOS *bjos);
        
        int _address = 0; //temp place to store the address until init
        int _fd = 0; // I2C device
        
        SharedGripperData *_data;
    };
}

#endif //_BLUEJAY_GRIPPER_CONTROLLER_H
