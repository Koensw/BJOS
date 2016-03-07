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

#ifndef _BLUEJAY_EYES_CONTROLLER_H
#define _BLUEJAY_EYES_CONTROLLER_H

#include <cstdlib>
#include <mutex>
#include <thread>

#include "../bjos/bjos.h"
#include "../bjos/helpers/process.h"
#include "../bjos/controller.h"

#include "../libs/log.h"

#include <stdio.h>  
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
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

#include <wiringPi.h> 
#include <wiringPiI2C.h>

//defines for the I2C interface with the pwm board (FIXME: better defines)
#define MODE1 0x00          //Mode  register  1
#define MODE2 0x01          //Mode  register  2
#define LED0 0x6            //LED0 start register
#define LED0_ON_L 0x6       //LED0 output and brightness control byte 0
#define LED0_ON_H 0x7       //LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8      //LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9      //LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4    // For the other 15 channels
#define PRE_SCALE 0xFE      //prescaler for output frequency
#define CLOCK_FREQ 25000000.0 //25MHz default osc clock

//FIXME: check that this frequency cannot be set higher
#define EYES_PWM_FREQ 1000

#define EYES_PWM_ON 3000
#define EYES_PWM_OFF 0

namespace bjos {
    struct SharedEyesData{
        int address;
        int channel;
        
        bool state;
    };
    
    class EyesController : public Controller{
    public:
        EyesController();
        EyesController(int addr, int chan);
        virtual ~EyesController();
        
        //set if they are enabled
        bool areEnabled();
        
        //toggle the state
        void setEnabled(bool state);

    private:
        void reset();
        void set_pwm_freq(int freq);
        void set_pwm(uint8_t device, int value);
        int get_pwm(uint8_t device);
        
        /* Initialize the main instance */ 
        void init(BJOS *bjos);
        /* load node is called for all childeren */
        void load(BJOS *bjos);
        
        int _address = 0; //store the address until shared data available
        int _channel = 0; //store the channel until shared data available and keeps as copy
        
        int _fd = 0; //I2C device
        
        SharedEyesData *_data;
    };
}

#endif //_BLUEJAY_GRIPPER_CONTROLLER_H
