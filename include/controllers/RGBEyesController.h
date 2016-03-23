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

#ifndef _BLUEJAY_RGB_EYES_CONTROLLER_H
#define _BLUEJAY_RGB_EYES_CONTROLLER_H

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

#include <stdint.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>

#include "../libs/clk.h"
#include "../libs/gpio.h"
#include "../libs/dma.h"
#include "../libs/pwm.h"

#include "../libs/ws2811.h"

#define ARRAY_SIZE(stuff)                        (sizeof(stuff) / sizeof(stuff[0]))

#define TARGET_FREQ                              WS2811_TARGET_FREQ
#define GPIO_PIN                                 18						//pwm pin of the raspberry
#define DMA                                      5						

#define WIDTH                                    24						// Number of leds in a row
#define HEIGHT                                   1						// Number of leds in a colum
#define LED_COUNT                                (WIDTH * HEIGHT)		// Total number of leds

namespace bjos {
    struct SharedRGBEyesData{
        //int address;
        //int channel;
        
        bool state;
    };
    
    class RGBEyesController : public Controller{
    public:
        RGBEyesController();
        RGBEyesController(_ledstring ledstring, ws2811_led_t *matrix, int dotspos, ws2811_led_t dotcolors);
        virtual RGBEyesController();
        
        //set if they are enabled
        bool areEnabled();
        
        //toggle the state
        void setEnabled(bool state);

    private:
		int test(void);
		void matrix_render(void);
		void matrix_raise(void);
		void matrix_bottom(void);
        
        /* Initialize the main instance */ 
        void init(BJOS *bjos);
        /* load node is called for all childeren */
        void load(BJOS *bjos);
        


		ws2811_t _ledstring =
		{
			.freq = TARGET_FREQ,
			.dmanum = DMA,
			.channel =
		{
			[0] =
		{
			.gpionum = GPIO_PIN,
			.count = LED_COUNT,
			.invert = 0,
			.brightness = 255,
		},
			[1] =
		{
			.gpionum = 0,
			.count = 0,
			.invert = 0,
			.brightness = 0,
		},
		},
		};

		ws2811_led_t _matrix[WIDTH][HEIGHT];


		int _dotspos[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
		ws2811_led_t _dotcolors[] =
		{
			0x200000,  // red
			0x201000,  // orange
			0x202000,  // yellow
			0x002000,  // green
			0x002020,  // lightblue
			0x000020,  // blue
			0x100010,  // purple
			0x200010,  // pink
		};
        
        SharedRGBEyesData *_data;
    };
}

#endif //_BLUEJAY_RGB_EYES_CONTROLLER_H
