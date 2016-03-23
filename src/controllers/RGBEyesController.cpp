/**
 * @file GripperController.cpp
 *
 * @brief Gripper interface definitions
 *
 * Functions for controller the gripper
 *
 * @author Koen Wolters <koen@bluejayeindhoven.nl>
 */

#include "controllers/RGBEyesController.h"

#include <chrono>

using namespace bjos;

RGBEyesController::RGBEyesController():  _data(0) {

		_ledstring.freq = TARGET_FREQ,
		_ledstring.dmanum = DMA,

		_ledstring.channel[0].gpionum = GPIO_PIN,
		_ledstring.channel[0].count = LED_COUNT,
		_ledstring.channel[0].invert = 0,
		_ledstring.channel[0].brightness = 255,

		_ledstring.channel[1].gpionum = 0,
		_ledstring.channel[1].count = 0,
		_ledstring.channel[1].invert = 0,
		_ledstring.channel[1].brightness = 0,


		_dotcolors[0] = 0x200000;
		_dotcolors[1] = 0x201000;
		_dotcolors[2] = 0x202000;
		_dotcolors[3] = 0x002000;
		_dotcolors[4] = 0x002020;
		_dotcolors[5] = 0x000020;
		_dotcolors[6] = 0x100010;
		_dotcolors[7] = 0x200010;
	
	for (int i = 0; i < 8; i++)
	{
		_dotspos[i] = i;
	}

	
}

RGBEyesController::~RGBEyesController(){
    if(!Controller::isAvailable()) return;
    
    Controller::finalize<SharedRGBEyesData>();
}

void RGBEyesController::init(BJOS *bjos){        
    bool ret = Controller::init(bjos, "eyes", _data);
        
    if(!ret){
        //controller cannot be initialized...
        throw ControllerInitializationError(this, "Cannot initialize controller"); 
    }
        
}

void RGBEyesController::load(BJOS *bjos){    
    Controller::load(bjos, "eyes", _data);
}


void RGBEyesController::matrix_render(void) {
	int x, y;

	for (x = 0; x < WIDTH; x++)
	{
		for (y = 0; y < HEIGHT; y++)
		{
			_ledstring.channel[0].leds[(y * WIDTH) + x] = _matrix[x][y];
		}
	}    
}

void RGBEyesController::matrix_raise(void) {
	int x, y;

	for (y = 0; y < (HEIGHT - 1); y++)
	{
		for (x = 0; x < WIDTH; x++)
		{
			_matrix[x][y] = _matrix[x][y + 1];
		}
	}
}

void RGBEyesController::matrix_bottom(void) {
	int i;

	for (i = 0; i < ARRAY_SIZE(_dotspos); i++)
	{
		_dotspos[i]++;
		if (_dotspos[i] >(WIDTH - 1))
		{
			_dotspos[i] = 0;
		}

		_matrix[_dotspos[i]][HEIGHT - 1] = _dotcolors[i];
	}
}

void RGBEyesController::rgbsollid(int red, int green, int blue){

	for (int i = 0; i < LED_COUNT; i++)
	{
		_ledstring.channel[0].leds[i]= createRGB(red,green,blue);
	}
	if (ws2811_render(&_ledstring))
	{
		ret = -1;
		break;
	}
}

unsigned long RGBEyesController::createRGB(int r, int g, int b)
{
	// 0xRRGGBB
	return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}


int RGBEyesController::start(void) {
	int ret = 0;


	if (ws2811_init(&_ledstring))
	{
		return -1;
	}
	// the next part is only for testing
	/*while (1)
	{
		matrix_raise();
		matrix_bottom();
		matrix_render();

		if (ws2811_render(&_ledstring))
		{
			ret = -1;
			break;
		}

		// 15 frames /sec
		usleep(1000000 / 15);
	}

	ws2811_fini(&_ledstring);*/
	// end testing part

	return ret;

}

void RGBEyesController::stop(void) {
	
	ws2811_fini(&_ledstring);
	// end testing part
	
}


