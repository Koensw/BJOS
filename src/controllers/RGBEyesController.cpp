
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

		/*shared_data_mutex->lock();
		_data->confirm=false;
		_data->cancel=false;
		shared_data_mutex->unlock();*/
	
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
    bool ret = Controller::init(bjos, "rgbeyes", _data);
        
    if(!ret){
        //controller cannot be initialized...
        throw ControllerInitializationError(this, "Cannot initialize controller"); 
    }
        
}

void RGBEyesController::load(BJOS *bjos){    
    Controller::load(bjos, "rgbeyes", _data);
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




void RGBEyesController::angle(int deg) {

	int onled = (int)((float)deg*11.0f / 180.0f);
	for (int i = 0; i < 12; i++)
	{
		if (i == onled)
		{
			_ledstring.channel[0].leds[i] = createRGB(0, 0, 255);
			_ledstring.channel[0].leds[12 + i] = createRGB(0, 0, 255);
		}
		else
		{
			_ledstring.channel[0].leds[i] = createRGB(0, 0, 0);
			_ledstring.channel[0].leds[12 + i] = createRGB(0, 0, 0);
		}
	}
	ws2811_render(&_ledstring);

}

void RGBEyesController::rgbfill(int red, int green, int blue) {

	for (int i = 0; i < 12; i++)
	{
		_ledstring.channel[0].leds[i] = createRGB(0, 0, 0);
		_ledstring.channel[0].leds[12 + i] = createRGB(0, 0, 0);
	}
	ws2811_render(&_ledstring);
	int i = 0;
	for (i = 0; i < 12; i++)
	{
		_ledstring.channel[0].leds[11 - i] = createRGB(red, green, blue);
		_ledstring.channel[0].leds[12 + i] = createRGB(red, green, blue);
		ws2811_render(&_ledstring);
		if(RGBEyesCont)
		// 4 frames /sec
		usleep(1000000 / 4);
		/*shared_data_mutex->lock();
		if (_data->cancel)
			break;
		shared_data_mutex->unlock();*/
	}
	/*while (!(cancel || confirm))
	{

	}*/
}




void RGBEyesController::rgbsollid(int red, int green, int blue) {

	for (int i = 0; i < LED_COUNT; i++)
	{
		_ledstring.channel[0].leds[i] = createRGB(red, green, blue);
	}
	ws2811_render(&_ledstring);

}

void RGBEyesController::rgbmatrix(int rgbmatrix[24][3]) {
	for (int i = 0; i < LED_COUNT; i++)
	{
		_ledstring.channel[0].leds[i] = createRGB(rgbmatrix[i][0], rgbmatrix[i][1], rgbmatrix[i][2]);
	}
	ws2811_render(&_ledstring);
}




void RGBEyesController::cancel() {

	for (int i = 0; i < LED_COUNT; i++)
	{
		_ledstring.channel[0].leds[i] = createRGB(255, 0, 0);
	}
	ws2811_render(&_ledstring);
	usleep(1000000 / 1);
	for (int i = 0; i < LED_COUNT; i++)
	{
		_ledstring.channel[0].leds[i] = createRGB(0, 0, 0);
	}
	ws2811_render(&_ledstring);
}

void RGBEyesController::confirm() {

	for (int i = 0; i < LED_COUNT; i++)
	{
		_ledstring.channel[0].leds[i] = createRGB(0, 255, 0);
	}
	ws2811_render(&_ledstring);
	usleep(1000000 / 1);
	for (int i = 0; i < LED_COUNT; i++)
	{
		_ledstring.channel[0].leds[i] = createRGB(0, 0, 0);
	}
	ws2811_render(&_ledstring);
}

void RGBEyesController::animate() {
	for (int i = 0; i < 240; i++)
	{
		matrix_raise();
		matrix_bottom();
		matrix_render();

		ws2811_render(&_ledstring);


		// 15 frames /sec
		usleep(1000000 / 15);
	}
}

uint32_t RGBEyesController::createRGB(int r, int g, int b)
{
	// 0xRRGGBB
	return ((g & 0xff) << 16) + ((r & 0xff) << 8) + (b & 0xff);
}


int RGBEyesController::test(void) {
	int ret = 0;


	if (ws2811_init(&_ledstring))
	{
		return -1;
	}
	// the next part is only for testing
	while (1)
	{
		int option = 0;
		std::cout << "chose function:" << std::endl;
		std::cout << "1: rgb solid on" << std::endl;
		std::cout << "2: rgb fill" << std::endl;
		std::cout << "3: blue direction:" << std::endl;
		std::cout << "4: confirm:" << std::endl;
		std::cout << "5: cancel:" << std::endl;
		std::cout << "6: animate:" << std::endl;
		std::cout << "7: recieve 24x3 matrix:" << std::endl;
		std::cin >> option;

		int red;
		int blue;
		int green;
		int deg = 0;
		switch (option)
		{
		case 1:
			std::cout << "RGB color (0-255 0-225 0-255): " << std::endl;
			std::cin >> red;
			std::cin >> green;
			std::cin >> blue;
			rgbsollid(red, green, blue);
			break;

		case 2:
			std::cout << "RGB fill color (0-255 0-225 0-255): " << std::endl;
			std::cin >> red;
			std::cin >> green;
			std::cin >> blue;
			rgbfill(red, green, blue);
			break;

		case 3:
			std::cout << "view direction(0-180): " << std::endl;
			std::cin >> deg;
			angle(deg);
			break;

		case 4:
			confirm();
			break;

		case 5:
			cancel();
			break;

		case 6:
			animate();
			break;

		case 7:
			int m[24][3] = {0};
			std::cout << "RGB matrix: " << std::endl;
			for (int i = 0; i < 24; i++)
			{
				std::cout << "RGB fill color "<< i <<"th led (0-255 0-225 0-255): " << std::endl;
				std::cin >> m[i][0];
				std::cin >> m[i][1];
				std::cin >> m[i][2];
			}
			rgbmatrix(m)
			std::cin >> red;
			std::cin >> green;
			std::cin >> blue;
			rgbfill(red, green, blue);
			break;

		default:
			break;		
		}

		/*matrix_raise();
		matrix_bottom();
		matrix_render();

		if (ws2811_render(&_ledstring))
		{
			ret = -1;
			break;
		}*/

		// 15 frames /sec
		usleep(1000000 / 15);
	}

	ws2811_fini(&_ledstring);
	// end testing part

	return ret;

}
