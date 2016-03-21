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

RGBEyesController::RGBEyesController(): _ledstring(-1), _matrix(-1), _dotspos(-1), _dotcolors(-1), _data(0) {}
RGBEyesController::RGBEyesController(_ledstring ledstring, ws2811_led_t *matrix, int dotspos, ws2811_led_t dotcolors): _ledstring(ledstring), _matrix(matrix), dotspos(dotpos), _dotcolors(dotcolors), _data(0)  {}
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

void RGBEyesController::load(BJOS *bjos){    
    Controller::load(bjos, "eyes", _data);
        
    int addr = 0;
    shared_data_mutex->lock();
    addr = _data->address;
    _channel = _data->channel;
    shared_data_mutex->unlock();
        
    //load i2c (TODO: all I2C operations should take only place in loader and only in main thread?)
    _fd = wiringPiI2CSetup(addr);
}

bool RGBEyesController::areEnabled(){
    int pwm = get_pwm(_channel);
    return (pwm > (EYES_PWM_ON+EYES_PWM_OFF)/2.0);
}

void RGBEyesController::setEnabled(bool eyes){
    if(eyes) set_pwm(_channel, EYES_PWM_ON);
    else set_pwm(_channel, EYES_PWM_OFF);
}


void RGBEyesController::matrix_render(void) {
	int x, y;

	for (x = 0; x < WIDTH; x++)
	{
		for (y = 0; y < HEIGHT; y++)
		{
			ledstring.channel[0].leds[(y * WIDTH) + x] = matrix[x][y];
		}
	}    
}

void RGBEyesController::matrix_raise(void) {
	int x, y;

	for (y = 0; y < (HEIGHT - 1); y++)
	{
		for (x = 0; x < WIDTH; x++)
		{
			matrix[x][y] = matrix[x][y + 1];
		}
	}
}

void RGBEyesController::matrix_bottom(void) {
	int i;

	for (i = 0; i < ARRAY_SIZE(dotspos); i++)
	{
		dotspos[i]++;
		if (dotspos[i] >(WIDTH - 1))
		{
			dotspos[i] = 0;
		}

		matrix[dotspos[i]][HEIGHT - 1] = dotcolors[i];
	}
}

static void RGBEyesController::ctrl_c_handler(int signum) {
	ws2811_fini(&ledstring);
}

static void RGBEyesController::setup_handlers(void) {
	struct sigaction sa =
	{
		.sa_handler = ctrl_c_handler,
	};

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
}

int RGBEyesController::test(void) {
	int ret = 0;

	setup_handlers();

	if (ws2811_init(&ledstring))
	{
		return -1;
	}
	// the next part is only for testing
	while (1)
	{
		matrix_raise();
		matrix_bottom();
		matrix_render();

		if (ws2811_render(&ledstring))
		{
			ret = -1;
			break;
		}

		// 15 frames /sec
		usleep(1000000 / 15);
	}

	ws2811_fini(&ledstring);
	// end testing part

	return ret;

}
