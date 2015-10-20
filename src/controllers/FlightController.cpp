/**
* @file flightController.cpp
*
* @brief Drone interface functions
*
* Functions for sending and receiving commands to a drone via MAVLink
*
* @author Joep Linssen,	<joep.linssen@bluejayeindhoven.nl>
*/

#include "controllers/FlightController.h"

#include <chrono>

using namespace bjos;

//WARNING: blocks while no initial messages are received from the drone
//TODO: neater exceptions
void FlightController::init(BJOS *bjos) {
	bool ret = Controller::init(bjos, "flight", _data);

	if (!ret) {
		//controller cannot be initialized...
		log.fatal("FlightController::init","Cannot initialize %s", getControllerName());
		throw 1;
	}

	//NOTE: uses default uart="/dev/ttyAMA0", baudrate = "57600"
	serial_port = new Serial_Port();
	if (not serial_port->status == 1) //SERIAL_PORT_OPEN
	{
		//serial_port not open...
		log.fatal("FlightController::init","Serial port not open");
		throw 1;
	}

	_read_thrd_running = true;
	_read_thrd = boost::thread(read_thread);

	std::cout << "Receiving initial position ...";
	while (_init_set == false) {
		//TODO: timeout
		usleep(500000); //2 Hz
		std::cout << " ...";
	}

	std::cout << " Received!" << std::endl;
	log.info("FlightController::init", "Initial position: xyz=[%.4f %.4f %.4f] vxvyvz=[%.4f %.4f %.4f]", initial_position.x, initial_position.y, initial_position.z, initial_position.vx, initial_position.y, initial_position.z);

	_write_thrd = boost::thread(write_thread);
	
	//wait until it starts
	while (_write_thrd_running == false)
		usleep(100000); //10 Hz

	int result = toggle_offboard_control(true);
	


	//NOTE: initial_position and current_setpoint are not initialised here. 
}