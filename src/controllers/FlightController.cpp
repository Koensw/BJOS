
/**
* @file flightController.cpp
*
* @brief Drone interface functions
*
* Functions for sending and receiving commands to a drone via MAVLink
*
* @author Joep Linssen,	<joep.linssen@bluejayeindhoven.nl>
*/

//TODO: implement basic toggle_offboard_control

#include "controllers/FlightController.h"

#include <chrono>

using namespace bjos;

uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}


//WARNING: blocks while no initial messages are received from the drone
//TODO: neater exceptions
void FlightController::init(BJOS *bjos) {
	bool ret = Controller::init(bjos, "flight", _data);
	if (!ret)
		throw ControllerInitializationError(this, "Controller::init failed");

	//open a uart connection to the pixhawk
	//NOTE: uses default uart="/dev/ttyAMA0", baudrate = "57600"
	serial_port = new Serial_Port();
	serial_port->start();
	//give the serial port some time
	usleep(100000);
	//check the status of the port
	if (not serial_port->status == 1) //SERIAL_PORT_OPEN
		throw ControllerInitializationError(this, "Serial port not open");

	//start read thread
	_read_thrd_running = true;
	_read_thrd = boost::thread(&FlightController::read_thread, this);

	// read_thread initialises 'initial_position' and signals this by setting _init_set
	std::cout << "Receiving initial position ...";
	while (_init_set == false) {
		//TODO: timeout
		usleep(100000); //10 Hz
		std::cout << " ...";
	}
	std::cout << " Received!" << std::endl;
	log.info("FlightController::init", "Initial position: xyz=[%.4f %.4f %.4f] vxvyvz=[%.4f %.4f %.4f]", initial_position.x, initial_position.y, initial_position.z, initial_position.vx, initial_position.y, initial_position.z);

	// write_thread initialises 'current_setpoint': all velocities 0
	_write_thrd = boost::thread(&FlightController::write_thread, this);

	//wait until write_thread starts
	while (_write_thrd_running == false)
		usleep(100000); //10 Hz

	//in order for the drone to react to the streamed setpoints, offboard mode has to be enabled
	int result = toggle_offboard_control(true);
	if (result == -1)
		throw ControllerInitializationError(this, "Could not set offboard mode: unable to write message on serial port");
	else if (result == 0)
		log.warn("FlightController::init", "double (de-)activation of offboard mode [ignored]");

	// Done!
}

void FlightController::load(bjos::BJOS *bjos) {
	Controller::load(bjos, "flight", _data);
}

void FlightController::read_thread() {
	while (_read_thrd_running)
	{
		read_messages();

		try {
			boost::this_thread::sleep_for(boost::chrono::microseconds(500)); //2000 Hz
		}
		catch (boost::thread_interrupted) {
			//if interrupt, stop and let the controller finish resources
			///Q: wha??
			return;
		}
	}

	return;
}

void FlightController::read_messages() {
	mavlink_message_t message;

	std::lock_guard<std::mutex> lock(serial_port_mutex);
	bool success = serial_port->read_message(message);
	
	if (success) {
		if (!_init_set) {
			system_id = message.sysid;
			autopilot_id = message.compid;
		}

		//Handle message per id
		switch (message.msgid) {
			case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			{
				log.info("FlightController::read_messages", "MAVLINK_MSG_LOCAL_POSITION_NED");
				
				//decode
				mavlink_local_position_ned_t local_position_ned;
				mavlink_msg_local_position_ned_decode(&message, &local_position_ned);

				//put position and velocity data into _data
				std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
				_data->pose.position.x = local_position_ned.x;
				_data->pose.position.y = local_position_ned.y;
				_data->pose.position.z = local_position_ned.z;
				_data->heading.velocity.vx = local_position_ned.vx;
				_data->heading.velocity.vy = local_position_ned.vy;
				_data->heading.velocity.vz = local_position_ned.vz;
			
				if (!_init_set) {
					mavlink_msg_local_position_ned_decode(&message, &initial_position);
					_init_set = true;
				}
				break;
			}
			case MAVLINK_MSG_ID_ATTITUDE:
			{
				log.info("FlightController::read_messages", "MAVLINK_MSG_ATTITUDE");

				//decode
				mavlink_attitude_t attitude;
				mavlink_msg_attitude_decode(&message, &attitude);

				//put attitude data into _data
				std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
				_data->pose.orientation.p = attitude.pitch;
				_data->pose.orientation.r = attitude.roll;
				_data->pose.orientation.y = attitude.yaw;
				_data->heading.angular_velocity.vp = attitude.pitchspeed;
				_data->heading.angular_velocity.vr = attitude.rollspeed;
				_data->heading.angular_velocity.vy = attitude.yawspeed;
				break;
			}
			default:
			{
				log.info("FlightController::read_messages", "Not handling this message: %" PRIu8, message.msgid);
			}
		}
	}
	else {
//                log.warn("",".");
	}

	return;
}

void FlightController::write_thread() {
	//prepare an initial setpoint: all velocities 0
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_BODY_NED;
	sp.vx = 0;
	sp.vy = 0;
	sp.vz = 0;
	sp.yaw_rate = 0;

	// set target
	current_setpoint = sp;
	
	// write a message and signal writing
	write_setpoint();
	_write_thrd_running = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz, otherwise it'll go into failsafe
	while (_write_thrd_running) {
		try {
			boost::this_thread::sleep_for(boost::chrono::milliseconds(100)); //Stream at 10 Hz
		}
		catch (boost::thread_interrupted) {
			//if interrupt, stop and let the controller finish resources
			///Q: wha??
			return;
		}

		write_setpoint();
	}

	return;
}

void FlightController::write_setpoint() {
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	std::lock_guard<std::mutex> lock(current_setpoint_mutex);
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if (not sp.time_boot_ms)
		sp.time_boot_ms = (uint32_t)(get_time_usec() / 1000);
	sp.target_system = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, autopilot_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	std::lock_guard<std::mutex> lock(serial_port_mutex);
	int len = serial_port->write_message(message);

	// check the write
	if (not len > 0)
                std::cout << ".";
	else {
		//TODO: log writing of setpoint per type_mask
		//log.info("FlightController::write_setpoint", "Wrote setpoint");
	}

	return;
}

//WARNING: do not use this function in multiple threads
int FlightController::toggle_offboard_control(bool flag) {
	static bool offboard = false;

	if (offboard == flag) return 0;
	else {
		// Prepare command for off-board mode
		/*
		mavlink_command_long_t com;
		com.target_system		= system_id;
		com.target_component	= autopilot_id;
		com.command				= MAV_CMD_NAV_GUIDED_ENABLE;
		com.confirmation		= true;
		com.param1				= (float)flag; // flag >0.5 => start, <0.5 => stop

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, autopilot_id, &message, &com);

		// Send
		std::lock_guard<std::mutex> lock(serial_port_mutex);
		int success = serial_port->write_message(message);
		*/
		bool success = true;
		if (success) {
			offboard = flag;
			return 1;
		}
		else {
			//write error
			return -1;
		}
	}
}

Pose FlightController::getPose() {
        std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
        return _data->pose;
}

Heading FlightController::getHeading() {
	std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
        return _data->heading;
}

//ALERT: can NOT be used to set roll, pitch, rollspeed or pitchspeed
int FlightController::setTarget(uint8_t type_mask, Pose pose, Heading heading) {
	//TODO: mutex lock writing and reading 'current_setpoint'
	mavlink_set_position_target_local_ned_t sp;

	sp.type_mask = type_mask;

	sp.x = pose.position.x;
	sp.y = pose.position.y;
	sp.z = pose.position.z;

	sp.vx = heading.velocity.vx;
	sp.vy = heading.velocity.vy;
	sp.vz = heading.velocity.vz;

	sp.yaw = pose.orientation.y;
	sp.yaw_rate = heading.angular_velocity.vy;

	std::lock_guard<std::mutex> lock(current_setpoint_mutex);
	current_setpoint = sp;
}

