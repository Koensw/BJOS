///Q: copyright notice for all files?

/**
 * @file flightController.h
 *
 * @brief Drone interface definitions
 *
 * Functions for sending and receiving commands to a drone via MAVLink
 *
 * @author Joep Linssen,	<joep.linssen@bluejayeindhoven.nl>
 */

#ifndef _BLUEJAY_FLIGHT_CONTROLLER_H
#define _BLUEJAY_FLIGHT_CONTROLLER_H

#define __STDC_FORMAT_MACROS 1

#include <mutex>
#include <atomic>

#include <boost/thread/thread.hpp>

#include <signal.h>
#include <time.h>
#include "sys/time.h"
#include <errno.h>
#include <string.h>
#include <inttypes.h>

#include "geometry.h"

#include "../bjos/bjos.h"
#include "../bjos/controller/controller.h"
#include "log.h"

#include "flight/serial_port.h"
#include "mavlink/include/mavlink/v1.0/common/mavlink.h" 

// ------------------------------------------------------------------------------
//   MAVLink info
// ------------------------------------------------------------------------------
/*
* MAVLink messages used per implemented function:
*	int setTargetPosition(...)	--- SET_POSITION_TARGET_LOCAL_NED with ~.type_mask = 3576
*	int setTargetVelocity(...)	--- SET_POSITION_TARGET_LOCAL_NED with ~.type_mask = 3527
*	int setCurrentPosition(...)	--- ATT_POS_MOCAP with ~.q = {1 0 0 0}
*	int setStreamFreq(...)		--- COMMAND_LONG with MAV_CMD_SET_MESSAGE_INTERVAL
*	void readMessages(...)		--- Decodes incoming messages of type as given by its argument
*
*	no further communicion is possible with the drone (as of now)
*		setYaw (via setCurrentAttitude(...)) is first to be implemented
*/

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

//				bit number:		87654321
#define SET_TARGET_POSITION		0b000001
#define SET_TARGET_VELOCITY		0b000010
#define SET_TARGET_YAW_ANGLE	0b000100
#define SET_TARGET_YAW_RATE		0b001000

namespace bjos {
	struct SharedFlightControllerData {
		//NOTE: class variables are double's not floats
		Pose pose; 
		Velocity velocity;		
	};

	class FlightController : public Controller { 
	public:
		FlightController() : _data(nullptr), _read_thrd_running(false), _write_thrd_running(false), system_id(0), autopilot_id(0), _init_set(false) {}

		/* get methods retrieve their payload from the private variables of the class
		 * the private variables are constantly updated via the threaded private readMessages() method
		 * therefor, a lock has to be applied when reading these values	 */
		float getRoll(); 
		float getPitch();
		float getYaw();	 

		float* getAttitude(); 
		float* getPosition(); 
		float* getVelocity(); 
		
		/**
		 * setTarget updates private variable current_setpoint 
		 * its first argument is a type_mask that specifies which of its other arguments should be used and which should be ignored, according to:
		 * bit 1: position
		 * bit 2: velocity
		 * bit 3: yaw
		 * bit 4: yaw_rate
		 * remaining bits unused
		 *
		 * With this header comes a set of bitmasks which should be used with this function: SET_TARGET_*		 
		 * These can be combined with bitwise &
		 *
		 * Example for velocity and yaw rate:
		 * uint8_t type_mask = SET_TARGET_VELOCITY & SET_TARGET_YAW_RATE;
		 */
		int setTarget(uint8_t type_mask, float xyx[3], float vxvyvz[3], float yaw_angle, float yaw_rate);

		int setCurrentPosition(float xyz[3]);	
		int setCurrentVelocity(float vxvyvz[3]);
		int setCurrentAttitude(float rpy[3]);

		int write_message(mavlink_message_t message);

		///Q: correct?
		/* Finalize this controller */
		~FlightController() {
			if (isMainInstance()) {
				//stop thread, wait for finish
				bool time_to_exit = true;
				_read_thrd_running = false;
				_write_thrd_running = false;
				_read_thrd_ptr->join();
				_write_thrd_ptr->join();
			}

			Controller::finalize<SharedFlightControllerData>();
		}

	private:
		Serial_Port *serial_port;
		Log log;

		/* Set offboard mode - has to be done in order to send setpoints */
		//NOTE: return -1 on double (de-)activation, return 0 on write error, return 1 on success;
		//TODO: neater errors
		int toggle_offboard_control(bool flag);

		//Used by messaging part
		int system_id;
		int autopilot_id;

		//Used to constantly send setpoints to the drone
		//TODO: make an array-form message, in order to meet the need for sending a path of ~20 setpoints
		mavlink_set_position_target_local_ned_t current_setpoint;

		//Used as reference for every setpoint (init_pos is considered [0, 0, 0] @ higher level, this is the abstractor)
		mavlink_local_position_ned_t initial_position;

		/* Initialize the main instance */
		void init(bjos::BJOS *bjos);
		/* load node is called for all childeren */
		void load(bjos::BJOS *bjos);

		SharedFlightControllerData *_data;

		//NOTE: only used by main instance
		boost::thread _read_thrd;
		boost::thread _write_thrd;
		std::atomic_bool _read_thrd_running;
		std::atomic_bool _write_thrd_running;
		/* threaded functions: */
		void read_thread();
		void write_thread();
		/* utility functions used by threads */
		void write_setpoint();
		void read_messages();
		/* initialiser check */
		std::atomic_bool _init_set;


		/* private 'defines' */
		static const uint16_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION	= 0b0000110111111000; //3576
		static const uint16_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY	= 0b0000110111000111; //3527
		static const uint16_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE	= 0b0000100111111111; //2559
		static const uint16_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE	= 0b0000010111111111; //1535

	};

}

#endif //_BLUEJAY_FLIGHT_CONTROLLER_H