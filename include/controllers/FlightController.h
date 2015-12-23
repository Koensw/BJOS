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

#include <iostream>

#include <utility>

#include "../libs/log.h"

#include "../bjos/bjos.h"
#include "../bjos/controller/controller.h"
#include "../bjos/helpers/error.h"

#include "flight/serial_port.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef _WIN32
#include "mavlink\include\mavlink\v1.0\common\mavlink.h"
#else
#include <mavlink/v1.0/common/mavlink.h>
#endif

// ------------------------------------------------------------------------------
//   MAVLink info
// ------------------------------------------------------------------------------
/*
 * MAVLink messages used per implemented function:
 *	int setTargetCF(...)		--- SET_POSITION_TARGET_LOCAL_NED
 *	int setCurrentPosition(...)	--- ATT_POS_MOCAP with ~.q = {1 0 0 0}
 *	void readMessages(...)		--- Decodes incoming ATTITUDE and LOCAL_POSITION_NED messages
 *
 *	no further communication is possible with the drone (as of now)
 */

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

//						bit number:			 210987654321
#define SET_TARGET_POSITION		3576 //0b0000110111111000
#define SET_TARGET_VELOCITY		3527 //0b0000110111000111
#define SET_TARGET_YAW_ANGLE	2559 //0b0000100111111111
#define SET_TARGET_YAW_RATE		1535 //0b0000010111111111

/* helper function */
uint64_t get_time_usec();

namespace bjos {
    //TODO: move this to a separate types file when we use more types
    struct IMUSensorData{
        uint64_t time; //in usec
        double ax; //in m/s^2
        double ay; //in m/s^2
        double az; //in m/s^2
        double gx; //in rad/s
        double gy; //in rad/s
        double gz; //in rad/s
    };
    
    struct SharedFlightControllerData {
        //NOTE: class variables are double's not floats
        Eigen::Vector3d positionNED;
        Eigen::Vector3d orientationNED;
        Eigen::Vector3d velocityNED;
        Eigen::Vector3d angularVelocityNED;
        
        /* The synchronized time of the Pixhawk */
        uint64_t syncBootTime; //ms
        uint64_t syncUnixTime; //ms
        
        //Used to constantly send setpoints to the drone
        //TODO: make an array-form message, in order to meet the need for sending a path of ~20 setpoints
        mavlink_set_position_target_local_ned_t current_setpoint;
        
        //used to retrieve last system time from Pixhawk (normally should not be directly used, instead the synchronized time should be used)
        mavlink_system_time_t sys_time;
        
        //raw sensors data
        IMUSensorData imuNED;
    };
    
    class FlightController : public Controller {
    public:
        FlightController() : system_id(0), autopilot_id(0),_data(nullptr), _read_thrd_running(false), _write_thrd_running(false), _init_set(false) {}
        

        /* Position, orientation, velocity and angular velocity methods */

        /* Positions */
        Eigen::Vector3d getPositionNED();
        Eigen::Vector3d getPositionWF();

        /* Orientations, where x = roll, y = pitch, z = yaw */
        Eigen::Vector3d getOrientationNED();
        Eigen::Vector3d getOrientationWF();
        Eigen::Vector3d getOrientationCF();

        /* Velocities */
        Eigen::Vector3d getVelocityNED();
        Eigen::Vector3d getVelocityWF();
        Eigen::Vector3d getVelocityCF();

        /* Angular velocities, where x = roll, y = pitch, z = yaw */
        Eigen::Vector3d getAngularVelocityNED();
        Eigen::Vector3d getAngularVelocityWF();
        Eigen::Vector3d getAngularVelocityCF();

        /**
         * setTargetCF updates private variable current_setpoint
         * its first argument is a type_mask that specifies which of its other arguments should be used and which should be ignored
         *
         * With this header comes a set of bitmasks which should be used with this function: SET_TARGET_*		 
         * These can be combined with bitwise &
         *
         * Example for velocity and yaw rate setpoint:
         * uint16_t type_mask = SET_TARGET_VELOCITY & SET_TARGET_YAW_RATE;
         */
        void setTargetCF(uint16_t type_mask, Eigen::Vector3d position, Eigen::Vector3d orientation, Eigen::Vector3d velocity,
                Eigen::Vector3d angularVelocity);

        Eigen::Vector3d getTargetOrientationCF();
        Eigen::Vector3d getTargetVelocityCF();
        
        /* setCurrent* functions are to be used by a computer vision algorithm supplying the drone with external absolute measurements of its states */
        void setCurrentPositionWF(Eigen::Vector3d position);	
        void setCurrentVelocityWF(Eigen::Vector3d velocity);
        void setCurrentAttitudeWF(Eigen::Vector3d angularVelocity);
        
        /* Return raw sensor data */
        IMUSensorData getIMUDataCF();
        
        ///Q: correct?
        /* Finalize this controller */
        ~FlightController() {
            std::cout << "FlightController destructor" << std::endl;
            if (isMainInstance()) {
                //disable offboard control mode if not already
                int result = toggle_offboard_control(false);
                if (result == -1)
                    Log::error("FlightController::init",
                            "Could not set offboard mode: unable to write message on serial port");
                else if (result == 0)
                    Log::warn("FlightController::init",
                            "double (de-)activation of offboard mode [ignored]");
                
                //stop threads, wait for finish
                _read_thrd_running = false;
                _write_thrd_running = false;
                _read_thrd.interrupt();
                _write_thrd.interrupt();
                _read_thrd.join();
                _write_thrd.join();

				//if the read_trhd is still waiting to receive an initial MAVLink message:
                //close it and throw an ControllerInitializationError
				if (_init_set == false) {
					_init_set = true;
					Log::error("FlightController::init",
                            "Did not receive any MAVLink messages, check physical connections and make sure it is running on the right port!");
				}
                
                //stop the serial_port and clean up the pointer
                serial_port->stop();
                delete serial_port;
            }
            
            Controller::finalize<SharedFlightControllerData>();
        }
        
    private:
        Serial_Port *serial_port;
        
        /* Set offboard mode - has to be done in order to send setpoints */
        //NOTE: returns -1 on write error, returns 0 on double (de-)activation, returns 1 on success;
        int toggle_offboard_control(bool flag);
        
        /* Synchronizes the time with the Pixhawk (WARNING: this should only be done on primary load, to ensure that monotic time is always consistent!) */
        //NOTE: returns false on error, returns true on success;
        bool synchronize_time();
        
        //Used by messaging part
        int system_id;
        int autopilot_id;
        
        //Used as reference for every setpoint (init_pos is considered [0, 0, 0] @ higher level, this is the abstractor)
        mavlink_local_position_ned_t initial_position;
        
        /* Initialize the main instance */
        void init(bjos::BJOS *bjos);
        /* load node is called for all childeren */
        void load(bjos::BJOS *bjos);
        
        SharedFlightControllerData *_data;
        
        /* Frame conversion functions 
         * 
         * The frames used in this program are defined in the Frame Specification.pdf file!
         *
         * Make sure you have a mutex lock on the shared data before calling this!
         */
        //Eigen::Vector3d positionCFtoBodyNED(Eigen::Vector3d positionCF);
        Eigen::Vector3d CFtoBodyNED(Eigen::Vector3d vectorCF);
        Eigen::Vector3d positionNEDtoCF(Eigen::Vector3d positionNED);
        Eigen::Vector3d NEDtoCF(Eigen::Vector3d vectorNED);
        Eigen::Vector3d positionWFtoNED(Eigen::Vector3d positionWF);
        Eigen::Vector3d WFtoNED(Eigen::Vector3d vectorWF);
        Eigen::Vector3d positionNEDtoWF(Eigen::Vector3d positionNED);
        Eigen::Vector3d NEDtoWF(Eigen::Vector3d vectorNED);

        
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
    };
    
}

#endif //_BLUEJAY_FLIGHT_CONTROLLER_H
