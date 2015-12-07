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

#include "../libs/geometry.h"
#include "../libs/log.h"

#include "../bjos/bjos.h"
#include "../bjos/controller/controller.h"
#include "../bjos/helpers/error.h"

#include "flight/serial_port.h"

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

//						bit number:	  210987654321
#define SET_TARGET_POSITION		0b0000110111111000 //3576
#define SET_TARGET_VELOCITY		0b0000110111000111 //3527
#define SET_TARGET_YAW_ANGLE	0b0000100111111111 //2559
#define SET_TARGET_YAW_RATE		0b0000010111111111 //1535

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
        /* Structs storing pose and heading of the drone in different frames; Local NED (Pixhawk), Control Frame and World Frame */
        Pose poseNED;
        Heading headingNED;
        //NOTE: position of drone in CF frame is always 0 (its a drone-fixed frame)
        Orientation orientationCF;
        Heading headingCF;
        Pose poseWF;
        Heading headingWF;
        
        //Used to constantly send setpoints to the drone
        //TODO: make an array-form message, in order to meet the need for sending a path of ~20 setpoints
        mavlink_set_position_target_local_ned_t current_setpoint;
        
        //raw sensors data
        IMUSensorData imuNED;
    };
    
    class FlightController : public Controller {
    public:
        FlightController() : system_id(0), autopilot_id(0),_data(nullptr), _read_thrd_running(false), _write_thrd_running(false), _init_set(false) {}
        
        float getRoll();
        float getPitch();
        float getYaw();
        
        /* Returns a Pose struct that contains Point and Orientation structs */
        Pose getPoseNED();
        // Control Frame always has position 0, to remind users of these get functions of that, only a Orientation struct is returned
        Orientation getOrientationCF();
        /* Returns a Heading struct that contains Velocity and AngularVelocity structs */
        Heading getHeadingNED();
        Heading getHeadingCF();
        
        /**
         * setTargetCF updates private variable current_setpoint
         * its first argument is a type_mask that specifies which of its other arguments should be used and which should be ignored
         *
         * With this header comes a set of bitmasks which should be used with this function: SET_TARGET_*		 
         * These can be combined with bitwise &
         *
         * Example for velocity and yaw rate:
         * uint16_t type_mask = SET_TARGET_VELOCITY & SET_TARGET_YAW_RATE;
         */
        void setTargetCF(uint16_t type_mask, Pose poseCF, Heading headingCF);
        //FIXME: reference frame is missing (and name is not fully compliant)
        std::pair<Pose, Heading> getCurrentSetpoint();
        
        /* setCurrent* functions are to be used by a computer vision algorithm supplying the drone with external absolute measurements of its states */
        void setCurrentPositionWF(float xyz[3]);	
        void setCurrentVelocityWF(float vxvyvz[3]);
        void setCurrentAttitudeWF(float rpy[3]);
        
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
                    throw ControllerInitializationError(this, "Could not set offboard mode: unable to write message on serial port");
                else if (result == 0)
                    Log::warn("FlightController::init", "double (de-)activation of offboard mode [ignored]");
                
                //stop threads, wait for finish
                _read_thrd_running = false;
                _write_thrd_running = false;
                _read_thrd.interrupt();
                _write_thrd.interrupt();
                _read_thrd.join();
                _write_thrd.join();
                
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
         * yaw_P is the yaw as returned by the Pixhawk, this can be retreived from _data->headingNED.orientation.y
         */
        Point CFtoNED(Point pointCF, double yaw_P, Point pointP);
        Velocity CFtoNED(Velocity headingCF, double yaw_P);
        Point NEDtoCF(Point pointNED, double yaw_P, Point pointP);
        Velocity NEDtoCF(Velocity headingNED, double yaw_P);
        /* World Frame conversions ommitted for now (since no module uses WF yet) */
        
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
