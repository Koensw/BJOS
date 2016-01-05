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

// These typemasks are 'ignore-masks': 1 means ignore, while 0 means use
// They, thus, have to be combined with &
//
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
        /* Structs storing pose and heading of the drone in different frames; Local NED (Pixhawk), Control Frame and World Frame */
        Pose poseNED;
        Heading headingNED;
        //NOTE: position of drone in CF frame is always 0 (its a drone-fixed frame)
        Orientation orientationCF;
        Heading headingCF;
        Pose poseWF;
        Heading headingWF;
        
        /* The synchronized time of the Pixhawk */
        uint64_t syncBootTime; //ms
        uint64_t syncUnixTime; //ms
        
		/* Offset between vision WF and drone WF (frame configuration flipped from drone NED) */
        Point visionPosOffset;
        double visionYawOffset;

        //Used to constantly send setpoints to the drone
        //TODO: make an array-form message, in order to meet the need for sending a path of ~20 setpoints
        mavlink_set_position_target_local_ned_t current_setpoint;
        
        //Used to stream position and velocity estimates to the drone
        mavlink_vision_position_estimate_t vision_position_estimate;

        //used to retrieve last system time from Pixhawk (normally should not be directly used, instead the synchronized time should be used)
        mavlink_system_time_t sys_time;

        //Tracks if the drone is landed or not
        bool landed;

        //raw sensors data
        IMUSensorData imuNED;
    };
    
    class FlightController : public Controller {
    public:
        FlightController();
        ~FlightController();

        /* Returns a Pose struct that contains Point and Orientation structs */
        Pose getPoseNED();
        Pose getPoseWF();
        // Control Frame always has position 0, to remind users of these get functions of that, only a Orientation struct is returned
        Orientation getOrientationCF();

        /* Returns a Heading struct that contains Velocity and AngularVelocity structs */
        Heading getHeadingNED();
        Heading getHeadingCF();
		Heading getHeadingWF();
        
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
        void setTargetCF(uint16_t type_mask, Pose poseCF, Heading headingCF);
        //FIXME: reference frame is missing (and name is not fully compliant)
        std::pair<Pose, Heading> getCurrentSetpoint();

		/* At a given moment, the Kinect module calls this function with its current estimate of the drone position and its own rotation w.r.t. the magnetic north
		 * The drone then uses this information as a constant base for the set*EstimateWF functions */
		void syncVision(Point visionPosOffset, double visionYawOffset);

        /* set*EstimateWF functions are to be used by a computer vision algorithm supplying the drone with external absolute measurements of its states 
		 * These functions assume 'syncVision' is called beforehand */
        void setPositionEstimateWF(Point posEst);	
        void setAttitudeEstimateWF(Orientation attEst);
        
        // Return if landed
        bool isLanded();

        /* Return raw sensor data */
        IMUSensorData getIMUDataCF();
        
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
         * These functions do not access shared data!
         * 
         * The frames used in this program are defined in the Frame Specification.pdf file!
         *
         * yaw_P is the yaw as returned by the Pixhawk, this can be retreived from _data->headingNED.orientation.y
         */
        Point CFtoNED(Point pointCF, double yaw_P, Point pointP);
        Velocity CFtoNED(Velocity headingCF, double yaw_P);
        Point NEDtoCF(Point pointNED, double yaw_P, Point pointP);
        Velocity NEDtoCF(Velocity headingNED, double yaw_P);        
		Point NEDtoWF(Point pointNED, Point visionPosOffset);
		//Velocity NEDtoWF(Velocity velocityNED);
        
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
        void write_estimate();
        void read_messages();
        /* initialiser check */
        std::atomic_bool _init_set;
    };
    
}

#endif //_BLUEJAY_FLIGHT_CONTROLLER_H
