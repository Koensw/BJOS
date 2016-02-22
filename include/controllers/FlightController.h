///Q: copyright notice for all files?

/**
 * @file flightController.h
 *
 * @brief Drone interface definitions
 *
 * Functions for sending commands to and receiving information from a drone via MAVLink
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
#include <errno.h>
#include <string.h>
#include <inttypes.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>

#include <iostream>

#include <utility>

#include "../libs/log.h"

#include "../bjos/bjos.h"
#include "../bjos/controller.h"
#include "../bjos/helpers/error.h"

#include "flight/serial_port.h"
#include "flight/raw_estimate.h"

#include "../libs/geometry.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef _WIN32
#include "mavlink\include\mavlink\v1.0\common\mavlink.h"
#else
#include <mavlink/v1.0/common/mavlink.h>
#endif

// -------------------------------------------------------------------------------------
//   MAVLink info - latest documentation for reference: https://pixhawk.ethz.ch/mavlink/
// -------------------------------------------------------------------------------------
/*
 * MAVLink messages used per implemented function:
 *	setTargetCF		        --- SET_POSITION_TARGET_LOCAL_NED
 *	setCurrentPosition	    --- VISION_POSITION_ESTIMATE
 *  toggle_offboard_control --- COMMAND_LONG (MAV_CMD_NAV_GUIDED_ENABLE)
 *  synchronize_time        --- SYSTEM_TIME
 *	readMessages            --- Decodes the following incoming messages:
 *                                      ATTITUDE, LOCAL_POSITION_NED, STATUSTEXT, SYSTEM_TIME, HIGHRES_IMU, EXTENDED_SYS_STATE
 */

#define SYS_ID 255
#define COMP_ID 255

// -------------------------------------------------------------------------------------
//   Defines
// -------------------------------------------------------------------------------------

// These typemasks are 'ignore-masks': 1 means ignore, while 0 means use
// They, thus, have to be combined with &
//
//						bit number:		    43210987654321
#define SET_TARGET_POSITION		15864 //0b0011110111111000
#define SET_TARGET_VELOCITY		15815 //0b0011110111000111
#define SET_TARGET_YAW_ANGLE	14847 //0b0011100111111111
#define SET_TARGET_YAW_RATE		13823 //0b0011010111111111
#define SET_TARGET_LAND         11719 //0b0010110111000111
#define SET_TARGET_TAKEOFF      7623  //0b0001110111000111

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
        //As a general rule: every pose information vector has its corresponding frame appended to its name (even in function arguments)
        /* Latest pose information from drone */
        Eigen::Vector3d positionNED;
        Eigen::Vector3d orientationNED;
        Eigen::Vector3d velocityNED;
        Eigen::Vector3d angularVelocityNED;
        
        /* The synchronized time of the Pixhawk */
        uint64_t syncBootTime; //ms
        uint64_t syncUnixTime; //ms
        uint64_t ourBootTime; //ms
        
		/* Offset between vision WF and drone WF (frame configuration flipped from drone NED) */
        Eigen::Vector3d visionPosOffset;
        double visionYawOffset;

        /* Latest setpoint sent to the drone */
        //TODO: make an array-form message, in order to meet the need for sending a path of ~20 setpoints
        mavlink_set_position_target_local_ned_t current_setpoint;
        
        /* Latest estimate sent to the drone */
        mavlink_vision_position_estimate_t vision_position_estimate;

        /* Stores last system time from the Pixhawk (normally should not be directly used, instead the synchronized time should be used) */
        mavlink_system_time_t sys_time;

        /* Current drone landed state */
        bool landed;

        /* Current vision syned state */
        bool _vision_sync;

        /* Flag for flight termination */
        bool terminateFlight;

        /* Raw IMU sensor data */
        IMUSensorData imuNED;
    };
    
    class FlightController : public Controller {
    public:
        FlightController();
        virtual ~FlightController();

        /* Equal to getPositionWF and getOrientationWF combined, held for backwards compatability. See typedef in geometry.h */
        Pose getPoseWF();
        
        Eigen::Vector3d getPositionNED();
        Eigen::Vector3d getPositionWF();

        Eigen::Vector3d getOrientationNED();
        Eigen::Vector3d getOrientationWF();
        //Eigen::Vector3d getOrientationCF();

        Eigen::Vector3d getVelocityNED();
        //Eigen::Vector3d getVelocityWF();
        //Eigen::Vector3d getVelocityCF();

        Eigen::Vector3d getAngularVelocityNED();
        //Eigen::Vector3d getAngularVelocityWF();
        //Eigen::Vector3d getAngularVelocityCF();

        /**
         * setTargetCF updates the shared variable current_setpoint (which is streamed to the Pixhawk)
         *
         * The first argument is a type_mask that specifies which of its other arguments should be used and which should be ignored
         * With this header comes a set of bitmasks which should be used with this function: SET_TARGET_*		 
         * These can be combined with bitwise &
         *
         * Example for velocity and yaw rate setpoint:
         * uint16_t type_mask = SET_TARGET_VELOCITY & SET_TARGET_YAW_RATE;
         */
        void setTargetCF(uint16_t type_mask, Eigen::Vector3d position, Eigen::Vector3d orientation, Eigen::Vector3d velocity,
                Eigen::Vector3d angularVelocity);

        /* Getters for current target */
        Eigen::Vector3d getTargetOrientationCF();
        Eigen::Vector3d getTargetVelocityCF();
        
        /* MAVLink struct to tuple of Vector3d converter for the current setpoint */
        std::tuple<Eigen::Vector3d, double, Eigen::Vector3d, double> getCurrentSetpointNED();
        std::tuple<Eigen::Vector3d, double, Eigen::Vector3d, double> getCurrentSetpointWF();

		/* At a given moment, the Kinect module calls this function with its current estimate of the drone position and its own rotation w.r.t. the magnetic north
		 * This module then uses this information as a constant base for the WF conversions */
		void syncVision(Eigen::Vector3d visionPosOffset, double visionYawOffset);

        /* set*EstimateWF functions are to be used by a computer vision algorithm supplying the drone with external absolute measurements of its states 
		 * These functions assume 'syncVision' is called beforehand */
        void setPositionEstimateWF(Eigen::Vector3d posEst);	
        void setAttitudeEstimateWF(double yawEst);
        
        /* Returns current landed state */
        bool isLanded();

        /* Terminate flight immediately */
        void terminateFlight();

        /* Returns IMU sensor data */
        IMUSensorData getIMUDataCF();
    private:
        /* Instance of the MAVLink serial communication handling class (uses UART) */
        Serial_Port *serial_port;
        
        /* Set offboard mode - has to be done in order to send setpoints */
        //NOTE: returns -1 on write error, returns 0 on double (de-)activation, returns 1 on success;
        int toggle_offboard_control(bool flag);
        
        /* Synchronizes the time with the Pixhawk (WARNING: this should only be done on primary load, to ensure that monotic time is always consistent!) */
        //NOTE: returns false on error, returns true on success;
        bool synchronize_time();
        
        /* Termination utility method */
        bool terminator();

        /* Used by messaging part to allow multiple drone streaming, and to just generally indicate which drone is being sent to */
        int system_id;
        int autopilot_id;
                
        /* Initialize the main instance */
        void init(bjos::BJOS *bjos);
        /* Load node is called for all childeren */
        void load(bjos::BJOS *bjos);
        
        /* An instance of the data that the main instance shares with its children */
        SharedFlightControllerData *_data;
        
        /* Frame conversion functions
         * ALERT: These functions do not access shared data on their own!
         * Please obey this rule when adding/changing any of the functions to prevent deadlocks
         *
         * The frames used in this program are defined in the Frame Specification.pdf file
         */
        //Eigen::Vector3d positionCFtoBodyNED(Eigen::Vector3d positionCF);
        Eigen::Vector3d CFtoBodyNED(Eigen::Vector3d vectorCF);
        Eigen::Vector3d positionNEDtoCF(Eigen::Vector3d positionNED, double yawNED);
        Eigen::Vector3d BodyNEDtoCF(Eigen::Vector3d vectorNED);
        Eigen::Vector3d NEDtoCF(Eigen::Vector3d vectorNED, double yawNED);
        Eigen::Vector3d positionWFtoNED(Eigen::Vector3d positionWF, Eigen::Vector3d visionPosOffset, double visionYawOffset);
        Eigen::Vector3d orientationWFtoNED(Eigen::Vector3d positionWF, double visionYawOffset);
        Eigen::Vector3d WFtoNED(Eigen::Vector3d vectorWF);
        Eigen::Vector3d positionNEDtoWF(Eigen::Vector3d positionNED, Eigen::Vector3d visionPosOffset, double visionYawOffset);
        Eigen::Vector3d orientationNEDtoWF(Eigen::Vector3d positionNED, double visionYawOffset);
        Eigen::Vector3d NEDtoWF(Eigen::Vector3d vectorNED, Eigen::Vector3d visionPosOffset);
      
        /* Thread handlers, only used by the main instance */
        boost::thread _read_thrd;
        boost::thread _write_thrd;
        std::atomic_bool _read_thrd_running;
        std::atomic_bool _write_thrd_running;
        /* Threaded functions */
        void read_thread();
        void write_thread();
        /* Utility functions used by threads */
        void write_setpoint();
        void write_estimate();
        void read_messages();
        
        /* Used to check if nothing is wrong with the MAVLink connection */
        std::atomic_bool _mavlink_received;

        /* Raw socket used by Philips (WARNING: sends raw struct data, is not portable and not needed because of our own communication layer: need fix later) */
        int _raw_sock;
        struct sockaddr_un _raw_sock_name;
    };
    
}

#endif //_BLUEJAY_FLIGHT_CONTROLLER_H
