/**
 * @file flightController.cpp
 *
 * @brief Drone interface functions
 *
 * Functions for sending commands to and receiving information from a drone via MAVLink
 *
 * @author Joep Linssen 	    <joep.linssen@bluejayeindhoven.nl>
 * @author Koen Wolters         <koen.wolters@bluejayeindhoven.nl>
 * @author Wouter van der Stoel <wouter@bluejayeindhoven.nl>
 */

#include "controllers/FlightController.h"

#include <chrono>
#include <sstream>

using namespace bjos;
using namespace bjcomm;

uint64_t get_time_usec(clockid_t clk_id)
{
    struct timespec _time_stamp;
    clock_gettime(clk_id, &_time_stamp);
    return _time_stamp.tv_sec * 1000000ULL + _time_stamp.tv_nsec / 1000ULL;
}

FlightController::FlightController() : system_id(0), autopilot_id(0), _data(nullptr), _read_thrd_running(false), _write_thrd_running(false), _mavlink_received(false) {}

FlightController::~FlightController() {
    if(!Controller::isAvailable()) return;
    
    if (isMainInstance()) {
        shared_data_mutex->lock();
        bool force_failsafe = _data->force_failsafe;
        shared_data_mutex->unlock();
        
        //disable offboard control mode if not already and we should not force a failsafe hang
        if(!force_failsafe){
            int result = toggle_offboard_control(false);
            if (result == -1)
                Log::error("FlightController::init", "Could not set offboard mode: unable to write message on serial port");
            else if (result == 0)
                Log::warn("FlightController::init", "double (de-)activation of offboard mode [ignored]");
        }else{
            Log::error("FlightController::init", "Not toggling offboard, because we do a failsafe shutdown and we want the Pixhawk to try and figure this out!");
        }

        //stop threads
        _read_thrd_running = false;
        _write_thrd_running = false;
        _read_thrd.interrupt();
        _write_thrd.interrupt();

        //wait till threads are finished
        _read_thrd.join();
        _write_thrd.join();
        
        //stop the raw stream
        close(_raw_sock);

        //stop the serial_port and clean up the pointer
        serial_port->stop();
        delete serial_port;
    }

    Controller::finalize<SharedFlightControllerData>();
}

void FlightController::init(bjos::BJOS *bjos) {
    // -------------------------------------------------------------------------------------------------
    //   Initialize the controller in BJOS
    // -------------------------------------------------------------------------------------------------
    bool ret = Controller::init(bjos, "flight", _data);
    if (!ret)
        throw ControllerInitializationError(this, "Controller::init failed");

    // -------------------------------------------------------------------------------------------------
    //   Shared data initialisation
    // -------------------------------------------------------------------------------------------------
    _data->landed = false;
    _data->write_estimate = false; //disable writing estimate by default (should be explicitly enabled!)
    _data->write_thrust_setpoint = false; //disable writing thrust setpoint by default (should be explicitly enabled!)                                      
    _data->end_thrust_setpoint = false;
    _data->kill_motors = false;
    _data->force_failsafe = false;
    _data->battery_percentage = 1; //assume full battery when not known...
    _data->vision_sync = false;
    _data->do_reboot = false;
    _data->write_param = false;
    _data->arming_flag = 0;
    
    //set vision_position_estimate to non-valid
    _data->vision_position_estimate.usec = 0;
    _data->vision_position_estimate.x = NAN;
    _data->vision_position_estimate.y = NAN;
    _data->vision_position_estimate.z = NAN;
    _data->vision_position_estimate.roll = NAN;
    _data->vision_position_estimate.pitch = NAN;
    _data->vision_position_estimate.yaw = NAN;
    _data->visionYawOffset = 0;
    _data->visionPosOffset = {0, 0, 0};
    
    // -------------------------------------------------------------------------------------------------
    //   UART to Pixhawk (NOTE: uses beefed up baudrate = "921600" on uart="/dev/ttyAMA0")
    // -------------------------------------------------------------------------------------------------
    serial_port = new Serial_Port();
    serial_port->start();
    //give the serial port some time
    usleep(100000);
    //check the status
    if (serial_port->status != 1) //1 == SERIAL_PORT_OPEN
        throw ControllerInitializationError(this, "Serial port can't be opened");
 
    // -------------------------------------------------------------------------------------------------
    //   Raw unix domain socket (WARNING: this is needed for Philips, this should not be hardcoded here) 
    // -------------------------------------------------------------------------------------------------
    _raw_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (_raw_sock < 0)
        throw ControllerInitializationError(this, "Cannot open raw socket");
    
    _raw_sock_name.sun_family = AF_UNIX;
    strcpy(_raw_sock_name.sun_path, "/tmp/bluejay/modules/philips-localization");
    
    // -------------------------------------------------------------------------------------------------
    //   Read thread
    // -------------------------------------------------------------------------------------------------
    _read_thrd_running = true;
    _read_thrd = boost::thread(&FlightController::read_thread, this);
    
    //If any MAVLink is received this is signaled with _mavlink_received
    unsigned int n = 0;
    unsigned int timeout = 20; //deciseconds
    while (_mavlink_received == false && n < timeout) {
		n++;
        usleep(100000); //10 Hz
    }
	if (n == timeout)
		throw ControllerInitializationError(this, "Did not receive any MAVLink messages");
    
    // -------------------------------------------------------------------------------------------------
    //   Write thread (initializes with all 0 velocity&yaw_rate setpoint)
    // -------------------------------------------------------------------------------------------------
    _write_thrd = boost::thread(&FlightController::write_thread, this);
    while (_write_thrd_running == false)
        usleep(100000); //10 Hz
    
    //NOTE: no error check because this never fails if the serial_port is initialized correctly
        
    // -------------------------------------------------------------------------------------------------
    //   Syncronize time with Pixhawk
    // -------------------------------------------------------------------------------------------------
    int result = synchronize_time();
    if(result == -1)
        throw ControllerInitializationError(this,
                "Could not synchronize time with the Pixhawk: error with connection");
        
    // -------------------------------------------------------------------------------------------------
    //   Enable offboard mode (this needs to be enabled manually as well in order for the drone to move)
    // -------------------------------------------------------------------------------------------------
    //WARNING: this is currently done by the RC, unless we are in posix simulation mode
#ifndef __arm__
    result = toggle_offboard_control(true);
    if (result == -1)
        throw ControllerInitializationError(this,
                "Could not set offboard mode: unable to write message on serial port");
    else if (result == 0)
        Log::warn("FlightController::init", "double (de-)activation of offboard mode [ignored]");
#endif
    // Done!
}

void FlightController::load(bjos::BJOS *bjos) {
    Log::info("FlightController::load", "New instance loaded");
    Controller::load(bjos, "flight", _data);
}

void FlightController::read_thread() {
    uint64_t prev_time = get_time_usec(CLOCK_MONOTONIC);
    
    while (_read_thrd_running) {
        shared_data_mutex->lock();
        bool force_failsafe = _data->force_failsafe;
        shared_data_mutex->unlock();
        
        try {
            uint64_t time_diff = get_time_usec(CLOCK_MONOTONIC) - prev_time;
            if (time_diff < 10ll) boost::this_thread::sleep_for(boost::chrono::microseconds(10ll - time_diff));
            prev_time = get_time_usec(CLOCK_MONOTONIC);
            
            //stop trying to read info from pixhawk in failsafe, because we cannot trust this anymore 
            if(force_failsafe) usleep(100000);
            else read_messages();
            
            //boost::this_thread::sleep_for(boost::chrono::microseconds(500)); //2000 Hz
        }
        catch (boost::thread_interrupted) {
            //if interrupt, stop and let the controller finish resources
            //TODO: finish resources?
            return;
        }
    }
    
    return;
}

void FlightController::read_messages() {
    static int errors = 0;
    
    mavlink_message_t message;
    
    int success = serial_port->read_message(message);
    
    if (success > 0) {
        errors = 0;
                
        if (!_mavlink_received) {
            system_id = message.sysid;
            autopilot_id = message.compid;
            
            shared_data_mutex->lock();
            _data->system_id = message.sysid;
            _data->autopilot_id = message.compid;
            shared_data_mutex->unlock();
        }
        
        //get current time
        shared_data_mutex->lock();
        uint64_t pixBootTime = _data->syncBootTime;
        uint64_t rpiBootTime = _data->ourBootTime;
        shared_data_mutex->unlock();
        
        //stringstream utility for sending bjcomm messages
        std::stringstream sstr;

        //Handle message per id
        switch (message.msgid) {
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            {                             
                //decode
                mavlink_local_position_ned_t local_position_ned;
                mavlink_msg_local_position_ned_decode(&message, &local_position_ned);
                
                //put position and velocity data into _data
                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                _data->positionNED[0] = local_position_ned.x;
                _data->positionNED[1] = local_position_ned.y;
                _data->positionNED[2] = local_position_ned.z;
                _data->velocityNED[0] = local_position_ned.vx;
                _data->velocityNED[1] = local_position_ned.vy;
                _data->velocityNED[2] = local_position_ned.vz;

                //only send bjcomm messages when vision has been synced and therefor WF is enabled
                if (_data->vision_sync) {
                    Message msg("position_estimate");
                    Eigen::Vector3d wf = positionNEDtoWF(Eigen::Vector3d(local_position_ned.x, local_position_ned.y, local_position_ned.z), _data->visionPosOffset, _data->visionYawOffset);
                    sstr.clear();
                    sstr << wf[0] << " " << wf[1] << " " << wf[2];
                    msg.setData(sstr.str());
                    //Log::info("flightcontroller::read_messages", sstr.str()+" - %f %f %f - %f", _data->visionPosOffset[0], _data->visionPosOffset[1], _data->visionPosOffset[2], _data->visionYawOffset);
                    send_state_message(msg);

                    msg = Message("velocity_estimate");
                    //FIXME: missing conversion here...
                    sstr.clear();
                    sstr.str("");
                    sstr << local_position_ned.vx << " " << local_position_ned.vy << " " << local_position_ned.vz;
                    msg.setData(sstr.str());
                    send_state_message(msg);
                }
                
                if (!_mavlink_received) _mavlink_received = true;
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE:
            {                           
                //decode
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&message, &attitude);     

                //Log::info("FlightController::read_messages", "attitude: %.2f %.2f %.2f", attitude.roll, attitude.pitch, attitude.yaw);

                //bjcomm message handling
                if (_data->vision_sync) {
                    Message msg("attitude_estimate");
                    Eigen::Vector3d wf = orientationNEDtoWF(Eigen::Vector3d(attitude.roll, attitude.pitch, attitude.yaw), _data->visionYawOffset);
                    sstr.clear();
                    sstr << wf[0] << " " << wf[1] << " " << wf[2];
                    msg.setData(sstr.str());
                    send_state_message(msg);
                }

                
                /*msg = Message("attitude_rate_estimate");
                sstr.clear();
                sstr << attitude.rollspeed << " " << attitude.pitchspeed << " " << attitude.yawspeed;
                msg.setData(sstr.str());
                send_state_message(msg);    */
                
                //put attitude data into _data
                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);

                _data->orientationNED[0] = attitude.roll;
                _data->orientationNED[1] = attitude.pitch;
                _data->orientationNED[2] = attitude.yaw;
                _data->angularVelocityNED[0] = attitude.rollspeed;
                _data->angularVelocityNED[1] = attitude.pitchspeed;
                _data->angularVelocityNED[2] = attitude.yawspeed;

                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            {                       
                //decode
                mavlink_attitude_quaternion_t attitude;
                mavlink_msg_attitude_quaternion_decode(&message, &attitude);
                
                //create a quaternion
                Eigen::Quaterniond q(attitude.q1, attitude.q2, attitude.q3, attitude.q4);
                //double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
                std::swap(q.x(), q.y());
                q.z() = -q.z();

                //printf("%f %f\n", yaw, q.z());

                //send the attitude to raw stream
                flight_raw_estimate raw_estimate;
                raw_estimate.type = FLIGHT_RAW_ORIENTATION;
                raw_estimate.time = get_time_usec(CLOCK_MONOTONIC)/1000ULL; //time_boot_ms - pixBootTime + rpiBootTime;
                raw_estimate.data[0] = q.w();
                raw_estimate.data[1] = q.x();
                raw_estimate.data[2] = q.y();
                raw_estimate.data[3] = q.z();
                sendto(_raw_sock, &raw_estimate, sizeof(raw_estimate), 0, (struct sockaddr *) &_raw_sock_name, SUN_LEN(&_raw_sock_name));

                //Convert quaternions to euler angles
                //Log::info("TEMP", "%.2f %.2f %.2f %.2f --- %.2f %.2f %.2f %.2f", attitude.q1, attitude.q2, attitude.q3, attitude.q4, q2.w(), q2.x(), q2.y(), q2.z());
                //Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

                //Log::info("FlightController::read_messages", "QtoEuler: %.2f %.2f %.2f", euler[0], euler[1], euler[2]);

                break;
            }
            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            {                
                //decode
                mavlink_position_target_local_ned_t setpoint;
                mavlink_msg_position_target_local_ned_decode(&message, &setpoint);

                //send over bjcomm
                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                if (_data->vision_sync) {
                    Eigen::Vector3d wf = positionNEDtoWF(Eigen::Vector3d(setpoint.x, setpoint.y, setpoint.z), _data->visionPosOffset, _data->visionYawOffset);
                    sstr.clear();
                    //FIXME: Sometimes the target is very far away, causing the graphs to become crap
                    // This value should in our use case always be less than 100
                    if (wf.norm() < 100) {
                        Message msg("position_setpoint");
                        sstr << wf[0] << " " << wf[1] << " " << wf[2];
                        msg.setData(sstr.str());
                        send_state_message(msg);
                    }

                    Message msg("velocity_setpoint");
                    //FIXME: missing conversion here...
                    sstr.clear();
                    sstr << setpoint.vx << " " << setpoint.vy << " " << setpoint.vz;
                    msg.setData(sstr.str());
                    send_state_message(msg);
                }
                break;
            }

            case MAVLINK_MSG_ID_STATUSTEXT:
            {
                mavlink_statustext_t statustext;
                mavlink_msg_statustext_decode(&message, &statustext);
                
                Log::info("FlightController::read_messages", "[status]\t%s", statustext.text);
                break;
            }
            case MAVLINK_MSG_ID_SYS_STATUS:
            {                
                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                mavlink_sys_status_t sys_state;
                mavlink_msg_sys_status_decode(&message, &sys_state);
                if(static_cast<int>(sys_state.battery_remaining) > 0) _data->battery_percentage = static_cast<double>(sys_state.battery_remaining)/100.0;

                Message msg("battery_remaining");
                sstr.clear();
                sstr << _data->battery_percentage;
                msg.setData(sstr.str());
                send_state_message(msg);

                msg = Message("sensors_valid");
                sstr.str("");
                sstr.clear();
                sstr << sys_state.errors_count1;  //Hijacked MAVLink message field. TODO make a seperate one
                msg.setData(sstr.str());
                send_state_message(msg);

                break;
            }
            case MAVLINK_MSG_ID_SYSTEM_TIME:
            {
                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                mavlink_system_time_t sys_time;
                mavlink_msg_system_time_decode(&message, &sys_time);
                _data->sys_time = sys_time;
                
                break;
            }
            case MAVLINK_MSG_ID_HIGHRES_IMU:
            {
                //decode
                mavlink_highres_imu_t highres_imu;
                mavlink_msg_highres_imu_decode(&message, &highres_imu);
                
                //send the acc to raw stream
                flight_raw_estimate raw_estimate;
                raw_estimate.type = FLIGHT_RAW_ACC;
                raw_estimate.time = highres_imu.time_usec/1000ULL - pixBootTime + rpiBootTime;
                raw_estimate.data[0] = highres_imu.xacc; 
                raw_estimate.data[1] = highres_imu.yacc; 
                raw_estimate.data[2] = highres_imu.zacc; 
                sendto(_raw_sock, &raw_estimate, sizeof(raw_estimate), 0, (const sockaddr *) &_raw_sock_name, SUN_LEN(&_raw_sock_name));
                
                //send the acc to raw stream
                raw_estimate.type = FLIGHT_RAW_GYRO;
                raw_estimate.time = highres_imu.time_usec/1000ULL - pixBootTime + rpiBootTime;
                raw_estimate.data[0] = highres_imu.xgyro; 
                raw_estimate.data[1] = highres_imu.ygyro; 
                raw_estimate.data[2] = highres_imu.zgyro; 
                sendto(_raw_sock, &raw_estimate, sizeof(raw_estimate), 0, (const sockaddr *) &_raw_sock_name, SUN_LEN(&_raw_sock_name));
                
                /*std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                _data->imuNED.time = highres_imu.time_usec/1000ULL;
                _data->imuNED.ax = highres_imu.xacc;
                _data->imuNED.ay = highres_imu.yacc;
                _data->imuNED.az = highres_imu.zacc;
                _data->imuNED.gx = highres_imu.xgyro;
                _data->imuNED.gy = highres_imu.ygyro;
                _data->imuNED.gz = highres_imu.zgyro;*/
                break;
            }
            case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
            {
                mavlink_extended_sys_state_t extended_sys_state;
                mavlink_msg_extended_sys_state_decode(&message, &extended_sys_state);

                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                switch (extended_sys_state.landed_state) {
                    case MAV_LANDED_STATE_ON_GROUND:
                    {
                        _data->landed = true;
                        break;
                    }
                    case MAV_LANDED_STATE_IN_AIR:
                    {
                        _data->landed = false;
                        break;
                    }
                    case MAV_LANDED_STATE_UNDEFINED:
                    default:
                    {
                        Log::info("FlightController::read_messages", "Unable to handle landed state %" PRIu8, extended_sys_state.landed_state);
                    }
                }
                break;
            }
            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            {
                mavlink_servo_output_raw_t servo_output_raw;
                mavlink_msg_servo_output_raw_decode(&message, &servo_output_raw);
                
                float servo_output_percentage[4];
                servo_output_percentage[0] = ((float)servo_output_raw.servo1_raw - 1000) / 1000.0;
                servo_output_percentage[1] = ((float)servo_output_raw.servo2_raw - 1000) / 1000.0;
                servo_output_percentage[2] = ((float)servo_output_raw.servo3_raw - 1000) / 1000.0;
                servo_output_percentage[3] = ((float)servo_output_raw.servo4_raw - 1000) / 1000.0;

                Message msg("engine_power");
                sstr.clear();
                sstr << servo_output_percentage[2] << " " << servo_output_percentage[0] << " " << servo_output_percentage[1] << " " << servo_output_percentage[3];
                msg.setData(sstr.str());
                send_state_message(msg);
                break;
            }
            case MAVLINK_MSG_ID_PARAM_VALUE:
            {
                Log::info("FlightController::read_param", "Received requested parameter");
                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                _data->param_read = message;
                _data->read_param_response = true;
                break;
            }
            default:
            {
                //Log::info("FlightController::read_messages", "Not handling this message: %" PRIu8, message.msgid);
            }
        }
    } else if(success == -1 &&_mavlink_received) { 
        //Mid-execution MAVLink loss check (so we already received MAVLink at least once)
        //On timeout: shutdown
        errors++;
        Log::warn("FlightController:read_messages", "Serial port error %i!", errors);
        if (errors > 5) { 
            Log::error("FlightController:read_messages", "MAVLink connection timed out");
            
            //force failsafe mode and request shutdown
            forceFailsafe();
            bjos::BJOS::getOS()->shutdown(); 
        }
    }
}

void FlightController::write_thread() {
    //prepare an initial setpoint: land if landed, hold if in air
    mavlink_set_position_target_local_ned_t sp;
    
    if (isLanded()) {
        sp.type_mask = SET_TARGET_LAND;
    }
    else {
        sp.type_mask = SET_TARGET_VELOCITY &
            SET_TARGET_YAW_RATE;      
    }

    sp.coordinate_frame = MAV_FRAME_BODY_NED;
    sp.vx = 0;
    sp.vy = 0;
    sp.vz = 0;
    sp.yaw_rate = 0;
    
    // set target
    _data->current_setpoint = sp;
    
    // write a message and signal writing
    write_setpoint();
    _write_thrd_running = true;
    
    // initialise motors as not killed
    motor_killer(false);
                                                                                   
    uint64_t prev_time = get_time_usec(CLOCK_MONOTONIC);
    // Pixhawk needs to see off-board commands at minimum 2Hz, otherwise it'll go into failsafe
    while (_write_thrd_running) {
        try {
            uint64_t time_diff = (get_time_usec(CLOCK_MONOTONIC) - prev_time)/1000ll;
            if(time_diff < 75ll) boost::this_thread::sleep_for(boost::chrono::milliseconds(75ll - time_diff)); //Stream at 10 Hz
            prev_time = get_time_usec(CLOCK_MONOTONIC);
            
            //TODO lock_guard over scope
            shared_data_mutex->lock();
            bool do_write_estimate = _data->write_estimate;
            bool do_write_thrust_setpoint = _data->write_thrust_setpoint; 
            bool do_end_thrust_setpoint = _data->end_thrust_setpoint;
            bool kill_motors = _data->kill_motors;
            bool force_failsafe = _data->force_failsafe;
            bool do_reboot = _data->do_reboot;
            bool do_write_param = _data->write_param;
            bool do_read_param = _data->read_param;
            bool do_write_arm = (bool)_data->arming_flag;
            bool arming_state = do_write_arm ? (bool)(_data->arming_flag - 1) : false;
            shared_data_mutex->unlock();
            
            //check state
            if (do_reboot){
                // send reboot command
                execute_reboot();
                
                // set reboot complete
                shared_data_mutex->lock();
                _data->do_reboot = false;
                shared_data_mutex->unlock();
            }
            else if (kill_motors) {
                //try to kill motors when requested
                force_failsafe = true; //this implies forcing failsafe but then request higher emergency state!
                
                if (motor_killer(true)) {
                    //If we succesfully kill the motors of the drone, try to shut down BJOS
                    bjos::BJOS::getOS()->shutdown();
                }
                //wait some time to try again
                usleep(100000);
            }
            else if (force_failsafe) {
                //If we succesfully kill the motors of the drone, try to shut down BJOS
                bjos::BJOS::getOS()->shutdown();
                //wait some time to try again
                usleep(100000);
            }
            else if (do_write_thrust_setpoint) {
                //enable writing thrust setpoint
                write_thrust_setpoint(false);
            }
            else if (do_write_arm) {
                //arm or disarm
                write_arm(arming_state);
            }else{
                if (do_end_thrust_setpoint) write_thrust_setpoint(true);
                
                // write setpoint
                write_setpoint();
            
                //write setpoint and estimate
                if(do_write_estimate) write_estimate();
                
                //read param if necessary
                if(do_read_param) read_param();
                
                //write param if necessary
                if(do_write_param) write_param();
            }
        }
        catch (boost::thread_interrupted) {
            //if interrupt, stop and let the controller finish resources
            return;
        }
    }
}

void FlightController::write_setpoint() {
    //pull from current setpoint
    shared_data_mutex->lock();
    mavlink_set_position_target_local_ned_t sp = _data->current_setpoint;
    shared_data_mutex->unlock();
    
    //Log::info("FlightController::write_setpoint","current_setpoint: %.4f %.4f %.4f", sp.vx, sp.vy, sp.vz);
    
    //double check some system parameters
    if (not sp.time_boot_ms)
        sp.time_boot_ms = (uint32_t)(get_time_usec(CLOCK_MONOTONIC) / 1000);
    sp.target_system = system_id;
    sp.target_component = autopilot_id;
    
    //encode
    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(SYS_ID, COMP_ID, &message, &sp);
    
    //do the write
    serial_port->write_message(message);
    
    //TODO: check if write is succesfull
}

void FlightController::write_estimate() {
    //pull from current estimate
    shared_data_mutex->lock();
    
    mavlink_vision_position_estimate_t est = _data->vision_position_estimate;
    //reset the current estimate to ignore values
    _data->vision_position_estimate.usec = 0;
    _data->vision_position_estimate.x = NAN;
    _data->vision_position_estimate.y = NAN;
    _data->vision_position_estimate.z = NAN;
    _data->vision_position_estimate.roll = NAN;
    _data->vision_position_estimate.pitch = NAN;
    _data->vision_position_estimate.yaw = NAN;
    
    //bool do_write_estimate = _data->write_estimate;
    shared_data_mutex->unlock();

    //set estimate time if not yet set
    if (not est.usec)
        est.usec = get_time_usec(CLOCK_MONOTONIC);

    //Log::info("FlightController::write_estimate", "estimate %f %f %f %f %f %f %f %f", est.x, est.y, est.z, est.yaw, cur.x(), cur.y(), cur.z(), yaw);
    
    //encode 
    mavlink_message_t message;
    mavlink_msg_vision_position_estimate_encode(SYS_ID, COMP_ID, &message, &est);

    //do the write
    serial_port->write_message(message);
    
    //TODO: check if write is succesfull
}

void FlightController::write_thrust_setpoint(bool end) {
    //pull from current setpoint
    shared_data_mutex->lock();
    mavlink_set_attitude_target_t sp = _data->thrust_setpoint;
    shared_data_mutex->unlock();

    //Log::info("FlightController::write_thrust_setpoint","current_setpoint: %.4f", sp.thrust);

    //double check some system parameters
    if (not sp.time_boot_ms)
        sp.time_boot_ms = (uint32_t)(get_time_usec(CLOCK_MONOTONIC) / 1000);
    sp.target_system = system_id;
    sp.target_component = autopilot_id;

    //if we need to end sending thrust setpoints, set all flags on false and do one last write
    if (end) sp.type_mask = END_THRUST_SETPOINT;

    //encode
    mavlink_message_t message;
    mavlink_msg_set_attitude_target_encode(SYS_ID, COMP_ID, &message, &sp);

    //do the write
    serial_port->write_message(message);

    //TODO: check if write is succesfull
}

bool FlightController::read_param() {
    //get param to send
    shared_data_mutex->lock();
    mavlink_message_t message = _data->param_request;
    _data->read_param = false;
    shared_data_mutex->unlock();
    
    //do the write
    int success = serial_port->write_message(message);
    
    //error check
    if (success) {
        Log::info("FlightController::read_param", "Parameter read request is succesfull");
        return true;
    }
    else {
        return false;
    }
}

bool FlightController::write_param() {
    //get param to send
    shared_data_mutex->lock();
    mavlink_message_t message = _data->param_write;
    _data->write_param = false;
    shared_data_mutex->unlock();
    
    //do the write
    int success = serial_port->write_message(message);
    
    //error check
    if (success) {
        Log::info("FlightController::write_param", "Parameter is successfully written");
        return true;
    }
    else {
        return false;
    }
}

bool FlightController::write_arm(bool flag) {
    /* This function writes an arming state to the pixhawk and changes offboard mode accordingly */

    if(!flag) {
        int ret = toggle_offboard_control(flag, true);
        if (ret == -1) { // write error
            Log::warn("FlightController::write_arm", "Could not change offboard mode, serial port write error");
            return false;
        }
        else if (ret == 0) {
            Log::info("FlightController::write_arm", "Did not change offboard mode state, already %s", flag ? "in offboard" : "out of offboard");
        }
        usleep(100000); //short time for toggling offboard
    }

    //prepare command
    mavlink_command_long_t com;
    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation = true;
    com.param1 = (float)flag;

    //encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(SYS_ID, COMP_ID, &message, &com);

    //do the write
    int success = serial_port->write_message(message);

    //error check
    if (success) {
        Log::info("FlightController::write_arm", "drone is %s", flag ? "armed" : "disarmed");
        //disable future arming or disarming except for explicit call
        shared_data_mutex->lock();
        _data->arming_flag = 0;
        shared_data_mutex->unlock();

        //disable or enable offboard control accordingly
        if(flag) {
            usleep(100000); //short time for disarming
            int ret = toggle_offboard_control(flag, true);
            if (ret == -1) { // write error
                Log::warn("FlightController::write_arm", "Could not change offboard mode, serial port write error");
                return false;
            }
            else if (ret == 0) {
                Log::info("FlightController::write_arm", "Did not change offboard mode state, already %s", flag ? "in offboard" : "out of offboard");
                return false;
            }
            else
                return true;
        } else
            return true;
    }
    else {
        return false;
    }
}

bool FlightController::motor_killer(bool flag) {
    //prepare command
    mavlink_command_long_t com;
    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = 223; //MAV_CMD_DO_LOCKDOWN TODO implement into mavlink
    com.confirmation = true;
    com.param1 = (float)flag;

    //encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(SYS_ID, COMP_ID, &message, &com);

    //do the write
    int success = serial_port->write_message(message);

    //error check
    if (success) {
        if(flag) Log::info("FlightController::motor_killer", "MOTORS ARE KILLED");
        return true;
    }
    else {
        return false;
    }
}

bool FlightController::execute_reboot(){
    Log::info("FlightController::execute_reboot", "Starting reboot process");
    
    //stop read thread temporarily and wait till finished
    _read_thrd_running = false;
    _read_thrd.interrupt();
    _read_thrd.join();
    
    {
        // claim the lock for safety (WARNING: this hangs every other instance until reboot is finished!) and goto failsafe on every error
        std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
        
        //prepare command
        mavlink_command_long_t com;
        com.target_system = system_id;
        com.target_component = autopilot_id;
        com.command = 246; //MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        com.confirmation = true;
        com.param1 = 1;

        //encode
        mavlink_message_t message;
        mavlink_msg_command_long_encode(SYS_ID, COMP_ID, &message, &com);

        //do the write
        int success = serial_port->write_message(message);
        if (!success){
            _data->force_failsafe = true;
            return false;
        }

        //stop the serial and clean up the pointer
        serial_port->stop();
        delete serial_port;
        
        //give the pixhawk some time to restart
        sleep(REBOOT_SECS);
        
        //start the new serial port
        serial_port = new Serial_Port();
        serial_port->start();
        //give the serial port some time
        usleep(100000);
        //check the status
        if (serial_port->status != 1) {
            _data->force_failsafe = true;
            return false;
        }
        
        //reset all variables
        _data->kill_motors = false;
        _data->force_failsafe = false;
        _data->battery_percentage = 1; 
        _data->vision_position_estimate.usec = 0;
        _data->vision_position_estimate.x = NAN;
        _data->vision_position_estimate.y = NAN;
        _data->vision_position_estimate.z = NAN;
        _data->vision_position_estimate.roll = NAN;
        _data->vision_position_estimate.pitch = NAN;
        _data->vision_position_estimate.yaw = NAN;
        _data->visionYawOffset = 0;
        _data->visionPosOffset = {0, 0, 0};
        
        // set to no mavlink received
        _mavlink_received = false;
    }
    // restart read thread
    _read_thrd_running = true;
    _read_thrd = boost::thread(&FlightController::read_thread, this);
    
    //wait for mavlink to reinitalize
    unsigned int n = 0;
    unsigned int timeout = 20; //deciseconds
    while (_mavlink_received == false && n < timeout) {        
		n++;
        usleep(100000); //10 Hz
    }
    //mavlink is dead
	if (n == timeout) {        
        _data->force_failsafe = true;
        return false;
    }
    
    int result = synchronize_time();
    //mavlink is dead
    if(result == -1) {        
        _data->force_failsafe = true;
        return false;
    }
    
     //enable offboard in posix mode
#ifndef __arm__
    result = toggle_offboard_control(true, true);
    if (result == -1)
        throw ControllerInitializationError(this,
                "Could not set offboard mode: unable to write message on serial port");
    else if (result == 0)
        Log::warn("FlightController::init", "double (de-)activation of offboard mode [ignored]");
#endif
    
    Log::info("FlightController::execute_reboot", "Reboot completed!");
    return true;
}

//TODO: check mode in order to handle outside mode switching (by a telemetry command for instance)
int FlightController::toggle_offboard_control(bool flag, bool force) {
    //keep track of which mode we're in
    static bool offboard = false;

    Log::info("FlightController::toggle_offboard_control", "entered toggle_offboard_control %i", flag);

    //check for double (de-)activation
    if (force == false && offboard == flag) return 0;
    else {
        //prepare command for off-board mode
        mavlink_command_long_t com;
        com.target_system		= system_id;
        com.target_component	= autopilot_id;
        com.command				= MAV_CMD_NAV_GUIDED_ENABLE;
        com.confirmation		= true;
        com.param1				= (float)flag; // flag >0.5 => start, <0.5 => stop
        
        //encode
        mavlink_message_t message;
        mavlink_msg_command_long_encode(SYS_ID, COMP_ID, &message, &com);
        
        //do the write
        int success = serial_port->write_message(message);

        //error check
        if (success) {
            Log::info("FlightController::toggle_offboard_control", "successful write on port");
            offboard = flag;
            return 1;
        }
        else {
            return -1;
        }
    }
}

bool FlightController::synchronize_time() {
    Log::info("FlightController::synchronize_time", "entered synchronize_time");
    
    //prepare system_time message for sync
    mavlink_system_time_t sys_time;
    uint64_t cur_time;
    cur_time = sys_time.time_unix_usec = get_time_usec(CLOCK_REALTIME);
    sys_time.time_boot_ms = 0; //this is ignored

    //encode
    mavlink_message_t message;
    mavlink_msg_system_time_encode(SYS_ID, COMP_ID, &message, &sys_time);
    
    //do the write
    int success = serial_port->write_message(message);
    uint64_t boot_time = get_time_usec(CLOCK_MONOTONIC)/1000ULL;

    //error check
    if (success) {
        Log::info("FlightController::synchronize_time", "successful write on port");
    } else {
        Log::info("FlightController::synchronize_time", "failed to write on port");
        return false;
    }
    
    //reset the synchronized time to wait for new set
    shared_data_mutex->lock();
    _data->sys_time.time_unix_usec = 0;
    _data->sys_time.time_unix_usec = 0;
    sys_time = _data->sys_time;
    shared_data_mutex->unlock();
    
    //wait until we receive the system time from the Pixhawk
    int cnt = 0;
    while (sys_time.time_unix_usec < cur_time && cnt < 50){
        shared_data_mutex->lock();
        sys_time = _data->sys_time;
        shared_data_mutex->unlock();
        
        //wait some time to give the read thread time
        usleep(100000); //10 Hz
        ++cnt;
    }

    //timeout check    
    if(sys_time.time_unix_usec <= cur_time){
        Log::info("FlightController::synchronize_time", "failed to received system time within 5 seconds");
        return false;
    }
    
    //set the synchronized time
    shared_data_mutex->lock();
    _data->syncUnixTime = _data->sys_time.time_unix_usec/1000;
    _data->syncBootTime = _data->sys_time.time_boot_ms;
    _data->ourBootTime = boot_time;
    shared_data_mutex->unlock();
    
    Log::info("FlightController::synchronize_time", "successfully synchronized, Unix time is %" PRIu64 " and boot time is %" PRIu64, _data->syncUnixTime, _data->syncBootTime);
    return true;
}

Pose FlightController::getPoseWF(){
    Pose pose;
    pose.position = FlightController::getPositionWF();
    pose.orientation = FlightController::getOrientationWF();
    return pose;
}

Eigen::Vector3d FlightController::getPositionNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->positionNED;
}

Eigen::Vector3d FlightController::getPositionWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return positionNEDtoWF(_data->positionNED, _data->visionPosOffset, _data->visionYawOffset);
}

Eigen::Vector3d FlightController::getOrientationNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->orientationNED;
}

Eigen::Vector3d FlightController::getOrientationWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return orientationNEDtoWF(_data->orientationNED, _data->visionYawOffset);
}
/*
Eigen::Vector3d FlightController::getOrientationCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoCF(_data->orientationNED);
}*/

Eigen::Vector3d FlightController::getVelocityNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->velocityNED;
}
/*
Eigen::Vector3d FlightController::getVelocityWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoWF(_data->velocityNED);
}*/
/*
Eigen::Vector3d FlightController::getVelocityCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoCF(_data->velocityNED);
}*/    


Eigen::Vector3d FlightController::getAngularVelocityNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->angularVelocityNED;
}
/*
Eigen::Vector3d FlightController::getAngularVelocityWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoWF(_data->angularVelocityNED);
}*/
/*
Eigen::Vector3d FlightController::getAngularVelocityCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoCF(_data->angularVelocityNED);
}*/


Eigen::Vector3d FlightController::getTargetOrientationCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return BodyNEDtoCF(Eigen::Vector3d(
            _data->current_setpoint.x, _data->current_setpoint.y, _data->current_setpoint.z));
}

Eigen::Vector3d FlightController::getTargetVelocityCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return BodyNEDtoCF(Eigen::Vector3d(
        _data->current_setpoint.vx, _data->current_setpoint.vy, _data->current_setpoint.vz));
}

std::tuple<Eigen::Vector3d, double, Eigen::Vector3d, double> FlightController::getCurrentSetpointNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return std::make_tuple(Eigen::Vector3d(_data->current_setpoint.x, _data->current_setpoint.y, _data->current_setpoint.z),
                           _data->current_setpoint.yaw,
                           Eigen::Vector3d(_data->current_setpoint.vx, _data->current_setpoint.vy, _data->current_setpoint.vz),
                           _data->current_setpoint.yaw_rate);
}

/*std::tuple<Eigen::Vector3d, double, Eigen::Vector3d, double> FlightController::getCurrentSetpointWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return std::make_tuple(positionWFtoNED(Eigen::Vector3d(_data->current_setpoint.x, _data->current_setpoint.y, _data->current_setpoint.z)),
                           ýawWFtoNED(_data->current_setpoint.yaw, _data->visionYawOffset),
                           velocityWFtoNED(Eigen::Vector3d(_data->current_setpoint.vx, _data->current_setpoint.vy, _data->current_setpoint.vz)),
                           yawWFtoNED(_data->current_setpoint.yaw_rate));
}*/

void FlightController::toggleWriteEstimate(bool write_estimate){
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    _data->write_estimate = write_estimate;
}
bool FlightController::writeEstimateEnabled(){
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->write_estimate;
}

void FlightController::toggleWriteThrustSetpoint(bool write_thrust_setpoint) {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    _data->write_thrust_setpoint = write_thrust_setpoint;
    if (!write_thrust_setpoint) _data->end_thrust_setpoint = true;
}
bool FlightController::writeThrustSetpointEnabled() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->write_thrust_setpoint;
}

void FlightController::syncVision(Eigen::Vector3d visionPosEstimate, double visionYawOffset) {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    Log::info("FlightController::syncVision", "Sync at %f %f %f %f!", visionPosEstimate.x(), visionPosEstimate.y(), visionPosEstimate.z(), visionYawOffset);

    Eigen::AngleAxisd Rz(-visionYawOffset, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Rx(M_PI, Eigen::Vector3d::UnitX());

    Eigen::Vector3d visionPosOffset = Rx*_data->positionNED - Rz*visionPosEstimate;

    _data->visionPosOffset = visionPosOffset;
    _data->visionYawOffset = visionYawOffset;
    _data->vision_sync = true;
}

bool FlightController::isWFDefined(){
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->vision_sync;
}

//NOTE: this function is a rework of setTargetCF
void FlightController::setTargetCF(uint16_t type_mask, Eigen::Vector3d velocity, double yaw, double yaw_speed) {
    setTargetCF(type_mask, Eigen::Vector3d(), Eigen::Vector3d(0, 0, yaw), velocity, Eigen::Vector3d(0, 0, yaw_speed));
}

//WARNING: can NOT be used to set position, roll, pitch, rollspeed or pitchspeed
void FlightController::setTargetCF(uint16_t type_mask, Eigen::Vector3d,
        Eigen::Vector3d orientation, Eigen::Vector3d velocity, Eigen::Vector3d angularVelocity) {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    
    /* Tranform CF to NED Body frame */
    Eigen::Vector3d orientationNED = orientationWFtoNED(orientation, _data->visionYawOffset);
    Eigen::Vector3d velocityNED = CFtoBodyNED(velocity);
    
    mavlink_set_position_target_local_ned_t sp;
    
    sp.type_mask = type_mask;
    
    sp.x = 0; // positionNED.x();
    sp.y = 0; // positionNED.y();
    sp.z = 0; // positionNED.z();
    
    sp.vx = velocityNED.x();
    sp.vy = velocityNED.y();
    sp.vz = velocityNED.z();
    
    sp.yaw = orientationNED.z();
    sp.yaw_rate = -angularVelocity.z();
    
    sp.coordinate_frame = MAV_FRAME_BODY_NED;
    
    //Log::info("FlightController::setTargetCF", "%f %f %f - %f", sp.vx, sp.vy, sp.vz, sp.yaw_rate); 

    _data->current_setpoint = sp;
}

//WARNING: cannot be used to set velocity
void FlightController::setTargetWF(uint16_t type_mask, Eigen::Vector3d position, double yaw, double yaw_speed) {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    
    /* Tranform WF to NED frame */
    double yawNED = -yaw + _data->visionYawOffset;
    if(yaw < M_PI) yaw += 2*M_PI;
    if(yaw > M_PI) yaw -= 2*M_PI;
    Eigen::Vector3d positionNED = positionWFtoNED(position,  _data->visionPosOffset, _data->visionYawOffset);
    
    mavlink_set_position_target_local_ned_t sp;
    
    sp.type_mask = type_mask;
    
    sp.x = positionNED.x();
    sp.y = positionNED.y();
    sp.z = positionNED.z();

    sp.vx = 0;
    sp.vy = 0;
    sp.vz = 0;
    
    sp.yaw = yawNED;
    sp.yaw_rate = -yaw_speed;
    
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    
    //Log::info("FlightController::setTargetWF", "%f %f %f - %f", sp.x, sp.y, sp.z, sp.yaw_rate); 

    _data->current_setpoint = sp;
}

void FlightController::setPositionEstimateWF(Eigen::Vector3d posEst) {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    Eigen::Vector3d posEstNED = positionWFtoNED(posEst, _data->visionPosOffset, _data->visionYawOffset);
    
    //Log::info("FlightController::setPositionEstimateWF", "own: (%f, %f, %f), estimate: (%f, %f, %f)", own_estimate.x(), own_estimate.y(), own_estimate.z(), posEst.x(), posEst.y(), posEst.z());
    
    _data->vision_position_estimate.x = posEstNED[0];
    _data->vision_position_estimate.y = posEstNED[1];
    _data->vision_position_estimate.z = posEstNED[2];
}

Eigen::Vector3d FlightController::getPositionEstimateWF(){
    Eigen::Vector3d posEstNED;
    posEstNED[0] = _data->vision_position_estimate.x;
    posEstNED[1] = _data->vision_position_estimate.y;
    posEstNED[2] = _data->vision_position_estimate.z;
    
    return positionNEDtoWF(posEstNED, _data->visionPosOffset, _data->visionYawOffset);
}

void FlightController::setYawEstimateWF(double yawEst) {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    Eigen::Vector3d attEstNED = orientationWFtoNED(Eigen::Vector3d(_data->orientationNED[0], -_data->orientationNED[1], yawEst), _data->visionYawOffset);

    _data->vision_position_estimate.roll = attEstNED[0];
    _data->vision_position_estimate.pitch = attEstNED[1];
    _data->vision_position_estimate.yaw = attEstNED[2];
}

double FlightController::getYawEstimateWF(){
    Eigen::Vector3d attEstNED;
    attEstNED[0] = _data->vision_position_estimate.roll;
    attEstNED[1] = _data->vision_position_estimate.pitch;
    attEstNED[2] = _data->vision_position_estimate.yaw;
    
    return orientationNEDtoWF(attEstNED, _data->visionYawOffset)[2];
}

void FlightController::setThrustSetpoint(float thrust) {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);

    mavlink_set_attitude_target_t sp;

    sp.type_mask = SET_THRUST_SETPOINT;
    
    sp.thrust = thrust;
    
    //Log::info("FlightController::setThrustSetpoint", "%f", sp.thrust); 

    _data->thrust_setpoint = sp;
}

//TODO: fix yaw rotation on Raspberry Pi side in order to send position setpoints
/*Eigen::Vector3d FlightController::positionCFtoBodyNED(Eigen::Vector3d positionCF) {
    Eigen::Affine3d t;
    t = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    return t * positionCF + _data->positionNED;  //_data->positionNED is not in the same frame as t * positionCF an thus can't be used
}*/

Eigen::Vector3d FlightController::CFtoBodyNED(Eigen::Vector3d vectorCF) {
    Eigen::Affine3d t;
    t = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    return t * vectorCF;
}

Eigen::Vector3d FlightController::positionNEDtoCF(Eigen::Vector3d positionNED, double yawNED) {
    Eigen::Vector3d res = positionNED - _data->positionNED;
    Eigen::Affine3d t;
    t = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    t *= Eigen::AngleAxisd(yawNED, Eigen::Vector3d::UnitZ());
    return t * res;
}

Eigen::Vector3d FlightController::NEDtoCF(Eigen::Vector3d vectorNED, double yawNED) {
    Eigen::Affine3d t;
    t = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    t *= Eigen::AngleAxisd(yawNED, Eigen::Vector3d::UnitZ());
    return t * vectorNED;
}

Eigen::Vector3d FlightController::BodyNEDtoCF(Eigen::Vector3d vectorNED) {
    Eigen::Affine3d t;
    t = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());
    return t * vectorNED;
}

//FIXME
/*Point FlightController::NEDtoWF(Eigen::Vector3d vectorNED, Eigen::Vector3d visionPosOffset) {
    Eigen::Affine3d t;
	RotationMatrix R(-M_PI, 'x');
	return R.rotatePoint(pointNED) - visionPosOffset;
} */

Eigen::Vector3d FlightController::positionWFtoNED(Eigen::Vector3d positionWF, Eigen::Vector3d visionPosOffset, double visionYawOffset) {
    Eigen::Affine3d t = Eigen::AngleAxisd(visionYawOffset, Eigen::Vector3d::UnitZ())*Eigen::Translation3d(-visionPosOffset)*Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    return t.inverse()*positionWF;
}

Eigen::Vector3d FlightController::orientationWFtoNED(Eigen::Vector3d orientationWF, double visionYawOffset) {
    Eigen::Vector3d out;
    out[0] = orientationWF[0];
    out[1] = -orientationWF[1];
    out[2] = -orientationWF[2] + visionYawOffset;
    if(out[2] < M_PI) out[2] += 2*M_PI;
    if(out[2] > M_PI) out[2] -= 2*M_PI;
    return out;
}

//NOTE: don't touch this function or change this algorithm in any way before contacting @author!
Eigen::Vector3d FlightController::positionNEDtoWF(Eigen::Vector3d positionNED, Eigen::Vector3d visionPosOffset, double visionYawOffset) {
    Eigen::Affine3d t = Eigen::AngleAxisd(visionYawOffset, Eigen::Vector3d::UnitZ())*Eigen::Translation3d(-visionPosOffset)*Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    return t*positionNED;
}

Eigen::Vector3d FlightController::orientationNEDtoWF(Eigen::Vector3d orientationNED, double visionYawOffset) {
    Eigen::Vector3d out;
    out[0] = orientationNED[0];
    out[1] = -orientationNED[1];
    out[2] = -orientationNED[2] + visionYawOffset;
    if(out[2] < M_PI) out[2] += 2*M_PI;
    if(out[2] > M_PI) out[2] -= 2*M_PI;
    return out;
}

std::pair<float, uint8_t> FlightController::readParameter(const char* id){
    if(id[0] == 0) return std::make_pair(NAN, UINT8_MAX);
    
    shared_data_mutex->lock();
    bool read_param = _data->read_param;
    int32_t system_id = _data->system_id;
    int32_t autopilot_id = _data->autopilot_id;
    
    //set sys and comp_id
    shared_data_mutex->unlock();
    
    // FIXME: check if read is fully finished
    if(read_param) return std::make_pair(NAN, UINT8_MAX);
        
    //build mavlink message
    mavlink_message_t msg;
    mavlink_param_request_read_t param_request;
    
    param_request.param_index = -1;
    param_request.target_system = system_id;
    param_request.target_component = autopilot_id;
    strncpy(param_request.param_id, id, MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN);
    mavlink_msg_param_request_read_encode(SYS_ID, COMP_ID, &msg, &param_request);
    
    //set param request
    shared_data_mutex->lock();  
    bool read_param_response = _data->read_param_response = false;
    _data->param_request = msg;
    _data->read_param = true;
    shared_data_mutex->unlock();    
    
    int cnt = 0;
    int timeout = 20;
    while(!read_param_response && cnt < timeout){
        //wait for read parameter to complete
        //FIXME: do something better than semi-busy waiting...
        shared_data_mutex->lock();    
        read_param_response = _data->read_param_response;
        shared_data_mutex->unlock();    
        
        ++cnt;
        usleep(100000); //10 Hz
    }
    
    shared_data_mutex->lock();    
    _data->read_param_response = false;
    msg = _data->param_read;
    shared_data_mutex->unlock();
    
    if(!read_param_response) return std::make_pair(NAN, UINT8_MAX);    
    
    return std::make_pair(mavlink_msg_param_value_get_param_value(&msg), mavlink_msg_param_value_get_param_type(&msg));;
}

bool FlightController::writeParameter(const char* id, float value, uint8_t type) {
    /* Writes parameter number with value
    * WARN: THIS FUNCTION REQUIRES KNOWLEDGE OF THE PIXHAWK PARAMETER TYPES, DO NOT CALL IF UNSURE
    *
    * TODO: Check if MAVLink returns a PARAM_VALUE message, this is an acknowledgement of this message
    **/
    shared_data_mutex->lock();
    bool write_param = _data->write_param;
    
    int32_t system_id = _data->system_id;
    int32_t autopilot_id = _data->autopilot_id;
    if(!write_param) _data->write_param = true;

    //set sys and comp_id
    shared_data_mutex->unlock();
    if(write_param) return false;
    
    // Param may not be null terminated if exactly fits
    char paramId[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN + 1];
    paramId[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN] = 0;
    strncpy(paramId, id, MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN);

    mavlink_message_t msg;
    mavlink_msg_param_set_pack(SYS_ID,
                               COMP_ID,
                               &msg,
                               system_id,
                               autopilot_id,
                               paramId,
                               value,
                               type
                              );

    shared_data_mutex->lock();
    _data->param_write = msg;
    shared_data_mutex->unlock();

    //FIXME: wait till command is executed and check response
    return true;
}

void FlightController::armDisarm(bool flag) {
    uint32_t arming_flag = flag ? 2 : 1;

    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    _data->arming_flag = arming_flag;
}

void FlightController::killMotors() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    _data->kill_motors = true;
}

//WARNING: only do this in landed state else bad things can happen
void FlightController::reboot() {
    shared_data_mutex->lock();
    //unsync the world frame (before the actual reboot)
    _data->vision_sync = false;
    
    //ask for a reboot
    _data->do_reboot = true;
    shared_data_mutex->unlock();
    
    //wait until reboot is done
    bool reboot = true;
    while(reboot){
        usleep(100000); //10 Hz
        shared_data_mutex->lock();
        reboot = _data->do_reboot;
        shared_data_mutex->unlock();
    }
    return;
}

void FlightController::forceFailsafe(){
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    _data->force_failsafe = true;
}

bool FlightController::inFailsafe(){
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->force_failsafe;
}

bool FlightController::isLanded() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->landed;
}
double FlightController::getBatteryPercentage(){
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->battery_percentage;
}

//get raw imu data
/*IMUSensorData FlightController::getIMUDataCF(){
    //get data (FIXME: ensure proper frame)
    shared_data_mutex->lock();
    IMUSensorData imuCF = _data->imuNED;

    uint64_t unixTime = _data->syncUnixTime;
    uint64_t bootTime = _data->syncBootTime;
    shared_data_mutex->unlock();
    
    //scale the time to microseconds in unix timestamp
    imuCF.time = imuCF.time - bootTime + unixTime;
    
    return imuCF;
}*/
