/**
 * @file flightController.cpp
 *
 * @brief Drone interface functions
 *
 * Functions for sending and receiving commands to a drone via MAVLink
 *
 * @author Joep Linssen,	<joep.linssen@bluejayeindhoven.nl>
 * @author Wouter van der Stoel <wouter@bluejayeindhoven.nl>
 */

#include "controllers/FlightController.h"

#include <chrono>

using namespace bjos;

uint64_t get_time_usec(clockid_t clk_id)
{
    struct timespec _time_stamp;
    clock_gettime(clk_id, &_time_stamp);
    return _time_stamp.tv_sec * 1000000ULL + _time_stamp.tv_nsec / 1000ULL;
}

//WARNING: blocks while no initial messages are received from the drone
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
    if (serial_port->status != 1) //SERIAL_PORT_OPEN
        throw ControllerInitializationError(this, "Serial port not open");
    
    //start read thread
    _read_thrd_running = true;
    _read_thrd = boost::thread(&FlightController::read_thread, this);
    
    // read_thread initialises 'initial_position' and signals this by setting _init_set
    std::cout << "Receiving initial position ...";
	unsigned int n = 0;
    unsigned int timeout = 20; //deciseconds
    while (_init_set == false && n < timeout) {
		n++;
        usleep(100000); //10 Hz
    }
	if (n == timeout)
		throw ControllerInitializationError(this, "Did not receive any MAVLink messages");

    std::cout << " Received!" << std::endl;
    Log::info("FlightController::init",
            "Initial position: xyz=[%.4f %.4f %.4f] vxvyvz=[%.4f %.4f %.4f]", initial_position.x,
            initial_position.y, initial_position.z, initial_position.vx, initial_position.y,
            initial_position.z);
    
    // write_thread initialises 'current_setpoint': all velocities 0
    _write_thrd = boost::thread(&FlightController::write_thread, this);
    
    //wait until write_thread starts
    while (_write_thrd_running == false)
        usleep(100000); //10 Hz
        
    //synchronize the time with pixhawk
    int result = synchronize_time();
    if(result == -1)
        throw ControllerInitializationError(this,
                "Could not synchronize time with the Pixhawk: error with connection");
        
    //in order for the drone to react to the streamed setpoints, offboard mode has to be enabled
    result = toggle_offboard_control(true);
    if (result == -1)
        throw ControllerInitializationError(this,
                "Could not set offboard mode: unable to write message on serial port");
    else if (result == 0)
        Log::warn("FlightController::init", "double (de-)activation of offboard mode [ignored]");
    
    // Done!
}

void FlightController::load(bjos::BJOS *bjos) {
    Controller::load(bjos, "flight", _data);
}

void FlightController::read_thread() {
    while (_read_thrd_running)
    {
        try {
            read_messages();
            
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
                //Log::info("FlightController::read_messages", "MAVLINK_MSG_LOCAL_POSITION_NED");
                
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
                
                if (!_init_set) {
                    mavlink_msg_local_position_ned_decode(&message, &initial_position);
                    _init_set = true;
                }
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE:
            {
                //Log::info("FlightController::read_messages", "MAVLINK_MSG_ATTITUDE");
                
                //decode
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&message, &attitude);
                
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
            case MAVLINK_MSG_ID_STATUSTEXT:
            {
                mavlink_statustext_t statustext;
                mavlink_msg_statustext_decode(&message, &statustext);
                
                Log::info("FlightController::read_messages", "[status]\t%s", statustext.text);
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
                std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
                mavlink_highres_imu_t highres_imu;
                mavlink_msg_highres_imu_decode(&message, &highres_imu);
                
                _data->imuNED.time = highres_imu.time_usec/1000ULL;
                _data->imuNED.ax = highres_imu.xacc;
                _data->imuNED.ay = highres_imu.yacc;
                _data->imuNED.az = highres_imu.zacc;
                _data->imuNED.gx = highres_imu.xgyro;
                _data->imuNED.gy = highres_imu.ygyro;
                _data->imuNED.gz = highres_imu.zgyro;
                break;
            }
            default:
            {
                //Log::info("FlightController::read_messages", "Not handling this message: %" PRIu8, message.msgid);
            }
        }
    }
    else {
        //                Log::warn("",".");
    }
    
    return;
}

void FlightController::write_thread() {
    //prepare an initial setpoint: all velocities 0
    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = SET_TARGET_VELOCITY &
    SET_TARGET_YAW_RATE;
    sp.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;
    sp.vx = 0;
    sp.vy = 0;
    sp.vz = 0;
    sp.yaw_rate = 0;
    
    // set target
    _data->current_setpoint = sp;
    
    // write a message and signal writing
    write_setpoint();
    _write_thrd_running = true;
    
    // Pixhawk needs to see off-board commands at minimum 2Hz, otherwise it'll go into failsafe
    while (_write_thrd_running) {
        try {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100)); //Stream at 10 Hz
            
            write_setpoint();
        }
        catch (boost::thread_interrupted) {
            //if interrupt, stop and let the controller finish resources
            ///Q: wha??
            return;
        }
    }
    
    return;
}

void FlightController::write_setpoint() {
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    
    // pull from position target
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    mavlink_set_position_target_local_ned_t sp = _data->current_setpoint;
    //Log::info("FlightController::write_setpoint","current_setpoint: %.4f %.4f %.4f", sp.vx, sp.vy, sp.vz);
    
    // double check some system parameters
    if (not sp.time_boot_ms)
        sp.time_boot_ms = (uint32_t)(get_time_usec(CLOCK_MONOTONIC) / 1000);
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
    int len = serial_port->write_message(message);
    
    // check the write
    if (len <= 0)
        std::cout << ".";
    else {
        //TODO: log writing of setpoint per type_mask
        //Log::info("FlightController::write_setpoint", "Wrote setpoint");
    }
    
    return;
}

//TODO: check mode in order to handle outside mode switching (by a telemetry command for instance)
int FlightController::toggle_offboard_control(bool flag) {
    static bool offboard = false;
    Log::info("FlightController::toggle_offboard_control", "entered toggle_offboard_control %i", flag);
    if (offboard == flag) return 0;
    else {
        // Prepare command for off-board mode
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
        int success = serial_port->write_message(message);
        if (success) {
            Log::info("FlightController::toggle_offboard_control", "successful write on port");
            offboard = flag;
            return 1;
        }
        else {
            //write error
            return -1;
        }
    }
}

bool FlightController::synchronize_time() {
    Log::info("FlightController::synchronize_time", "entered synchronize_time");
    
    // prepare command for time synchronize
    mavlink_system_time_t sys_time;
    uint64_t cur_time;
    cur_time = sys_time.time_unix_usec = get_time_usec(CLOCK_REALTIME);
    sys_time.time_boot_ms = 0; //this is ignored

    //encode and send
    mavlink_message_t message;
    mavlink_msg_system_time_encode(system_id, autopilot_id, &message, &sys_time);
    
    int success = serial_port->write_message(message);
    if (success) {
        Log::info("FlightController::synchronize_time", "successfull write on port");
    }else{
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
    
    if(sys_time.time_unix_usec <= cur_time){
        Log::info("FlightController::synchronize_time", "failed to received system time within 5 seconds");
        return false;
    }
    
    //set the synchronized time
    shared_data_mutex->lock();
    _data->syncUnixTime = _data->sys_time.time_unix_usec/1000;
    _data->syncBootTime = _data->sys_time.time_boot_ms;
    shared_data_mutex->unlock();
    
    Log::info("FlightController::synchronize_time", "successfully synchronized, Unix time is %" PRIu64 " and boot time is %" PRIu64, _data->syncUnixTime, _data->syncBootTime);
    return true;
}

Eigen::Vector3d FlightController::getPositionNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->positionNED;
}
/*
Eigen::Vector3d FlightController::getPositionWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return positionNEDtoWF(_data->positionNED);
}
*/

Eigen::Vector3d FlightController::getOrientationNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->orientationNED;
}
/*
Eigen::Vector3d FlightController::getOrientationWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoWF(_data->orientationNED);
}*/

Eigen::Vector3d FlightController::getOrientationCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoCF(_data->orientationNED);
}


Eigen::Vector3d FlightController::getVelocityNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->velocityNED;
}
/*
Eigen::Vector3d FlightController::getVelocityWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoWF(_data->velocityNED);
}*/

Eigen::Vector3d FlightController::getVelocityCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoCF(_data->velocityNED);
}    


Eigen::Vector3d FlightController::getAngularVelocityNED() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return _data->angularVelocityNED;
}

/*
Eigen::Vector3d FlightController::getAngularVelocityWF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoWF(_data->angularVelocityNED);
}*/

Eigen::Vector3d FlightController::getAngularVelocityCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return NEDtoCF(_data->angularVelocityNED);
}


Eigen::Vector3d FlightController::getTargetOrientationCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return Eigen::Vector3d(
            _data->current_setpoint.x, _data->current_setpoint.y, _data->current_setpoint.z);
}

Eigen::Vector3d FlightController::getTargetVelocityCF() {
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    return Eigen::Vector3d(
            _data->current_setpoint.vx, _data->current_setpoint.vy, _data->current_setpoint.vz);
}

//ALERT: can NOT be used to set roll, pitch, rollspeed or pitchspeed
void FlightController::setTargetCF(uint16_t type_mask, Eigen::Vector3d position,
        Eigen::Vector3d orientation, Eigen::Vector3d velocity, Eigen::Vector3d angularVelocity) {	
    std::lock_guard<bjos::BJOS::Mutex> lock(*shared_data_mutex);
    /* Tranform given position, velocity and yaw from CF frame to NED Body frame */
    //Eigen::Vector3d positionNED = positionCFtoNED(position);
    Eigen::Vector3d orientationNED = CFtoNED(orientation);
    Eigen::Vector3d velocityNED = CFtoNED(velocity);
    
    mavlink_set_position_target_local_ned_t sp;
    
    sp.type_mask = type_mask;
    
    sp.x = 0; // positionNED.x();
    sp.y = 0; // positionNED.y();
    sp.z = 0; // positionNED.z();
    
    sp.vx = velocityNED.x();
    sp.vy = velocityNED.y();
    sp.vz = velocityNED.z();
    
    sp.yaw = orientationNED.z();
    sp.yaw_rate = angularVelocity.z(); //yaw velocity is independent of frame
    
    sp.coordinate_frame = MAV_FRAME_BODY_NED;
    
    //Mutex already locked above
    _data->current_setpoint = sp;
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

Eigen::Vector3d FlightController::positionNEDtoCF(Eigen::Vector3d positionNED) {
    Eigen::Vector3d res = positionNED - _data->positionNED;
    Eigen::Affine3d t;
    t = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    t *= Eigen::AngleAxisd(_data->orientationNED[2], Eigen::Vector3d::UnitZ());
    return t * res;
}

Eigen::Vector3d FlightController::NEDtoCF(Eigen::Vector3d vectorNED) {
    Eigen::Affine3d t;
    t = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    t *= Eigen::AngleAxisd(_data->orientationNED[2], Eigen::Vector3d::UnitZ());
    return t * vectorNED;
}

//get raw imu data
IMUSensorData FlightController::getIMUDataCF(){
    //copy the data
    shared_data_mutex->lock();
    IMUSensorData imuCF = _data->imuNED;
    double yaw = _data->orientationNED.z();
    uint64_t unixTime = _data->syncUnixTime;
    uint64_t bootTime = _data->syncBootTime;
    shared_data_mutex->unlock();

    //scale the time to microseconds in unix timestamp
    imuCF.time = imuCF.time - bootTime + unixTime;
    
    //convert the acceleration to control frame
    /*Point acc(imuCF.ax, imuCF.ay, imuCF.az);
    acc = NEDtoCF(acc, yaw, Point());
    imuCF.ax = acc.x;
    imuCF.ay = acc.y;
    imuCF.az = acc.z;*/
    
    return imuCF;
}
