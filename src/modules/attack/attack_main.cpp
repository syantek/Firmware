/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file attack_main.cpp
 * Implementation of various false data injection attack for cyber-physical
 * system research work.
 *
 * @author Scott Yantek
 */

#include <math.h>
#include <px4_posix.h>
//#include <px4_time.h>
#include <poll.h>
#include <px4_log.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <drivers/drv_hrt.h>

extern "C" __EXPORT int attack_main(int argc, char *argv[]);

//TODO implement the attack class
class Attack;

namespace attack
{
Attack *instance = nullptr;
}

class Attack
{
public:
	/**
	 * Constructor
	 */
	Attack();

	/**
	 * Destructor, also kills task.
	 */
	~Attack();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();
	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	 
	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

	void print_status();

	int task_main(int argc, char *argv[]);
	
	void exit() { _task_should_exit = true; }
	
private:
    int	_control_task = -1;		// task handle for task
    bool _task_should_exit = false;
    bool _attack_active = false;
    hrt_abstime _start;
    
    orb_advert_t _sensors_fdi_pub;
    orb_advert_t _vehicle_attitude_setpoint_fdi_pub;
    orb_advert_t _vehicle_gps_position_fdi_pub;
    
    // parameters
    struct {
        int attack_switch;
        int attack_type;
        float lat_ramp;
        float lon_ramp;
    } _parameters;
    
    // parameter handles
    struct {
        param_t attack_switch_h;
        param_t attack_type_h;
        param_t lat_ramp_h;
        param_t lon_ramp_h;
    } _parameter_handles;
};

Attack::Attack()
{

}

Attack::~Attack()
{

}

void Attack::print_status()
{
    if (_attack_active) {
        PX4_INFO("Attack active");
    } else {
        PX4_INFO("Attack NOT active");
    }
    return;
}

void Attack::task_main()
{
    //subscribe to relevant uORB topics (sensor, actuator)
    int sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
    int params_sub = orb_subscribe(ORB_ID(parameter_update));
    
    //set up publishers for relevant topics (all the attack stuff)
    /*struct sensor_combined_s sensors_fdi;
    memset(&sensors_fdi, 0, sizeof(sensors_fdi));
    _sensors_fdi_pub = orb_advertise(ORB_ID(sensor_combined_fdi),
            &sensors_fdi);*/
    struct vehicle_attitude_setpoint_s vehicle_attitude_setpoint_fdi_fd;
    memset(&vehicle_attitude_setpoint_fdi_fd, 0, sizeof(vehicle_attitude_setpoint_fdi_fd));
    _vehicle_attitude_setpoint_fdi_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint_fdi),
            &vehicle_attitude_setpoint_fdi_fd);
    
    struct vehicle_gps_position_s vehicle_gps_position_fdi_fd;
    memset(&vehicle_gps_position_fdi_fd, 0, sizeof(vehicle_gps_position_fdi_fd));
    _vehicle_gps_position_fdi_pub = orb_advertise(ORB_ID(vehicle_gps_position_fdi), &vehicle_gps_position_fdi_fd);
    
    // initialize current time
    hrt_abstime now = 0;
    
    /*px4_pollfd_struct_t fds[2] = {};
    fds[0].fd = sensors_sub;
    fds[0].events = POLLIN; // POLLIN is #defined as (0x01) in poll.h
    fds[1].fd = params_sub;
    fds[1].events = POLLIN;TODO same as the part that uses this in the loop*/
    
    // initialize parameter cache
    //updateParams(); //TODO part of block in controllib, not sure what to do, probably don't need to do anything
    
    //initialize data structures for subscribed uORB topics
    //sensor_combined_s sensors = {};
    
    //get the parameter handles that matter
    _parameter_handles.attack_switch_h = param_find("ATTACK_MSTR_SW");
    _parameter_handles.attack_type_h = param_find("ATTACK_TYPE");
    _parameter_handles.lat_ramp_h = param_find("LAT_RAMP");
    _parameter_handles.lon_ramp_h = param_find("LON_RAMP");
    
    while (!_task_should_exit) {// main loop
        //TODO wait a bit so that this hopefully doesn't crash stuff - check other things to see how they do it.
        usleep(10000);
        // check master attack switch (if it's off, don't do anything)
        param_get(_parameter_handles.attack_switch_h, &(_parameters.attack_switch));
        if ((_parameters.attack_switch == 1478) & (!_attack_active)) {
            _start = hrt_absolute_time();
        }
        
        _attack_active = (_parameters.attack_switch == 1478);
        
        if (_attack_active) {
            now = hrt_absolute_time();
            // check which attack types are active
            param_get(_parameter_handles.attack_type_h, &(_parameters.attack_type));
            
            // pitch oscillation attack
            if (_parameters.attack_type == 1) {
                //calculate what to inject
                vehicle_attitude_setpoint_fdi_fd.pitch_body = 
                        0.7f*float(cos((hrt_absolute_time()-_start)*5e-6));
                //publish the attack
                orb_publish(ORB_ID(vehicle_attitude_setpoint_fdi), _vehicle_attitude_setpoint_fdi_pub, &vehicle_attitude_setpoint_fdi_fd);
                //TODO make fw_pos_control subscribe to the attack and add it to its publication
            }
            if (_parameters.attack_type == 2) {//GPS ramp attack
                //update ramp amounts
                param_get(_parameter_handles.lat_ramp_h, &(_parameters.lat_ramp));
                param_get(_parameter_handles.lon_ramp_h, &(_parameters.lon_ramp));
                
                // calculate what to inject
                vehicle_gps_position_fdi_fd.lat = (now - _start) * _parameters.lat_ramp;
                vehicle_gps_position_fdi_fd.lon = (now - _start) * _parameters.lon_ramp;
                
                // publish to GPS_fdi
                orb_publish(ORB_ID(vehicle_gps_position_fdi), _vehicle_gps_position_fdi_pub, &vehicle_gps_position_fdi_fd);
                
                //TODO have all GPS subscribers listen to GPS_fdi topic and add that
            }
        }
        
        
        
        // update current time
        //now = hrt_absolute_time();
        
        //orb_copy(ORB_ID(sensor_combined), sensors_sub, &sensors);
        /*PX4_INFO("Attack state:\t%i\t%i",
         *           (int)now,
         *           (int)_parameters.attack_switch);
         */
        //update attacks
        //TODO
        /*sensors_fdi.gyro_rad[0] = ;
        sensors_fdi.gyro_rad[1] = ;
        sensors_fdi.gyro_rad[3] = ;
        sensors_fdi.gyro_integral_dt
        sensors_fdi.accelerometer_timestamp_relative
        sensors_fdi.accelerometer_m_s2//TODO att 2 more
        sensors_fdi.accelerometer_integral_dt
        sensors_fdi.magnetometer_timestamp_relative
        sensors_fdi.magnetometer_ga//TODO add 2 more
        sensors_fdi.baro_timestamp_relative
        sensors_fdi.baro_alt_meter
        sensors_fdi.baro_temp_celcius
        */
        // publish attacks
        //orb_publish(ORB_ID(sensor_combined_fdi), _sensors_fdi_pub, &sensors);//TODO
    }
    
    //unsubscribe from orb topics
    orb_unsubscribe(sensors_sub);
    orb_unsubscribe(params_sub);
    
    //unadvertise published orb topics
    //orb_unadvertise(_sensors_fdi_pub);
    orb_unadvertise(_vehicle_attitude_setpoint_fdi_pub);
    orb_unadvertise(_vehicle_gps_position_fdi_pub);

	//clean up the attack instance and reset
	delete attack::instance;
	attack::instance = nullptr;

    return;
}











void Attack::task_main_trampoline(int argc, char *argv[])
{
	attack::instance->task_main();
}

int Attack::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("attack",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&Attack::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

int attack_main(int argc, char *argv[])
{
    if (argc < 2) {
		PX4_WARN("usage: attack {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (attack::instance != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		attack::instance = new Attack();

		if (attack::instance == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}


		if (OK != attack::instance->start()) {
			delete attack::instance;
			attack::instance = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (attack::instance == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		attack::instance->exit();

		// wait for the destruction of the instance
		while (attack::instance != nullptr) {
			usleep(50000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "print")) {
		if (attack::instance != nullptr) {

			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		if (attack::instance) {
			PX4_WARN("running");
			attack::instance->print_status();
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}
