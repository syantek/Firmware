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

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h> //TODO some of the below messages may not be necessary
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/tecs_status.h>


extern "C" __EXPORT int fdi_attack_main(int argc, char *argv[]);


//TODO implement the attack class
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
	bool		task_running() { return _task_running; }

private:
	orb_advert_t	_mavlink_log_pub;

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	
	// uORB subscriptions
	int		_sensor_combined_sub;
	int     _vehicle_gps_position_sub;
	int     _airspeed_sub;
	int     _optical_flow_sub;
	int     _distance_sensor_sub;
	int     _vision_position_estimate_sub;
	int     _vehicle_land_detected_sub;
	int     _vehicle_status_sub;
	int     _fw_pos_ctrl_status_sub;
	int     _vehicle_attitude_setpoint_sub;
	int     _tecs_status_sub;
	int		_params_sub;			/**< notification of parameter updates */
	
	// uORB advertisements
	orb_advert_t    _sensor_combined_fdi_pub; //attacked accel, gyro, baro, mag
	orb_advert_t    _vehicle_gps_position_fdi_pub; //attacked GPS
	orb_advert_t    _airspeed_fdi_pub; //attacked airspeed
	orb_advert_t    _vehicle_attitude_setpoint_fdi_pub; //attack attitude sp
	
	
	struct {
	    //TODO fill in
	} _parameters; //local copies of relevant parameters
	
	struct {
	    //TODO fill in
	} _parameter_handles; //handles of relevant parameters
	
	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update attack outputs
	 *
	 */
	void		attack_update();
	
	/**
	 * Check for accel updates.
	 */
	void		vehicle_sensor_combined_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();
















int fdi_attack_main(int argc, char *argv[])
{
    //subscribe to relevant uorb topics (sensor, actuator)
    
    //set up publishers for relevant topics (all the attack stuff)
    //TODO implement attack-based messages
    
    //get the parameter handles that matter
    
    
    //main loop
    while (!_task_should_exit) {
        //check for updates to attack status
        
        //check for uorb updates
        
        // compute what the injected attack should be
        
        //publish attack messages to uorb
    }


    return 0
}



}







