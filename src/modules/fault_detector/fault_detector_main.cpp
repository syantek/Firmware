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
 * @file fault_detector_main.cpp
 * Implementation of residual-based fault detection for cyber-physical
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
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/fault_detection.h>
#include <drivers/drv_hrt.h>

extern "C" __EXPORT int detector_main(int argc, char *argv[]);

class Detector;

namespace detector
{
Detector *instance = nullptr;
}

class Detector
{
public:
	/**
	 * Constructor
	 */
	Detector();

	/**
	 * Destructor, also kills task.
	 */
	~Detector();

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
	
	void print_example_detection();

	int task_main(int argc, char *argv[]);
	
	void exit() { _task_should_exit = true; }
	
private:
    int	_control_task = -1;		// task handle for task
    bool _task_should_exit = false;
    bool _detector_active = false;
    int _start;
    float residual_power;
    
    orb_advert_t _detector_pub;
    
    // parameters
    struct {
        int detect_threshold;
    } _parameters;
    
    // parameter handles
    struct {
        param_t detect_threshold_h;
    } _parameter_handles;
};

Detector::Detector()
{

}

Detector::~Detector()
{

}

void Detector::print_status()
{
    if (_detector_active) {
        PX4_INFO("Detector active");
    } else {
        PX4_INFO("Detector NOT active");
    }
    return;
}

void Detector::print_example_detection()
{
    PX4_WARN("Fault Detected. Residual power of 19.2 exceeds threshold of 15");
    return;
}

void Detector::task_main()
{
    //subscribe to relevant uORB topics (sensor, actuator)
    int innov_sub = orb_subscribe(ORB_ID(ekf2_innovations));
    int params_sub = orb_subscribe(ORB_ID(parameter_update));
    
    //set up publishers for relevant topics (all the fault detection stuff)
    struct fault_detection_s detector_fd;
    memset(&detector_fd, 0, sizeof(detector_fd));
    _detector_pub = orb_advertise(ORB_ID(fault_detection), &detector_fd);
    
    // initialize current time
    //hrt_abstime now = 0;
    
    /*px4_pollfd_struct_t fds[2] = {};
    fds[0].fd = sensors_sub;
    fds[0].events = POLLIN; // POLLIN is #defined as (0x01) in poll.h
    fds[1].fd = params_sub;
    fds[1].events = POLLIN;TODO same as the part that uses this in the loop*/
    
    // initialize parameter cache
    //updateParams(); //TODO part of block in controllib, not sure what to do, probably don't need to do anything
    
    //initialize data structures for subscribed uORB topics
    ekf2_innovations_s innov = {};
    
    //get the parameter handles that matter
    _parameter_handles.detect_threshold_h = param_find("DETECT_THRESHOLD");
    
    // set active flag true
    _detector_active = true;
    
    while (!_task_should_exit) {// main loop
        // check for change to detector threshold
        param_get(_parameter_handles.detect_threshold_h, &(_parameters.detect_threshold));
        
        // Get innovation information
        orb_copy(ORB_ID(ekf2_innovations), innov_sub, &innov);
        
        // Perform detector calculations
        
        residual_power = 0;
        //TODO change this in the future for non-diagonal innovation covariance
        // In JMAVSim, only the vel_pos and hagl terms (7 in total) have defined variances
        if (innov.vel_pos_innov_var[0] > 0) {
            residual_power += innov.vel_pos_innov[0]*innov.vel_pos_innov[0]/innov.vel_pos_innov_var[0];
            //PX4_INFO("0");
        }
        if (innov.vel_pos_innov_var[1] > 0) {
            residual_power += innov.vel_pos_innov[1]*innov.vel_pos_innov[1]/innov.vel_pos_innov_var[1];
            //PX4_INFO("1");
        }
        if (innov.vel_pos_innov_var[2] > 0) {
            residual_power += innov.vel_pos_innov[2]*innov.vel_pos_innov[2]/innov.vel_pos_innov_var[2];
            //PX4_INFO("2");
        }
        if (innov.vel_pos_innov_var[3] > 0) {
            residual_power += innov.vel_pos_innov[3]*innov.vel_pos_innov[3]/innov.vel_pos_innov_var[3];
            //PX4_INFO("3");
        }
        if (innov.vel_pos_innov_var[4] > 0) {
            residual_power += innov.vel_pos_innov[4]*innov.vel_pos_innov[4]/innov.vel_pos_innov_var[4];
            //PX4_INFO("4");
        }
        if (innov.vel_pos_innov_var[5] > 0) {
            residual_power += innov.vel_pos_innov[5]*innov.vel_pos_innov[5]/innov.vel_pos_innov_var[5];
            //PX4_INFO("5");
        }
        if (innov.mag_innov_var[0] > 0) {
            residual_power += innov.mag_innov[0]*innov.mag_innov[0]/innov.mag_innov_var[0];
            //PX4_INFO("6");
        }
        if (innov.mag_innov_var[1] > 0) {
            residual_power += innov.mag_innov[1]*innov.mag_innov[0]/innov.mag_innov_var[1];
            //PX4_INFO("7");
        }
        if (innov.mag_innov_var[2] > 0) {
            residual_power += innov.mag_innov[2]*innov.mag_innov[0]/innov.mag_innov_var[2];
            //PX4_INFO("8");
        }
        if (innov.heading_innov_var > 0) {
            residual_power += innov.heading_innov*innov.heading_innov/innov.heading_innov_var;
            //PX4_INFO("9");
        }
        if (innov.airspeed_innov_var > 0) {
            residual_power += innov.airspeed_innov*innov.airspeed_innov/innov.airspeed_innov_var;
            //PX4_INFO("10");
        }
        if (innov.beta_innov_var > 0) {
            residual_power += innov.beta_innov*innov.beta_innov/innov.beta_innov_var;
            //PX4_INFO("11");
        }
        
        // Flow will likely not be used, so these 2 terms will probably not count
        if (innov.flow_innov_var[0] > 0) {
            residual_power += innov.flow_innov[0]*innov.flow_innov[0]/innov.flow_innov_var[0];
            //PX4_INFO("12");
        }
        if (innov.flow_innov_var[1] > 0) {
            residual_power += innov.flow_innov[1]*innov.flow_innov[1]/innov.flow_innov_var[1];
            //PX4_INFO("13");
        }
        if (innov.hagl_innov_var > 0) {
            residual_power += innov.hagl_innov*innov.hagl_innov/innov.hagl_innov_var;
            //PX4_INFO("14");
        }
        
        //PX4_INFO("residual power: %f",(double)residual_power);
        // Raise alarm if threshold is exceeded
        if (residual_power > _parameters.detect_threshold) {
            PX4_WARN("Fault Detected!");
        }
        
        // publish detector info
        detector_fd.residual_power = residual_power;
        detector_fd.detector_threshold = _parameters.detect_threshold;
        orb_publish(ORB_ID(fault_detection), _detector_pub, &detector_fd);
    }
    
    //unsubscribe from orb topics
    orb_unsubscribe(innov_sub);
    orb_unsubscribe(params_sub);
    
    //unadvertise published orb topics
    orb_unadvertise(_detector_pub);

	//clean up the detector instance and reset
	delete detector::instance;
	detector::instance = nullptr;

    return;
}


void Detector::task_main_trampoline(int argc, char *argv[])
{
	detector::instance->task_main();
}

int Detector::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("detector",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&Detector::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

int detector_main(int argc, char *argv[])
{
    if (argc < 2) {
		PX4_WARN("usage: detector {start|stop|status|example}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (detector::instance != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		detector::instance = new Detector();

		if (detector::instance == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}


		if (OK != detector::instance->start()) {
			delete detector::instance;
			detector::instance = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (detector::instance == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		detector::instance->exit();

		// wait for the destruction of the instance
		while (detector::instance != nullptr) {
			usleep(50000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "print")) {
		if (detector::instance != nullptr) {

			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		if (detector::instance) {
			PX4_WARN("running");
			detector::instance->print_status();
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}
	
	if (!strcmp(argv[1], "example")) {
		detector::instance->print_example_detection();
		return 0;
	}

	PX4_WARN("unrecognized command");
	return 1;
}
