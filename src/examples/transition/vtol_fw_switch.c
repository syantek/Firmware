/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file vtol_fw_switch.c
 * A multiplexing thread for switching between a fixed_wing and multirotor controllers in order to fly a VTOL flying wing with multiple propellers and ailerons
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <poll.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int vtol_fw_switch_main(int argc, char *argv[]);

PARAM_DEFINE_INT32(VFW_SWITCH, 1);

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int vtol_fw_switch_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int vtol_fw_switch_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int vtol_fw_switch_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");
	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}
		thread_should_exit = false;
		daemon_task = task_spawn_cmd("daemon",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 vtol_fw_switch_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}
	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}
	usage("unrecognized command");
	exit(1);
}

#define SWITCH_SET_MULTIROTOR 0
#define SWITCH_SET_PLANE 1
static int vtol_switch = SWITCH_SET_PLANE;
int vtol_fw_switch_thread_main(int argc, char *argv[])
{
	warnx("vtol_fw_switch starting\n");
	thread_running = true;
	printf("Hello Sky!\n");
	//param_get(param_find("VFW_SWITCH"), &vtol_switch);
	printf("switch is: %d\n",vtol_switch);
	/* subscribe to sensor_combined topic */
	int plane_fd = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	int multirotor_fd = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	orb_set_interval(plane_fd, 1000);
	orb_set_interval(multirotor_fd, 1333);

	/* advertise attitude topic */
	struct actuator_controls_s act_ctls;
	memset(&act_ctls, 0, sizeof(act_ctls));
	orb_advert_t act_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &act_ctls);

	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
			{ .fd = plane_fd,   .events = POLLIN },
			{ .fd = multirotor_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};
	printf("plane_fd %x\n",plane_fd);
	printf("multi_fd %x\n",multirotor_fd);
	int error_counter = 0;

	while (!thread_should_exit) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		printf("about to poll\n");
		int poll_ret = poll(fds, 2, 10000);
		printf("did  poll= %x\n",poll_ret);
	 
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("[vtol_fw_switch] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[vtol_fw_switch] ERROR return value from poll(): %d\n"	, poll_ret);
			}
			error_counter++;
		} else {
			printf("about to check fds[0]\n");
			if ((fds[0].revents & POLLIN)) {
				printf("about to plane\n");
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, plane_fd, &act_ctls);
				if (vtol_switch == SWITCH_SET_PLANE){ printf("                publish plane\n"); orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, act_pub, &act_ctls); }
			}
			printf("about to check fds[1]\n");
 			if ((fds[1].revents & POLLIN)) {
				printf("about to multi\n");
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, multirotor_fd, &act_ctls);
				if (vtol_switch == SWITCH_SET_MULTIROTOR){ printf("                publish multi\n"); orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, act_pub, &act_ctls); }
			}
			printf("done loop\n");
 		}
	}
	warnx("exited loop\n");
	close(plane_fd);
	close(multirotor_fd);
	close(act_pub);
	thread_running = false;
	warnx("about to flush\n");
	fflush(stdout);
	exit(0);
	return 0;
}
