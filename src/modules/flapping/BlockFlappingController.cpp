/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *   Author: James Goppert, Scott Yantek
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

#include "BlockFlappingController.hpp"

void BlockFlappingController::update() {
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_attPoll, 1, 100) < 0) return; // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) return;

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
	if (_param_update.updated()) updateParams();

	// get new information from subscriptions
	updateSubscriptions();

	// default all output to zero unless handled by mode
	for (unsigned i = 2; i < NUM_ACTUATOR_CONTROLS; i++)
		_actuators.control[i] = 0.0f;

	// default controls
	float elevator = 0.0f;
	float aileron = 0.0f;
	float throttle = 0.0f;

	// handle autopilot modes
	if (_status.main_state == MAIN_STATE_AUTO)  {
		// TODO
	} else if (_status.main_state == MAIN_STATE_MANUAL) {
		elevator = _manual.x;
		aileron = _manual.y;
		throttle = _manual.z;
	}

	//set the cycle period based on current throttle setting
	float cyclePeriod = 0.0f;
	cyclePeriodFunction(throttle, cyclePeriod);

	// find cycle time
	float t = (_timeStamp - _cycleStartTimeStamp)/ 1.0e6f;

	// set cycle start timestamp, wrap time if new period
	if (t > cyclePeriod) {
		t -= cyclePeriod*int(t/cyclePeriod);
		_cycleStartTimeStamp = _timeStamp - t*1.0e6f;
	}

	// flapping cycle function
	flappingFunction(t, aileron, elevator, throttle,
		_actuators.control[CH_LEFT], 
		_actuators.control[CH_RIGHT]);

	// update all publications
	updatePublications();
}

void BlockFlappingController::cyclePeriodFunction(
		float throttle, float & cyclePeriod) {
	//set the length of a cycle based on current throttle setting
	if (throttle < 0.4f) {
		cyclePeriod = 0.6f;
	} else if (throttle < 1.0f) {
		cyclePeriod = 0.6f - 0.4f*throttle;
	}
}

void BlockFlappingController::flappingFunction(
		float t,
		float aileron,
		float elevator, float throttle,
		float & wingLeft, float & wingRight) {

	// function parameters
	const float servo_travel = 90.0f; // deg
	const float wing_up = 30.0f/servo_travel; // normalized
	const float wing_down = -25.0f/servo_travel;
	const float wing_glide = 8.0f/servo_travel;
	const float t_down2up = 0.06f; // t of transition
	const float t_up2glide = 0.16f; // t of transition
	const float throttle_glide = 0.2f; // glide below this
	
	float wing_left = 0.0f; // normalized
	float wing_right = 0.0f;
	if (throttle > throttle_glide) {
		if (t < t_down2up) { // wing down
			wing_left = wing_down - elevator - 2*aileron;
			wing_right = wing_down - elevator + 2*aileron;
		} else if (t < t_up2glide) { // wing up
			wing_left = wing_down - elevator - 2*aileron;
			wing_right = wing_down - elevator + 2*aileron;
		} else { // glide
			wing_left = wing_glide - elevator - aileron;
			wing_right = wing_glide - elevator + aileron;
		}
	} else { // glide
		wing_left = wing_glide - elevator - aileron;
		wing_right = wing_glide - elevator + aileron;
	}
}
