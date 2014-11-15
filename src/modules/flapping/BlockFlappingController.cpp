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
	// wait for a estimator update or exit at 100 Hz 
	// running twice as fast as pwm (50 Hz) to avoid lag issues
	if (poll(&_attPoll, 1, 10) < 0) return; // poll error

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
	for (unsigned i = 2; i < NUM_ACTUATOR_CONTROLS; i++) {
		_actuators_0.control[i] = 0.0f;
		_actuators_1.control[i] = 0.0f;
	}

	float learning = _learning.get();
	if (learning > 1.0f) {
		//TODO learning stuff
	}

	// default controls
	float elevator = 0.0f;
	float aileron = 0.0f;
	float throttle = 0.0f;

	// handle autopilot modes
	if ((_status.main_state == MAIN_STATE_AUTO_MISSION) ||
		(_status.main_state == MAIN_STATE_AUTO_RTL) ||
		(_status.main_state == MAIN_STATE_AUTO_LOITER)) {
		// TODO
	} else if (_status.main_state == MAIN_STATE_MANUAL) {
		elevator = _manual.x;
		aileron = _manual.y;
		throttle = _manual.z;
	} else {
		elevator = _manual.x;
		aileron = _manual.y;
		throttle = _manual.z;
	}

	//set the cycle frequency based on current throttle setting
	float cycleFrequency = 0.2f;
	cycleFrequencyFunction(throttle, cycleFrequency);
	//printf("cycle frequency : %10.2f\n", cycleFrequency);

	// find cycle time
	float t = (_timeStamp - _cycleStartTimeStamp)/ 1.0e6f;

	// set cycle start timestamp, wrap time if new period
	float cyclePeriod = 1.0f/cycleFrequency;
	//printf("cycle period : %10.2f\n", cyclePeriod);
	if (t > cyclePeriod) {
		t -= cyclePeriod*int(t/cyclePeriod);
		_cycleStartTimeStamp = _timeStamp - t*1.0e6f;
	}

	// flapping cycle function
	float wingLeft = 0;
	float wingRight = 0;
	flappingFunction(t, aileron, elevator, throttle, wingLeft, wingRight);

	// actuators 0
	_actuators_0.timestamp = _timeStamp;
	_actuators_0.control[CH_LEFT] = wingLeft;
	_actuators_0.control[CH_RIGHT] = wingRight;

	// actuators 1
	_actuators_1.timestamp = _timeStamp;
	_actuators_1.control[CH_LEFT] = wingLeft;
	_actuators_1.control[CH_RIGHT] = wingRight;

	//printf("left: %10.2f\tright: %10.2f\n", wingLeft, wingRight);

	// update all publications
	updatePublications();
}

void BlockFlappingController::cycleFrequencyFunction(
		float throttle, float & cycleFrequency) {
	//set the length of a cycle based on current throttle setting
	float b = _minFrequency.get();
	float m = _throttle2Frequency.get();
	float y = m*throttle + b;
	cycleFrequency = y;
	//printf("x : %10.2f\n", throttle);
	//printf("m : %10.2f\n", m);
	//printf("b : %10.2f\n", b);
	//printf("cycle freq in func : %10.2f\n", y);
}

void BlockFlappingController::flappingFunction(
		float t,
		float aileron,
		float elevator, float throttle,
		float & wingLeft, float & wingRight) {

	// function parameters
	float servoTravel = _servoTravel.get();
	float wingUp = _wingUp.get()/servoTravel;
	float wingDown = _wingDown.get()/servoTravel;
	float wingGlide = _wingGlide.get()/servoTravel;
	float tDown2Up = _tDown2Up.get();
	float tUp2Glide = _tUp2Glide.get();
	float throttleGlide = _throttleGlide.get();
	
	if (throttle > throttleGlide) {
		if (t < tDown2Up) { // wing down
			wingLeft = wingDown - elevator - 2*aileron;
			wingRight = wingDown - elevator + 2*aileron;
		} else if (t < tUp2Glide) { // wing up
			wingLeft = wingUp - elevator - 2*aileron;
			wingRight = wingUp - elevator + 2*aileron;
		} else { // glide
			wingLeft = wingGlide - elevator - aileron;
			wingRight = wingGlide - elevator + aileron;
		}
	} else { // glide
		wingLeft = wingGlide - elevator - aileron;
		wingRight = wingGlide - elevator + aileron;
	}
}

//TODO
void BlockFlappingController::learningFunction() {
	/* Pseudocode
	initialize parent, child, and new populations
	initialize members of parent population randomly
	for each generation:
		copy parent into child pop
		mutate child pop
		for each child:
			use child's params for a couple cycles
			fitness = -energy used
			record genome and fitness to file
		apply selective pressure based on fitness to get new pop
		new pop becomes parent pop
	*/
	
	return;
}

/*
string BlockFlappingController::getLearningParams(float num) {
	int temp = num;
	switch (temp) {
		case 0:
			return "TH2V_P";
		case 1:
			return "The2V_I";
		case 2:
			return "TH2V_I_MAX";
		case 3:
			return "Q2V";
		case 4:
			return "SRV_TRV";
		case 5:
			return "WNG_UP";
		case 6:
			return "WNG_DWN";
		case 7:
			return "WNG_GLD";
		case 8:
			return "T_DWN2UP";
		case 9:
			return "T_UP2GLD";
		case 1:
			return "THR_GLD";
		case 1:
			return "THR2FREQ";
		case 1:
			return "MIN_FREQ";
		default:
			return "ERROR";
		}

}*/
