/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file flapping.cpp
 *
 * Controller library code
 */

#include "flapping.hpp"

namespace control
{

namespace flapping
{

void BlockFlappingController::update() {
	//check for new updates
	if (_param_update.updated()) updateParams();

	//new info from subscriptions
	updateSubscriptions();

	//set initial values
	_aileron = 0.0015;
	_elevator = 0.0015;
	_throttle = 0.0011;
	_rudder = 0.0015;
	i=1;

	//everything is manual for now

	//update aileron, for -10 to 10 scale
	_aileron = (_manual.pitch - 0.0015)*24000;
	
	//update throttle, for 0 to 10 scale
	_throttle = (_manual.throttle - 0.0011)*12000;

	//update elevator, for -10 to 10 scale
	_elevator = (0.0015 - _manual.elevator)*24000;

	//update rudder, for -10 to 10 scale
	_rudder = (_manual.rudder - 0.0015)*24000;

	setCycle();
	battery();
	flapping();
	pulse();
	i++;
	if i > cycle {
		i = 1;
	}


int BlockFlappingController::setCycle() {
	//set the length of a cycle based on current throttle setting
	if _throttle > 8.0 {
		cycle = 10;
		return;
	}
	else if _throttle > 6.0 {
		cycle = 15;
		return;
	}
	else if _throttle > 4.0 {
		cycle = 20;
		return;
	}
	else {
		cycle = 30;
	}
	return;
}

void BlockFlappingController::battery() {
	//make sure battery has enough juice. If not, glide.
	//TODO implement
	return;
}

void BlockFlappingController::flapping() {
	//set angle for each wing
	float down = -25.0;
	float up = 30.0;
	if _throttle > 2.0 { //flapping cycle
		if i > 8 {
			wingAngleLeft = 8.0 - _elevator - _aileron;
			wingAngleRight = 8.0 - _elevator + _aileron;
		}
		else if i > 3 {
			wingAngleLeft = up - _elevator - _aileron*2.0;
			wingAngleRight = up - elevator + _aileron*2.0;
		}
		else {
			wingAngleLeft = down - _elevator - _aileron*2.0;
			wingAngleRight = down - _elevator + _aileron*2.0;
		}
	}
	else { //glide
		wingAngleLeft = 8.0 - _elevator - _aileron;
		wingAngleRight = 8.0 - _elevator + _aileron;
	}
	return
}

void BlockFlappingController::pulse() {
	//convert wing angles to commands for actuators
	pulseWidthLeft = 0.0015 + wingAngleLeft*0.000012;
	pulseWifthRight = 0.0015 - wingAngleRight*0.000012;
	//TODO send pwm to servos with delays as in the basic code below
	//_actuators.control[0] = ??;
	//_actuators.control[1] = ??;
	/* Call PulseOut(17, PulseWidthLeft, 1) ' Left servo
	Delay (0.0025 - PulseWidthLeft)
	Call PulseOut(18, PulseWidthRight, 1) ' Right servo
	Delay (0.0025 - PulseWidthRight)
	End Sub*/
	
	return;
}


} // namespace flapping

} // namespace control

