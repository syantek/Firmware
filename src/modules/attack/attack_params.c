/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file attack_params.c
 *
 * Parameters defined for controlling the specifics for cyber attacks for a
 * research project. XXX This should never see use outside of simulation, as it 
 * purposely degrades performance.
 *
 * @author Scott Yantek <>
 */

/*
 * Attack parameters, accessible via MAVLink
 */

/**
 * Attack master switch
 *
 * This is the master switch which determines whether any attacks are
 * performed or not. It allows for all desired attack types to be switched on
 * at once, so that different attack types can be combined and parameters can 
 * be adjusted without the attack starting until this switch is turned on.
 * The switch is on when its value is 1478 and off for all other values.
 *
 * @boolean
 * @group Attack
 */
PARAM_DEFINE_INT32(ATTACK_MSTR_SW, 0);

/**
 * Attack type
 *
 * Integer bitmask for setting the attack type. The value of this parameter 
 * corresponds to the type of attack that will be carried out when 
 * ATTACK_MSTR_SW is on. Generally, this should be set up before turning on the 
 * switch. Set bits to 1 to enable the attack type. Some of the attacks can be
 * mixed, but it does not make any practical sense to mix stealthy attacks with
 * other types of attack. TODO add more attack types as they are implemented
 *
 * 0 : pitch oscillation
 * 1 : stealthy to human observer (alter estimator position output)
 *
 * @group Attack
 * @min 0
 * @max 3
 * @bit 0 (+1) enable pitch oscillation attack
 * @bit 1 (+2) enable GPS attack
 */
PARAM_DEFINE_INT32(ATTACK_TYPE, 2);

PARAM_DEFINE_FLOAT(LAT_RAMP, 2e-11);
