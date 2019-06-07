/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualAcceleration.cpp
 */

#include "FlightTaskManualAcceleration.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

FlightTaskManualAcceleration::FlightTaskManualAcceleration()
{

}

bool FlightTaskManualAcceleration::updateInitialize()
{
	bool ret = FlightTaskManual::updateInitialize();
	return ret;
}

bool FlightTaskManualAcceleration::activate(vehicle_local_position_setpoint_s last_setpoint)
{
	bool ret = FlightTaskManual::activate(last_setpoint);
	return ret;
}

bool FlightTaskManualAcceleration::update()
{
	// Yaw
	_position_lock.updateYawFromStick(_yawspeed_setpoint, _yaw_setpoint,
					  _sticks_expo(3) * math::radians(_param_mpc_man_y_max.get()), _yaw, _deltatime);
	_yaw_setpoint = _position_lock.updateYawReset(_yaw_setpoint, _sub_attitude->get().quat_reset_counter,
			Quatf(_sub_attitude->get().delta_q_reset));

	// Map stick input to acceleration
	Vector2f stick_xy(&_sticks(0));
	_position_lock.limitStickUnitLengthXY(stick_xy);
	_position_lock.rotateIntoHeadingFrameXY(stick_xy, _yaw, _yaw_setpoint);

	_acceleration_setpoint = Vector3f(stick_xy(0), stick_xy(1), _sticks(2));

	_acceleration_setpoint *= 10;

	// Add drag to limit speed and brake again
	_acceleration_setpoint -= 2.f * _velocity;

	_constraints.want_takeoff = _checkTakeoff();
	return true;
}
