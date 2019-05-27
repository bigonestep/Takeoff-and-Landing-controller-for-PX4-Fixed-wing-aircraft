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

bool FlightTaskManualAcceleration::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTaskManual::initializeSubscriptions(subscription_array)) {
		return false;
	}
	return true;
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
	_velocity_setpoint = Vector3f();
	// Map sticks input with deadzone and exponential curve to acceleration and yawspeed
	_acceleration_setpoint = Vector3f(&_sticks_expo(0)) * 10;
	_yawspeed_setpoint = _sticks_expo(3) * math::radians(_param_mpc_man_y_max.get());

	printf("ACC IN TASK:\n");
	_acceleration_setpoint.print();

	// Rotate horizontal acceleration input to body heading
	float yaw_rotate = PX4_ISFINITE(_yaw_setpoint) ? _yaw_setpoint : _yaw;
	Vector3f v_r = Vector3f(Dcmf(Eulerf(0.0f, 0.0f, yaw_rotate)) * Vector3f(_acceleration_setpoint(0), _acceleration_setpoint(1), 0.0f));
	_acceleration_setpoint(0) = v_r(0);
	_acceleration_setpoint(1) = v_r(1);

	_yaw_setpoint = _position_lock.updateYawLock(_yawspeed, _yaw, _yawspeed_setpoint, _yaw_setpoint);
	_yaw_setpoint = _position_lock.updateYawReset(_yaw_setpoint, _sub_attitude->get().quat_reset_counter, Quatf(_sub_attitude->get().delta_q_reset));

	return true;
}
