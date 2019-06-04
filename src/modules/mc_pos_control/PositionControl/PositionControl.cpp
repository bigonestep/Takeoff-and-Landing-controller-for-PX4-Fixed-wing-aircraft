/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <ControlMath.hpp>
#include <px4_defines.h>

using namespace matrix;

PositionControl::PositionControl(ModuleParams *parent) :
	ModuleParams(parent)
{}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

void PositionControl::setConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.

	const float tilt_max_radians = math::radians(math::max(_param_mpc_tiltmax_air.get(), _param_mpc_man_tilt_max.get()));

	if (!PX4_ISFINITE(constraints.tilt)
	    || !(constraints.tilt < tilt_max_radians)) {
		_constraints.tilt = tilt_max_radians;
	}

	if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < _param_mpc_z_vel_max_up.get())) {
		_constraints.speed_up = _param_mpc_z_vel_max_up.get();
	}

	if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < _param_mpc_z_vel_max_dn.get())) {
		_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	}

	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < _param_mpc_xy_vel_max.get())) {
		_constraints.speed_xy = _param_mpc_xy_vel_max.get();
	}
}

void PositionControl::update(const float dt)
{
	_positionController();
	_velocityController(dt);

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
}

void PositionControl::_positionController()
{
	// P-position controller
	const Vector3f propotional_gain(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get());
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(propotional_gain);
	_addIfNotNanVector(_vel_sp, vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
				   Vector2f(_vel_sp - vel_sp_position), _param_mpc_xy_vel_max.get());
	_vel_sp(0) = vel_sp_xy(0);
	_vel_sp(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}

void PositionControl::_velocityController(const float &dt)
{
	// Generate desired thrust setpoint.
	// PID
	// u_des = P(vel_error) + D(vel_error_dot) + I(vel_integral)
	// Umin <= u_des <= Umax
	//
	// Anti-Windup:
	// u_des = _thr_sp; r = _vel_sp; y = _vel
	// u_des >= Umax and r - y >= 0 => Saturation = true
	// u_des >= Umax and r - y <= 0 => Saturation = false
	// u_des <= Umin and r - y <= 0 => Saturation = true
	// u_des <= Umin and r - y >= 0 => Saturation = false
	//
	// 	Notes:
	// - PID implementation is in NED-frame
	// - control output in D-direction has priority over NE-direction
	// - the equilibrium point for the PID is at hover-thrust
	// - the maximum tilt cannot exceed 90 degrees. This means that it is
	// 	 not possible to have a desired thrust direction pointing in the positive
	// 	 D-direction (= downward)
	// - the desired thrust in D-direction is limited by the thrust limits
	// - the desired thrust in NE-direction is limited by the thrust excess after
	// 	 consideration of the desired thrust in D-direction. In addition, the thrust in
	// 	 NE-direction is also limited by the maximum tilt.
	static constexpr float CONSTANTS_ONE_G = 9.80665f; // m/s^2
	const float hover_scale = CONSTANTS_ONE_G / _param_mpc_thr_hover.get(); // For backwards compatibility of the gains
	const Vector3f vel_gain_p(_param_mpc_xy_vel_p.get(), _param_mpc_xy_vel_p.get(), _param_mpc_z_vel_p.get());
	const Vector3f vel_gain_i(_param_mpc_xy_vel_i.get(), _param_mpc_xy_vel_i.get(), _param_mpc_z_vel_i.get());
	const Vector3f vel_gain_d(_param_mpc_xy_vel_d.get(), _param_mpc_xy_vel_d.get(), _param_mpc_z_vel_d.get());

	// Velocity PID
	const Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(vel_gain_p) + _vel_dot.emult(vel_gain_d) + _vel_int;

	acc_sp_velocity *= hover_scale;
	_addIfNotNanVector(_acc_sp, acc_sp_velocity);
	Vector3f thr_sp_velocity = (_acc_sp / hover_scale) - Vector3f(0, 0, _param_mpc_thr_hover.get());

	// The Thrust limits are negated and swapped due to NED-frame.
	const float uMax = math::min(-_param_mpc_thr_min.get(), -10e-4f);
	const float uMin = -_param_mpc_thr_max.get();

	// Apply Anti-Windup in vertical direction
	const bool stop_integral_D = (thr_sp_velocity(2) >= uMax && vel_error(2) >= 0.0f) ||
				     (thr_sp_velocity(2) <= uMin && vel_error(2) <= 0.0f);

	if (!stop_integral_D) {
		_vel_int(2) += vel_error(2) * vel_gain_i(2) * dt;

		// limit thrust integral
		_vel_int(2) = math::min(fabsf(_vel_int(2)), _param_mpc_thr_max.get()) * math::sign(_vel_int(2));
	}

	// Saturate thrust setpoint in vertical direction
	thr_sp_velocity(2) = math::constrain(thr_sp_velocity(2), uMin, uMax);

	if (PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))) {
		// Thrust set-point in NE-direction is already provided. Only
		// scaling by the maximum tilt is required.
		float thr_xy_max = fabsf(thr_sp_velocity(2)) * tanf(_constraints.tilt);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;
		thr_sp_velocity(0) = thr_sp_velocity(1) = 0.f;

	} else {
		// Get maximum allowed horizontal thrust based on tilt and excess thrust
		const float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		float thrust_max_NE = sqrtf(_param_mpc_thr_max.get() * _param_mpc_thr_max.get() - _thr_sp(2) * _thr_sp(2));
		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

		// Saturate thrust in horizontal direction.
		Vector2f thrust_sp_xy(thr_sp_velocity);

		if (thrust_sp_xy.norm_squared() > thrust_max_NE * thrust_max_NE) {
			const float mag = thrust_sp_xy.length();
			thr_sp_velocity(0) = thrust_sp_xy(0) / mag * thrust_max_NE;
			thr_sp_velocity(1) = thrust_sp_xy(1) / mag * thrust_max_NE;
		}

		// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		const float arw_gain = 2.f / _param_mpc_xy_vel_p.get();
		const Vector2f vel_error_lim = Vector2f(vel_error) - (arw_gain * (thrust_sp_xy - Vector2f(thr_sp_velocity)));

		// Update integral
		_vel_int(0) += vel_error_lim(0) * vel_gain_i(0) * dt;
		_vel_int(1) += vel_error_lim(1) * vel_gain_i(1) * dt;
	}

	// No control input from setpoints or corresponding states which are NAN, reset integrator if necessary
	_addIfNotNanVector(_vel_int, Vector3f());
	_addIfNotNanVector(_thr_sp, thr_sp_velocity);
}

void PositionControl::getOutputSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint)
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::updateParams()
{
	ModuleParams::updateParams();
}

void PositionControl::_addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void PositionControl::_addIfNotNanVector(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		_addIfNotNan(setpoint(i), addition(i));
	}
}
