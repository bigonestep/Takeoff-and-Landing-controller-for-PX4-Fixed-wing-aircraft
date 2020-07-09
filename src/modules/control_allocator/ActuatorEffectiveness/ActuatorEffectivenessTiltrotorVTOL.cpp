/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessTiltrotorVTOL.hpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessTiltrotorVTOL.hpp"

ActuatorEffectivenessTiltrotorVTOL::ActuatorEffectivenessTiltrotorVTOL()
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

bool
ActuatorEffectivenessTiltrotorVTOL::update()
{
	if (_updated) {
		_updated = false;
		return true;
	}

	return false;
}

void
ActuatorEffectivenessTiltrotorVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	ActuatorEffectiveness::setFlightPhase(flight_phase);

	_updated = true;

	// Trim
	float tilt = 0.0f;
	// float pi = 3.141;

	switch (_flight_phase) {
	case FlightPhase::HOVER_FLIGHT:  {
			tilt = 0.0f;
			break;
		}

	case FlightPhase::FORWARD_FLIGHT: {
			tilt = 1.5708f;
			break;
		}

	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF: {
			tilt = 0.7854f;
			break;
		}
	}

	const int tiltrotor_type = 1;

	if (tiltrotor_type == 0) {
		// quad

		// Trim: half throttle, tilted motors
		_trim(0) = 0.5f;
		_trim(1) = 0.5f;
		_trim(2) = 0.5f;
		_trim(3) = 0.5f;
		_trim(4) = tilt;
		_trim(5) = tilt;
		_trim(6) = tilt;
		_trim(7) = tilt;

		// Effectiveness
		const float tiltrotor_vtol[NUM_AXES][NUM_ACTUATORS] = {
			{-0.5f * cosf(_trim(4)),  0.5f * cosf(_trim(5)),  0.5f * cosf(_trim(6)), -0.5f * cosf(_trim(7)), 0.5f * _trim(0) *sinf(_trim(4)), -0.5f * _trim(1) *sinf(_trim(5)), -0.5f * _trim(2) *sinf(_trim(6)), 0.5f * _trim(3) *sinf(_trim(7)), -0.5f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{ 0.5f * cosf(_trim(4)), -0.5f * cosf(_trim(5)),  0.5f * cosf(_trim(6)), -0.5f * cosf(_trim(7)), -0.5f * _trim(0) *sinf(_trim(4)),  0.5f * _trim(1) *sinf(_trim(5)), -0.5f * _trim(2) *sinf(_trim(6)), 0.5f * _trim(3) *sinf(_trim(7)), 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{-0.5f * sinf(_trim(4)),  0.5f * sinf(_trim(5)),  0.5f * sinf(_trim(6)), -0.5f * sinf(_trim(7)), -0.5f * _trim(0) *cosf(_trim(4)), 0.5f * _trim(1) *cosf(_trim(5)), 0.5f * _trim(2) *cosf(_trim(6)), -0.5f * _trim(3) *cosf(_trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{ 0.25f * sinf(_trim(4)), 0.25f * sinf(_trim(5)), 0.25f * sinf(_trim(6)), 0.25f * sinf(_trim(7)), 0.25f * _trim(0) *cosf(_trim(4)), 0.25f * _trim(1) *cosf(_trim(5)), 0.25f * _trim(2) *cosf(_trim(6)), 0.25f * _trim(3) *cosf(_trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{-0.25f * cosf(_trim(4)), -0.25f * cosf(_trim(5)), -0.25f * cosf(_trim(6)), -0.25f * cosf(_trim(7)), 0.25f * _trim(0) *sinf(_trim(4)), 0.25f * _trim(1) *sinf(_trim(5)), 0.25f * _trim(2) *sinf(_trim(6)), 0.25f * _trim(3) *sinf(_trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
		};
		_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(tiltrotor_vtol);

	} else {
		// tri

		// Trim: half throttle, tilted motors
		_trim(0) = 0.5f;
		_trim(1) = 0.5f;
		_trim(2) = 0.5f;
		_trim(3) = 0.0f;
		_trim(4) = 0.0f;
		_trim(5) = 0.0f;

		const float l_x_front = 0.06f;
		const float l_x_rear = 0.24f;
		const float l_y_front = 0.20f;
		const float c_t_front = 8.f;
		const float c_t_rear = 4.f;
		// const float c_m_front = 0.05f;
		// const float c_m_rear = 0.025f;

		// Effectiveness
		const float tiltrotor_vtol[NUM_AXES][NUM_ACTUATORS] = {
			{-c_t_front *l_y_front * cosf(_trim(4)), c_t_front *l_y_front * cosf(_trim(5)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{ c_t_front *l_x_front * cosf(_trim(4)), c_t_front *l_x_front * cosf(_trim(5)),  -c_t_rear *l_x_rear * 1.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{ 0.f, 0.f, 0.0f, 0.f, -c_t_front *l_y_front * _trim(0), c_t_front *l_y_front * _trim(1), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
			{ -c_t_front * cosf(_trim(4)), -c_t_front * cosf(_trim(5)), -c_t_rear, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
		};

		_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(tiltrotor_vtol);
	}

}

void
ActuatorEffectivenessTiltrotorVTOL::updateAirspeedTilt(const float airspeed, const float tilt)
{
	_updated = true;

	const float l_x_front = 0.06f;
	const float l_x_rear = 0.24f;
	const float l_y_front = 0.20f;
	const float c_t_front = 8.f;
	const float c_t_rear = 4.f;
	const float c_m_front = 0.05f;
	const float c_m_rear = 0.025f;

	// const float A_elev = 0.17f * 0.04f;
	const float l_x_elev = 0.19f;
	const float l_y_elev = 0.19f;
	// const float delta_elev_max = 30.0f /180.f * 3.141f;

	const float c_elev = 0.03f; //without l_x/l_y

	float tilt_rad = tilt + 0.01f;

	// tilt_rad = 0.f;
	// -c_t_front *l_y_front * sinf(tilt_rad), c_t_front *l_y_front * sinf(tilt_rad)
	// -c_t_front *l_y_front * 0.5f, c_t_front *l_y_front * 0.5f

	const float tiltrotor_vtol[NUM_AXES][NUM_ACTUATORS] = {
		{-c_t_front *l_y_front * cosf(tilt_rad), c_t_front *l_y_front * cosf(tilt_rad), 0.f, 0.f, 0.f, 0.0f, c_elev *l_y_elev *airspeed * airspeed, c_elev *l_y_elev *airspeed * airspeed, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ c_t_front *l_x_front * cosf(tilt_rad), c_t_front *l_x_front * cosf(tilt_rad),  -c_t_rear *l_x_rear * 1.0f, 0.f, 0.f, 0.0f, c_elev *l_x_elev *airspeed * airspeed, -c_elev *l_x_elev *airspeed * airspeed, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ -c_t_front *l_y_front * sinf(tilt_rad) - c_m_front, c_t_front *l_y_front * sinf(tilt_rad) + c_m_front, c_m_rear, 0.f, -c_t_front *l_y_front * 0.5f, c_t_front *l_y_front * 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ -c_t_front * cosf(tilt_rad), -c_t_front * cosf(tilt_rad), -c_t_rear, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(tiltrotor_vtol);
}
