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
 * @file ActuatorEffectivenessTailsitterVTOL.hpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessTailsitterVTOL.hpp"

ActuatorEffectivenessTailsitterVTOL::ActuatorEffectivenessTailsitterVTOL()
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

bool
ActuatorEffectivenessTailsitterVTOL::update()
{
	if (_updated) {
		_updated = false;
		return true;
	}

	return false;
}

void
ActuatorEffectivenessTailsitterVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	ActuatorEffectiveness::setFlightPhase(flight_phase);

	_updated = true;

	// // Trim
	float airspeed = 0.0f;

	switch (_flight_phase) {
	case FlightPhase::HOVER_FLIGHT:  {
			airspeed = 0.0f;
			break;
		}

	case FlightPhase::FORWARD_FLIGHT: {
			airspeed = 12.0f;
			break;
		}

	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF: {
			airspeed = 8.f;
			break;
		}
	}


	// Trim: half throttle
	// const float thrust_hover = 1.25f;

	const float l_y_motors = 0.145f;
	const float l_y_elevons = 0.145f;
	const float l_z_elevons = 0.09f;
	const float c_t = 2.5f;

	const float A_elevon = 0.19f * 0.06f;
	// const float A_p = 0.0123;
	const float delta_max = 30;
	const float rho = 1.22;

	float v = 13.0f;

	float c_F_z = 2.f * 3.14f * A_elevon * rho * (v + airspeed) * (v + airspeed) * delta_max / 180.0f * 3.14f;
	c_F_z = 1.0f;

	// const float c_m = 0.025f;

	// Effectiveness
	const float tiltrotor_vtol[NUM_AXES][NUM_ACTUATORS] = {
		{ -c_t * l_y_motors, c_t * l_y_motors, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, -c_F_z * l_z_elevons, c_F_z * l_z_elevons, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, c_F_z * l_y_elevons, c_F_z * l_y_elevons, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ -c_t, -c_t, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};

	_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(tiltrotor_vtol);


}
