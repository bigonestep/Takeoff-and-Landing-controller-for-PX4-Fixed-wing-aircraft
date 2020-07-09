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
 * @file ActuatorEffectivenessPlane.cpp
 *
 * Actuator effectiveness for a plane.
 *
 */

#include "ActuatorEffectivenessPlane.hpp"

ActuatorEffectivenessPlane::ActuatorEffectivenessPlane():
	ModuleParams(nullptr)
{

	float c_t = 34.0;
	// float l_y = 0.7f; //distance cg to center of aileron
	// float l_x = 0.7f; //distance cg to center of elevator and rudder

	// float A_ail = 0.6f * 0.04f;
	// float A_elev = 0.5f * 0.035f;
	// float A_rud = 0.18f * 0.04f;

	// float rho = 1.23;

	// float delta_ail_max = 20 / 180 * 3.14;
	// float delta_elev_max = 20 / 180 * 3.14;
	// float delta_rud_max = 30 / 180 * 3.14;

	// float airspeed = 15.0f;

	// float inv_scaling_sq = 1.0f / (airspeed_scaling * airspeed_scaling);
	// float inv_scaling_sq = 1.0f;

	// const float B_plane[NUM_AXES][NUM_ACTUATORS] = {
	// 	{ 2 * airspeed *airspeed *A_ail *rho * 3.14f * delta_ail_max * l_y, 2 * airspeed *airspeed *A_ail *rho * 3.14f * delta_ail_max * l_y, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{ 0.f, 0.f, 2 * airspeed *airspeed *A_elev *rho * 3.14f * delta_elev_max * l_x, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{ 0.f, 0.f, 0.f, -2 * airspeed *airspeed *A_rud *rho * 3.14f * delta_rud_max * l_x, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{ 0.f, 0.f, 0.f, 0.f, c_t, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	// };

	float c_ail = 7.5f; // with u = 13
	float c_elev = 5.5f; // with u = 13
	float c_rud = 3.5f; // with u = 13


	const float B_plane[NUM_AXES][NUM_ACTUATORS] = {
		{ c_ail, c_ail, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, c_elev, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, c_rud, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, c_t, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(B_plane);

}

bool
ActuatorEffectivenessPlane::update()
{
	bool updated = false;

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		updated = true;
	}

	return updated;
}

void
ActuatorEffectivenessPlane::updateAirspeedTilt(const float airspeed, const float tilt)
{
	_updated = true;

	float c_t = 34.0;

	float c_ail = 0.044;
	float c_elev = 0.033f;
	float c_rud = 0.021f;

	const float B_plane[NUM_AXES][NUM_ACTUATORS] = {
		{ c_ail *airspeed * airspeed, c_ail *airspeed * airspeed, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, c_elev *airspeed * airspeed, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, c_rud *airspeed * airspeed, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, c_t, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(B_plane);
}
