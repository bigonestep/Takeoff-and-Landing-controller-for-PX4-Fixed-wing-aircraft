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
 * @file mavlink_simulator.h
 * MAVLink simulator
 *
 */

#pragma once

#include <ecl/geo/geo.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>

#include "mavlink_main.h"

class Mavlink;

class MavlinkSimulationManager
{
public:
	MavlinkSimulationManager(Mavlink *parent);
	~MavlinkSimulationManager() = default;

	void handle_message(mavlink_message_t *msg);

private:

	void handle_message_hil_gps(mavlink_message_t *msg);
	void handle_message_hil_optical_flow(mavlink_message_t *msg);
	void handle_message_hil_sensor(mavlink_message_t *msg);
	void handle_message_hil_state_quaternion(mavlink_message_t *msg);

	Mavlink				*_mavlink;

	// ORB publications
	uORB::Publication<airspeed_s>				_airspeed_pub{ORB_ID(airspeed)};
	uORB::Publication<optical_flow_s>			_flow_pub{ORB_ID(optical_flow)};
	uORB::Publication<vehicle_attitude_s>			_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>		_global_pos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_gps_position_s>		_gps_pub{ORB_ID(vehicle_gps_position)};
	uORB::Publication<vehicle_local_position_s>		_local_pos_pub{ORB_ID(vehicle_local_position)};

	// ORB publications (multi)
	uORB::PublicationMulti<distance_sensor_s>		_flow_distance_sensor_pub{ORB_ID(distance_sensor), ORB_PRIO_LOW};
	uORB::PublicationMulti<sensor_accel_s>			_accel_pub{ORB_ID(sensor_accel), ORB_PRIO_LOW};
	uORB::PublicationMulti<sensor_baro_s>			_baro_pub{ORB_ID(sensor_baro), ORB_PRIO_LOW};
	uORB::PublicationMulti<sensor_gyro_s>			_gyro_pub{ORB_ID(sensor_gyro), ORB_PRIO_LOW};
	uORB::PublicationMulti<sensor_mag_s>			_mag_pub{ORB_ID(sensor_mag), ORB_PRIO_LOW};

	// ORB subscriptions
	uORB::Subscription	_actuator_outputs_sub{ORB_ID(actuator_outputs)};

	map_projection_reference_s	_hil_local_proj_ref{};
	float				_hil_local_alt0{0.0f};
	bool				_hil_local_proj_inited{false};

};
