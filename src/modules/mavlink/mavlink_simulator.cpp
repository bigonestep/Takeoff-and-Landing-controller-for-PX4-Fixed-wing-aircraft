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
 * @file mavlink_simulator.cpp
 * MAVLink simulator message receive and dispatch
 *
 */

#include "mavlink_simulator.h"

#include <lib/airspeed/airspeed.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

MavlinkSimulationManager::MavlinkSimulationManager(Mavlink *parent) :
	_mavlink(parent)
{
}

void MavlinkSimulationManager::handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		handle_message_hil_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
		handle_message_hil_state_quaternion(msg);
		break;

	case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		handle_message_hil_optical_flow(msg);
		break;

	default:
		break;
	}

	if (_mavlink->get_hil_enabled() || (_mavlink->get_use_hil_gps() && msg->sysid == mavlink_system.sysid)) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_GPS:
			handle_message_hil_gps(msg);
			break;

		default:
			break;
		}
	}
}

void MavlinkSimulationManager::handle_message_hil_optical_flow(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	optical_flow_s f{};

	f.timestamp = hrt_absolute_time(); // XXX we rely on the system time for now and not flow.time_usec;
	f.integration_timespan = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral = flow.integrated_xgyro;
	f.gyro_y_rate_integral = flow.integrated_ygyro;
	f.gyro_z_rate_integral = flow.integrated_zgyro;
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.ground_distance_m = flow.distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;
	f.gyro_temperature = flow.temperature;

	_flow_pub.publish(f);

	/* Use distance value for distance sensor topic */
	distance_sensor_s d{};

	d.timestamp = hrt_absolute_time();
	d.min_distance = 0.3f;
	d.max_distance = 5.0f;
	d.current_distance = flow.distance; /* both are in m */
	d.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	d.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	_flow_distance_sensor_pub.publish(d);
}

void MavlinkSimulationManager::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	const uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		airspeed_s airspeed{};

		float ias = calc_IAS(imu.diff_pressure * 1e2f);
		float scale = 1.0f; //assume no instrument or pitot placement errors
		float eas = calc_EAS_from_IAS(ias, scale);
		float tas = calc_TAS_from_EAS(eas, imu.abs_pressure * 100, imu.temperature);

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = ias;
		airspeed.true_airspeed_m_s = tas;

		_airspeed_pub.publish(airspeed);
	}

	/* gyro */
	{
		sensor_gyro_s gyro{};

		gyro.timestamp = timestamp;
		gyro.x_raw = imu.xgyro * 1000.0f;
		gyro.y_raw = imu.ygyro * 1000.0f;
		gyro.z_raw = imu.zgyro * 1000.0f;
		gyro.x = imu.xgyro;
		gyro.y = imu.ygyro;
		gyro.z = imu.zgyro;
		gyro.temperature = imu.temperature;

		_gyro_pub.publish(gyro);
	}

	/* accelerometer */
	{
		sensor_accel_s accel{};

		accel.timestamp = timestamp;
		accel.x_raw = imu.xacc / (CONSTANTS_ONE_G / 1000.0f);
		accel.y_raw = imu.yacc / (CONSTANTS_ONE_G / 1000.0f);
		accel.z_raw = imu.zacc / (CONSTANTS_ONE_G / 1000.0f);
		accel.x = imu.xacc;
		accel.y = imu.yacc;
		accel.z = imu.zacc;
		accel.temperature = imu.temperature;

		_accel_pub.publish(accel);
	}

	/* magnetometer */
	{
		sensor_mag_s mag{};

		mag.timestamp = timestamp;
		mag.x_raw = imu.xmag * 1000.0f;
		mag.y_raw = imu.ymag * 1000.0f;
		mag.z_raw = imu.zmag * 1000.0f;
		mag.x = imu.xmag;
		mag.y = imu.ymag;
		mag.z = imu.zmag;

		_mag_pub.publish(mag);
	}

	/* baro */
	{
		sensor_baro_s baro{};

		baro.timestamp = timestamp;
		baro.pressure = imu.abs_pressure;
		baro.temperature = imu.temperature;

		/* fake device ID */
		baro.device_id = 1234567;

		_baro_pub.publish(baro);
	}
}

void MavlinkSimulationManager::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	const uint64_t timestamp = hrt_absolute_time();

	vehicle_gps_position_s hil_gps{};

	hil_gps.timestamp_time_relative = 0;
	hil_gps.time_utc_usec = gps.time_usec;

	hil_gps.timestamp = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.s_variance_m_s = 0.1f;

	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = ((gps.cog == 65535) ? NAN : matrix::wrap_2pi(math::radians(gps.cog * 1e-2f)));

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	hil_gps.heading = NAN;
	hil_gps.heading_offset = NAN;

	_gps_pub.publish(hil_gps);
}

void MavlinkSimulationManager::handle_message_hil_state_quaternion(mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	const uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		airspeed_s airspeed{};

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = hil_state.ind_airspeed * 1e-2f;
		airspeed.true_airspeed_m_s = hil_state.true_airspeed * 1e-2f;

		_airspeed_pub.publish(airspeed);
	}

	/* attitude */
	{
		vehicle_attitude_s hil_attitude{};

		hil_attitude.timestamp = timestamp;

		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);

		_attitude_pub.publish(hil_attitude);
	}

	/* global position */
	{
		vehicle_global_position_s hil_global_pos{};

		hil_global_pos.timestamp = timestamp;
		hil_global_pos.lat = hil_state.lat / ((double)1e7);
		hil_global_pos.lon = hil_state.lon / ((double)1e7);
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.vel_n = hil_state.vx / 100.0f;
		hil_global_pos.vel_e = hil_state.vy / 100.0f;
		hil_global_pos.vel_d = hil_state.vz / 100.0f;
		hil_global_pos.eph = 2.0f;
		hil_global_pos.epv = 4.0f;

		_global_pos_pub.publish(hil_global_pos);
	}

	/* local position */
	{
		double lat = hil_state.lat * 1e-7;
		double lon = hil_state.lon * 1e-7;

		if (!_hil_local_proj_inited) {
			_hil_local_proj_inited = true;
			_hil_local_alt0 = hil_state.alt / 1000.0f;

			map_projection_init(&_hil_local_proj_ref, lat, lon);
		}

		float x = 0.0f;
		float y = 0.0f;
		map_projection_project(&_hil_local_proj_ref, lat, lon, &x, &y);

		vehicle_local_position_s hil_local_pos{};
		hil_local_pos.timestamp = timestamp;

		hil_local_pos.ref_timestamp = _hil_local_proj_ref.timestamp;
		hil_local_pos.ref_lat = math::radians(_hil_local_proj_ref.lat_rad);
		hil_local_pos.ref_lon = math::radians(_hil_local_proj_ref.lon_rad);
		hil_local_pos.ref_alt = _hil_local_alt0;
		hil_local_pos.xy_valid = true;
		hil_local_pos.z_valid = true;
		hil_local_pos.v_xy_valid = true;
		hil_local_pos.v_z_valid = true;
		hil_local_pos.x = x;
		hil_local_pos.y = y;
		hil_local_pos.z = _hil_local_alt0 - hil_state.alt / 1000.0f;
		hil_local_pos.vx = hil_state.vx / 100.0f;
		hil_local_pos.vy = hil_state.vy / 100.0f;
		hil_local_pos.vz = hil_state.vz / 100.0f;

		matrix::Eulerf euler{matrix::Quatf(hil_state.attitude_quaternion)};
		hil_local_pos.yaw = euler.psi();
		hil_local_pos.xy_global = true;
		hil_local_pos.z_global = true;
		hil_local_pos.vxy_max = INFINITY;
		hil_local_pos.vz_max = INFINITY;
		hil_local_pos.hagl_min = INFINITY;
		hil_local_pos.hagl_max = INFINITY;

		_local_pos_pub.publish(hil_local_pos);
	}

	/* accelerometer */
	{
		sensor_accel_s accel{};

		accel.timestamp = timestamp;
		accel.x_raw = hil_state.xacc / CONSTANTS_ONE_G * 1e3f;
		accel.y_raw = hil_state.yacc / CONSTANTS_ONE_G * 1e3f;
		accel.z_raw = hil_state.zacc / CONSTANTS_ONE_G * 1e3f;
		accel.x = hil_state.xacc;
		accel.y = hil_state.yacc;
		accel.z = hil_state.zacc;
		accel.temperature = 25.0f;

		_accel_pub.publish(accel);
	}

	/* gyroscope */
	{
		sensor_gyro_s gyro{};

		gyro.timestamp = timestamp;
		gyro.x_raw = hil_state.rollspeed * 1e3f;
		gyro.y_raw = hil_state.pitchspeed * 1e3f;
		gyro.z_raw = hil_state.yawspeed * 1e3f;
		gyro.x = hil_state.rollspeed;
		gyro.y = hil_state.pitchspeed;
		gyro.z = hil_state.yawspeed;
		gyro.temperature = 25.0f;

		_gyro_pub.publish(gyro);
	}
}
