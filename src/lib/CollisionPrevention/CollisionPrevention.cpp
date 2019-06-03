/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file CollisionPrevention.cpp
 * CollisionPrevention controller.
 *
 */

#include <CollisionPrevention/CollisionPrevention.hpp>
using namespace matrix;
using namespace time_literals;


CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{
	memset(&_obstacle_map.distances[0], UINT16_MAX, sizeof(_obstacle_map.distances));
}

CollisionPrevention::~CollisionPrevention()
{
	//unadvertise publishers
	if (_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
	}
}

bool CollisionPrevention::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!subscription_array.get(ORB_ID(obstacle_distance), _sub_obstacle_distance)) {
		return false;
	}

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		if (!subscription_array.get(ORB_ID(distance_sensor), _sub_distance_sensor[i], i)) {
			return false;
		}
	}

	if (!subscription_array.get(ORB_ID(vehicle_attitude), _sub_attitude)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(vehicle_local_position), _sub_local_position)) {
		return false;
	}

	return true;
}

void CollisionPrevention::publishConstrainedSetpoint(const Vector2f &original_setpoint,
		const Vector2f &adapted_setpoint)
{

	collision_constraints_s	constraints;

	// fill in values
	constraints.timestamp = hrt_absolute_time();

	constraints.original_setpoint[0] = original_setpoint(0);
	constraints.original_setpoint[1] = original_setpoint(1);
	constraints.adapted_setpoint[0] = adapted_setpoint(0);
	constraints.adapted_setpoint[1] = adapted_setpoint(1);

	// publish constraints
	int instance_id;
	orb_publish_auto(ORB_ID(collision_constraints), &_constraints_pub, &constraints, &instance_id, ORB_PRIO_DEFAULT);

}

void CollisionPrevention::publishObstacleMap()
{
	// publish map
	int instance_id;
	orb_publish_auto(ORB_ID(obstacle_distance), &_map_pub, &_obstacle_map, &instance_id, ORB_PRIO_DEFAULT);
}

void CollisionPrevention::updateMapOffboard()
{
	const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance->get();

	// Update the map with offboard data if the data is not stale
	if (hrt_elapsed_time(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US
	    /*obstacle_distance.timestamp > _obstacle_map.timestamp*/) {
		_obstacle_map = obstacle_distance;
	}
}

void CollisionPrevention::updateMapDistanceSensor()
{
	distance_sensor_s distance_sensor = _sub_distance_sensor[0]->get();

	if (distance_sensor.max_distance > 5.0f) {
		distance_sensor = _sub_distance_sensor[1]->get();
	}

	const vehicle_local_position_s &local_position = _sub_local_position->get();

	const float hfov = 30; // degrees
	const float vfov = 30; // degrees

	// Update the map with rangefinder data if the data is not stale and newer
	if (hrt_elapsed_time(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US &&
	    distance_sensor.timestamp > _obstacle_map.timestamp) {

		_obstacle_map.timestamp = distance_sensor.timestamp;

		// Check sensor minimum distance
		if (distance_sensor.current_distance > distance_sensor.min_distance) {

			if (_obstacle_map.increment > 0) {
				// Map already has data
				_obstacle_map.max_distance = math::max(int(_obstacle_map.max_distance),
								       int(distance_sensor.max_distance * 100.0f));
				_obstacle_map.min_distance = math::min(int(_obstacle_map.min_distance),
								       int(distance_sensor.min_distance * 100.0f)); // TODO : static cast? rounding?

			} else {
				_obstacle_map.max_distance = distance_sensor.max_distance * 100.0f; // convert to cm
				_obstacle_map.min_distance = distance_sensor.min_distance * 100.0f; // convert to cm
				_obstacle_map.increment = hfov; // TODO : check rounding to ceil this when getting from message
			}

			// Calculate yaw offset of sensor in body frame
			float sensor_yaw_body = sensorOrientationToYawOffset(distance_sensor);

			// Convert the offset from body to local frame
			matrix::Quatf attitude = Quatf(_sub_attitude->get().q);
			float sensor_yaw_local = math::degrees(wrap_2pi(Eulerf(attitude).psi() + sensor_yaw_body));

			// Calculate pitch of sensor in local frame. TODO : any better way to do this?
			attitude = Quatf(Eulerf(0.0f, 0.0f, sensor_yaw_body)) * attitude;
			float sensor_pitch_local = Eulerf(attitude).theta();

			//PX4_INFO("sensor pitch : %f", (double)sensor_pitch_local);

			// Calculate maximum distance the sensor can measure without seeing the ground
			if (local_position.dist_bottom_valid) {

				float angle = sensor_pitch_local - math::radians(vfov) / 2.0f;

				float max_distance_compensated = 0.9f * -(local_position.dist_bottom + 0.5f) / sinf(angle);

				distance_sensor.max_distance = math::constrain(max_distance_compensated,
							       distance_sensor.min_distance,
							       distance_sensor.max_distance);

				// PX4_INFO("beam angle : %f, compensated max : %f", double(math::degrees(angle)), double(distance_sensor.max_distance));
			}

			// calculate the field of view boundary bin indices
			const int lower_bound = (int)floor(sensor_yaw_local - (hfov / 2.0f)) /
						float(_obstacle_map.increment);
			const int upper_bound = (int)floor(sensor_yaw_local + (hfov / 2.0f)) /
						float(_obstacle_map.increment); // TODO : why floor

			// Perform map update
			for (int bin = lower_bound; bin <= upper_bound; ++bin) {
				int wrap_bin = bin;

				if (wrap_bin < 0) {
					// wrap bin index around the array
					wrap_bin = (360 / _obstacle_map.increment) + bin;
				}

				// Check sensor maximum distance after ground compensation
				if (distance_sensor.current_distance < distance_sensor.max_distance) {

					// compensate measurement for sensor pitch and convert to cm
					_obstacle_map.distances[wrap_bin] = distance_sensor.current_distance * cosf(sensor_pitch_local) * 100.0f;

				} else {

					// No return from sensor - way is clear
					_obstacle_map.distances[wrap_bin] = UINT16_MAX;
				}

				//PX4_INFO("original %f, compensated %f", (double)distance_sensor.current_distance, (double)(distance_sensor.current_distance * cosf(attitude.theta())));

			}



		}

	}


}

void CollisionPrevention::calculateConstrainedSetpoint(Vector2f &setpoint,
		const Vector2f &curr_pos, const Vector2f &curr_vel)
{
	// Update obstacle map using all sensors
	updateMapOffboard();
	updateMapDistanceSensor();

	//The maximum velocity formula contains a square root, therefore the whole calculation is done with squared norms.
	//that way the root does not have to be calculated for every range bin but once at the end.
	float setpoint_length = setpoint.norm();
	Vector2f setpoint_sqrd = setpoint * setpoint_length;

	//Limit the deviation of the adapted setpoint from the originally given joystick input (slightly less than 90 degrees)
	float max_slide_angle_rad = 0.5f;

	if (hrt_elapsed_time(&_obstacle_map.timestamp) < RANGE_STREAM_TIMEOUT_US) { // TODO : update name
		if (setpoint_length > 0.001f) {

			int distances_array_size = sizeof(_obstacle_map.distances) / sizeof(_obstacle_map.distances[0]);

			for (int i = 0; i < distances_array_size; i++) {

				//determine if distance bin is valid and contains a valid distance measurement
				if (_obstacle_map.distances[i] < _obstacle_map.max_distance &&
				    _obstacle_map.distances[i] > _obstacle_map.min_distance &&
				    i * _obstacle_map.increment < 360) {
					float distance = _obstacle_map.distances[i] / 100.0f; //convert to meters
					float angle = math::radians((float)i * _obstacle_map.increment);

					//split current setpoint into parallel and orthogonal components with respect to the current bin direction
					Vector2f bin_direction = {cos(angle), sin(angle)};
					Vector2f orth_direction = {-bin_direction(1), bin_direction(0)};
					float sp_parallel = setpoint_sqrd.dot(bin_direction);
					float sp_orth = setpoint_sqrd.dot(orth_direction);
					float curr_vel_parallel = math::max(0.f, curr_vel.dot(bin_direction));

					//calculate max allowed velocity with a P-controller (same gain as in the position controller)
					float delay_distance = curr_vel_parallel * _param_mpc_col_prev_dly.get();
					float vel_max_posctrl = math::max(0.f,
									  _param_mpc_xy_p.get() * (distance - _param_mpc_col_prev_d.get() - delay_distance));
					float vel_max_sqrd = vel_max_posctrl * vel_max_posctrl;

					//limit the setpoint to respect vel_max by subtracting from the parallel component
					if (sp_parallel > vel_max_sqrd) {
						Vector2f setpoint_temp = setpoint_sqrd - (sp_parallel - vel_max_sqrd) * bin_direction;
						float setpoint_temp_length = setpoint_temp.norm();

						//limit sliding angle
						float angle_diff_temp_orig = acos(setpoint_temp.dot(setpoint) / (setpoint_temp_length * setpoint_length));
						float angle_diff_temp_bin = acos(setpoint_temp.dot(bin_direction) / setpoint_temp_length);

						if (angle_diff_temp_orig > max_slide_angle_rad && setpoint_temp_length > 0.001f) {
							float angle_temp_bin_cropped = angle_diff_temp_bin - (angle_diff_temp_orig - max_slide_angle_rad);
							float orth_len = vel_max_sqrd * tan(angle_temp_bin_cropped);

							if (sp_orth > 0) {
								setpoint_temp = vel_max_sqrd * bin_direction + orth_len * orth_direction;

							} else {
								setpoint_temp = vel_max_sqrd * bin_direction - orth_len * orth_direction;
							}
						}

						setpoint_sqrd = setpoint_temp;
					}
				}
			}

			//take the squared root
			if (setpoint_sqrd.norm() > 0.001f) {
				setpoint = setpoint_sqrd / std::sqrt(setpoint_sqrd.norm());

			} else {
				setpoint.zero();
			}
		}

	} else if (1/*_last_message + MESSAGE_THROTTLE_US < hrt_absolute_time()*/) {
		//mavlink_log_critical(&_mavlink_log_pub, "No range data received");
		_last_message = hrt_absolute_time();
	}
}

void CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed,
		const Vector2f &curr_pos, const Vector2f &curr_vel)
{
	//calculate movement constraints based on range data
	Vector2f new_setpoint = original_setpoint;
	calculateConstrainedSetpoint(new_setpoint, curr_pos, curr_vel);

	//warn user if collision prevention starts to interfere
	bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * max_speed
				      || new_setpoint(0) > original_setpoint(0) + 0.05f * max_speed
				      || new_setpoint(1) < original_setpoint(1) - 0.05f * max_speed
				      || new_setpoint(1) > original_setpoint(1) + 0.05f * max_speed);

	if (currently_interfering && (currently_interfering != _interfering)) {
		mavlink_log_critical(&_mavlink_log_pub, "Collision Warning");
	}

	_interfering = currently_interfering;
	publishConstrainedSetpoint(original_setpoint, new_setpoint);
	publishObstacleMap();
	original_setpoint = new_setpoint;
}
