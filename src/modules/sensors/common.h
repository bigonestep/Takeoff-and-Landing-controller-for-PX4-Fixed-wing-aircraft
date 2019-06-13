/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/ecl/validation/data_validator.h>
#include <lib/ecl/validation/data_validator_group.h>
/**
 * @file common.h
 * common definitions used in sensors module
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

namespace sensors
{

constexpr uint8_t MAG_COUNT_MAX = 4;
constexpr uint8_t GYRO_COUNT_MAX = 3;
constexpr uint8_t ACCEL_COUNT_MAX = 3;
constexpr uint8_t BARO_COUNT_MAX = 3;

constexpr uint8_t SENSOR_COUNT_MAX = math::max(MAG_COUNT_MAX,
				     math::max(GYRO_COUNT_MAX,
						     math::max(ACCEL_COUNT_MAX,
								     BARO_COUNT_MAX)));

struct SensorData {
	SensorData()
		: last_best_vote(0),
		  subscription_count(0),
		  voter(1),
		  last_failover_count(0)
	{
		for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
			enabled[i] = true;
			subscription[i] = -1;
			priority[i] = 0;
		}
	}

	bool enabled[SENSOR_COUNT_MAX];

	int subscription[SENSOR_COUNT_MAX]; /**< raw sensor data subscription */
	uint8_t priority[SENSOR_COUNT_MAX]; /**< sensor priority */
	uint8_t last_best_vote; /**< index of the latest best vote */
	int subscription_count;
	DataValidatorGroup voter;
	unsigned int last_failover_count;
};
} /* namespace sensors */
