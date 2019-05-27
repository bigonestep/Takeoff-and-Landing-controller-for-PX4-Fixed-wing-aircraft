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
 * @file PositionLock.hpp
 *
 * Library for position and yaw lock logic when flying by stick.
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <matrix/matrix/math.hpp>

class PositionLock
{
public:
	PositionLock() = default;
	~PositionLock() = default;

	/**
	 * Lock yaw when not currently turning
	 * When applying a yawspeed the vehicle is turning, when the speed is
	 * set to zero the vehicle needs to slow down and then lock at the yaw
	 * it stops at to not drift over time.
	 * @param yawspeed current yaw rotational rate state
	 * @param yaw current yaw rotational rate state
	 * @param yawspeed_setpoint rotation rate at which to turn around yaw axis
	 * @param yaw current yaw setpoint which then will be overwritten by the return value
	 * @return yaw setpoint to execute to have a yaw lock at the correct moment in time
	 */
	static float updateYawLock(const float yawspeed, const float yaw, const float yawspeed_setpoint, const float yaw_setpoint) {
		// Yaw-lock depends on desired yawspeed input. If not locked, yaw_sp is set to NAN.
		if (fabsf(yawspeed_setpoint) > FLT_EPSILON) {
			// no fixed heading when rotating around yaw by stick
			return NAN;

		} else {
			// break down and hold the current heading when no more rotation commanded
			if (!PX4_ISFINITE(yaw_setpoint) && fabsf(yawspeed) < 0.01f) {
				return yaw;

			} else {
				return yaw_setpoint;
			}
		}
	}

	/**
	 * Update the yaw state in case an estimation reset happened
	 * @param yaw_reset_counter counter value of the last heading reference reset
	 * @param delta_q_reset rotation offset from the last reset that happened
	 * @return updated yaw setpoint to use with reference reset taken into account
	 */
	float updateYawReset(const float yaw_setpoint, const uint8_t yaw_reset_counter, const matrix::Quatf delta_q_reset) {
		// check if reset counter changed and update yaw setpoint if necessary
		if (yaw_reset_counter != _yaw_reset_counter) {
			_yaw_reset_counter = yaw_reset_counter;
			return yaw_setpoint + matrix::Eulerf(delta_q_reset).psi();
		}
		return yaw_setpoint;
	}

	void setYawResetCounter(const uint8_t yaw_reset_counter) { _yaw_reset_counter = yaw_reset_counter; }

private:
	uint8_t _yaw_reset_counter = 0;

};
