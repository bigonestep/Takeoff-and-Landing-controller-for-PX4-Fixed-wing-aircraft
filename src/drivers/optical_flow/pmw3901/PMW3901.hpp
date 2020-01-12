/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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
 * @file PMW3901.hpp
 * @author Daniele Pettenuzzo
 *
 * Driver for the Pixart PMW3901 optical flow sensor connected via SPI.
 */

#pragma once

#include "PixArt_PMW3901MB_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/spi.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/optical_flow.h>

using namespace PixArt_PMW3901MB;

class PMW3901 : public device::SPI, public px4::ScheduledWorkItem
{
public:
	PMW3901(int bus, uint32_t device, float yaw_rotation_degrees = NAN);
	~PMW3901() override;

	int init() override;
	void print_info();

private:

	int probe() override;
	void Run() override;

	uint8_t RegisterRead(uint8_t reg);
	void    RegisterWrite(uint8_t reg, uint8_t data);

	bool Reset();
	void SetMode();

	uORB::PublicationMulti<optical_flow_s> _optical_flow_pub{ORB_ID(optical_flow)};

	matrix::Dcmf _rotation;

	uint64_t _interval{SAMPLE_INTERVAL};

	int _motion_count{0};

	int16_t _x_raw_prev{0};
	int16_t _y_raw_prev{0};

	perf_counter_t _sample_perf;
	perf_counter_t _no_motion_perf;
	perf_counter_t _bad_data_perf;
	perf_counter_t _duplicate_data_perf;
};
