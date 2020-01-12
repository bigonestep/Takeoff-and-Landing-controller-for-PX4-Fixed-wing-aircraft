/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "PAW3902.hpp"

#include <lib/conversion/rotation.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>

using namespace time_literals;
using namespace PixArt_PAW3902JF;

static constexpr int16_t combine(uint8_t lsb, uint8_t msb) { return (msb << 8u) | lsb; }

PAW3902::PAW3902(int bus, uint32_t device, float yaw_rotation_degrees) :
	SPI("PAW3902", nullptr, bus, device, SPIDEV_MODE0, SPI_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_no_motion_perf(perf_alloc(PC_COUNT, MODULE_NAME": no motion")),
	_bad_data_perf(perf_alloc(PC_COUNT, MODULE_NAME": bad data")),
	_duplicate_data_perf(perf_alloc(PC_COUNT, MODULE_NAME": duplicate data"))
{
	if (PX4_ISFINITE(yaw_rotation_degrees)) {
		_rotation = matrix::Dcmf{matrix::Eulerf{0.0f, 0.0f, math::radians(yaw_rotation_degrees)}};

	} else {
		// otherwise use the parameter SENS_FLOW_ROT
		param_t rot = param_find("SENS_FLOW_ROT");
		int32_t val = 0;

		if (param_get(rot, &val) == PX4_OK) {
			_rotation = get_rot_matrix((enum Rotation)val);

		} else {
			_rotation.identity();
		}
	}
}

PAW3902::~PAW3902()
{
	ScheduleClear();

	perf_free(_sample_perf);
	perf_free(_no_motion_perf);
	perf_free(_bad_data_perf);
	perf_free(_duplicate_data_perf);
}

int PAW3902::init()
{
	// do SPI init (and probe) first
	if (SPI::init() != OK) {
		return PX4_ERROR;
	}

	// default to low light mode
	SetMode(Mode::LowLight, true);

	return PX4_OK;
}

int PAW3902::probe()
{
	// Product_ID
	const uint8_t product_id = RegisterRead(Register::Product_ID);
	PX4_DEBUG("Product_ID: %X", product_id);

	// Revision_ID
	const uint8_t revision_id = RegisterRead(Register::Revision_ID);
	PX4_DEBUG("Revision_ID: %X", revision_id);

	// Inverse_Product_ID
	const uint8_t inverse_product_id = RegisterRead(Register::Inverse_Product_ID);
	PX4_DEBUG("Inverse_Product_ID: %X", inverse_product_id);

	if ((product_id == PRODUCT_ID) && (revision_id == REVISION_ID) && (inverse_product_id == INVERSE_PRODUCT_ID)) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

bool PAW3902::Reset()
{
	// Power on reset: Write 0x5A to this register to reset the chip.
	RegisterWrite(Register::Power_Up_Reset, 0x5A);
	px4_usleep(1_ms); // Wait for at least 1 ms.

	_motion_count = 0;

	return true;
}

bool PAW3902::SetMode(Mode newMode, bool force)
{
	if (newMode == _mode_change) {
		_mode_change_count++;

	} else {
		_mode_change = newMode;
		_mode_change_count = 0;
	}

	// require 10 consecutive requests
	if (((newMode != _mode) && (_mode_change_count > 9)) || force) {

		// Procedure to switch operation mode:
		// 1. Issue a soft reset to PAW3902JF by writing 0x5A to Power_Up_Reset register.
		Reset();

		ScheduleClear();
		PX4_DEBUG("changing from mode %d -> %d", static_cast<int>(_mode), static_cast<int>(newMode));

		// 2. Refer Section 8.2 Performance Optimization Registers to configure the needed registers in order to achieve optimum
		//    performance of the chip for the desired operation mode.
		switch (newMode) {
		case Mode::Bright:
			SetModeBright();
			break;

		case Mode::LowLight:
			SetModeLowLight();
			break;

		case Mode::SuperLowLight:
			SetModeSuperLowLight();
			break;
		}

		// 3. Discard the first three motion data.
		// handled in Run
		_motion_count = 0;

		// 4. Resume normal operation of reading motion data.
		_mode = newMode;
		_mode_change = newMode;
		_mode_change_count = 0;

		// Enable LED_N controls
		RegisterWrite(0x7F, 0x14);
		RegisterWrite(0x6F, 0x1c);
		RegisterWrite(0x7F, 0x00);

		ScheduleOnInterval(_interval / 2, _interval);

		return true;
	}

	return false;
}

void PAW3902::SetModeBright()
{
	// Mode 0: Bright (126 fps) 60 Lux typical
	_interval = static_cast<uint64_t>(Interval::Bright);

	// set performance optimization registers
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x55, 0x01);
	RegisterWrite(0x50, 0x07);
	RegisterWrite(0x7f, 0x0e);
	RegisterWrite(0x43, 0x10);

	RegisterWrite(0x48, 0x02);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x51, 0x7b);
	RegisterWrite(0x50, 0x00);
	RegisterWrite(0x55, 0x00);

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x61, 0xAD);
	RegisterWrite(0x7F, 0x03);
	RegisterWrite(0x40, 0x00);
	RegisterWrite(0x7F, 0x05);
	RegisterWrite(0x41, 0xB3);
	RegisterWrite(0x43, 0xF1);
	RegisterWrite(0x45, 0x14);
	RegisterWrite(0x5F, 0x34);
	RegisterWrite(0x7B, 0x08);
	RegisterWrite(0x5e, 0x34);
	RegisterWrite(0x5b, 0x32);
	RegisterWrite(0x6d, 0x32);
	RegisterWrite(0x45, 0x17);
	RegisterWrite(0x70, 0xe5);
	RegisterWrite(0x71, 0xe5);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x44, 0x1B);
	RegisterWrite(0x40, 0xBF);
	RegisterWrite(0x4E, 0x3F);
	RegisterWrite(0x7F, 0x08);
	RegisterWrite(0x66, 0x44);
	RegisterWrite(0x65, 0x20);
	RegisterWrite(0x6a, 0x3a);
	RegisterWrite(0x61, 0x05);
	RegisterWrite(0x62, 0x05);
	RegisterWrite(0x7F, 0x09);
	RegisterWrite(0x4F, 0xAF);
	RegisterWrite(0x48, 0x80);
	RegisterWrite(0x49, 0x80);
	RegisterWrite(0x57, 0x77);
	RegisterWrite(0x5F, 0x40);
	RegisterWrite(0x60, 0x78);
	RegisterWrite(0x61, 0x78);
	RegisterWrite(0x62, 0x08);
	RegisterWrite(0x63, 0x50);
	RegisterWrite(0x7F, 0x0A);
	RegisterWrite(0x45, 0x60);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x4D, 0x11);
	RegisterWrite(0x55, 0x80);
	RegisterWrite(0x74, 0x21);
	RegisterWrite(0x75, 0x1F);
	RegisterWrite(0x4A, 0x78);
	RegisterWrite(0x4B, 0x78);
	RegisterWrite(0x44, 0x08);
	RegisterWrite(0x45, 0x50);
	RegisterWrite(0x64, 0xFE);
	RegisterWrite(0x65, 0x1F);
	RegisterWrite(0x72, 0x0A);
	RegisterWrite(0x73, 0x00);
	RegisterWrite(0x7F, 0x14);
	RegisterWrite(0x44, 0x84);
	RegisterWrite(0x65, 0x47);
	RegisterWrite(0x66, 0x18);
	RegisterWrite(0x63, 0x70);
	RegisterWrite(0x6f, 0x2c);
	RegisterWrite(0x7F, 0x15);
	RegisterWrite(0x48, 0x48);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x41, 0x0D);
	RegisterWrite(0x43, 0x14);
	RegisterWrite(0x4B, 0x0E);
	RegisterWrite(0x45, 0x0F);
	RegisterWrite(0x44, 0x42);
	RegisterWrite(0x4C, 0x80);
	RegisterWrite(0x7F, 0x10);
	RegisterWrite(0x5B, 0x03);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x41);

	px4_usleep(10_ms); // Delay 10 ms

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x32, 0x00);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x40);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x68, 0x70);
	RegisterWrite(0x69, 0x01);
	RegisterWrite(0x7F, 0x0D);
	RegisterWrite(0x48, 0xC0);
	RegisterWrite(0x6F, 0xD5);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x5B, 0xA0);
	RegisterWrite(0x4E, 0xA8);
	RegisterWrite(0x5A, 0x50);
	RegisterWrite(0x40, 0x80);
	RegisterWrite(0x73, 0x1f);

	px4_usleep(10_ms); // Delay 10 ms

	RegisterWrite(0x73, 0x00);
}

void PAW3902::SetModeLowLight()
{
	// Mode 1: Low Light (126 fps) 30 Lux typical
	// low light and low speed motion tracking
	_interval = static_cast<uint64_t>(Interval::LowLight);

	// set performance optimization registers
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x55, 0x01);
	RegisterWrite(0x50, 0x07);
	RegisterWrite(0x7f, 0x0e);
	RegisterWrite(0x43, 0x10);

	RegisterWrite(0x48, 0x02);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x51, 0x7b);
	RegisterWrite(0x50, 0x00);
	RegisterWrite(0x55, 0x00);

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x61, 0xAD);
	RegisterWrite(0x7F, 0x03);
	RegisterWrite(0x40, 0x00);
	RegisterWrite(0x7F, 0x05);
	RegisterWrite(0x41, 0xB3);
	RegisterWrite(0x43, 0xF1);
	RegisterWrite(0x45, 0x14);
	RegisterWrite(0x5F, 0x34);
	RegisterWrite(0x7B, 0x08);
	RegisterWrite(0x5e, 0x34);
	RegisterWrite(0x5b, 0x65);
	RegisterWrite(0x6d, 0x65);
	RegisterWrite(0x45, 0x17);
	RegisterWrite(0x70, 0xe5);
	RegisterWrite(0x71, 0xe5);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x44, 0x1B);
	RegisterWrite(0x40, 0xBF);
	RegisterWrite(0x4E, 0x3F);
	RegisterWrite(0x7F, 0x08);
	RegisterWrite(0x66, 0x44);
	RegisterWrite(0x65, 0x20);
	RegisterWrite(0x6a, 0x3a);
	RegisterWrite(0x61, 0x05);
	RegisterWrite(0x62, 0x05);
	RegisterWrite(0x7F, 0x09);
	RegisterWrite(0x4F, 0xAF);
	RegisterWrite(0x48, 0x80);
	RegisterWrite(0x49, 0x80);
	RegisterWrite(0x57, 0x77);
	RegisterWrite(0x5F, 0x40);
	RegisterWrite(0x60, 0x78);
	RegisterWrite(0x61, 0x78);
	RegisterWrite(0x62, 0x08);
	RegisterWrite(0x63, 0x50);
	RegisterWrite(0x7F, 0x0A);
	RegisterWrite(0x45, 0x60);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x4D, 0x11);
	RegisterWrite(0x55, 0x80);
	RegisterWrite(0x74, 0x21);
	RegisterWrite(0x75, 0x1F);
	RegisterWrite(0x4A, 0x78);
	RegisterWrite(0x4B, 0x78);
	RegisterWrite(0x44, 0x08);
	RegisterWrite(0x45, 0x50);
	RegisterWrite(0x64, 0xFE);
	RegisterWrite(0x65, 0x1F);
	RegisterWrite(0x72, 0x0A);
	RegisterWrite(0x73, 0x00);
	RegisterWrite(0x7F, 0x14);
	RegisterWrite(0x44, 0x84);
	RegisterWrite(0x65, 0x67);
	RegisterWrite(0x66, 0x18);
	RegisterWrite(0x63, 0x70);
	RegisterWrite(0x6f, 0x2c);
	RegisterWrite(0x7F, 0x15);
	RegisterWrite(0x48, 0x48);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x41, 0x0D);
	RegisterWrite(0x43, 0x14);
	RegisterWrite(0x4B, 0x0E);
	RegisterWrite(0x45, 0x0F);
	RegisterWrite(0x44, 0x42);
	RegisterWrite(0x4C, 0x80);
	RegisterWrite(0x7F, 0x10);
	RegisterWrite(0x5B, 0x03);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x41);

	px4_usleep(10_ms); // Delay 10 ms

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x32, 0x00);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x40);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x68, 0x70);
	RegisterWrite(0x69, 0x01);
	RegisterWrite(0x7F, 0x0D);
	RegisterWrite(0x48, 0xC0);
	RegisterWrite(0x6F, 0xD5);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x5B, 0xA0);
	RegisterWrite(0x4E, 0xA8);
	RegisterWrite(0x5A, 0x50);
	RegisterWrite(0x40, 0x80);
	RegisterWrite(0x73, 0x1f);

	px4_usleep(10_ms); // Delay 10 ms

	RegisterWrite(0x73, 0x00);
}

void PAW3902::SetModeSuperLowLight()
{
	// Mode 2: Super Low Light (50 fps) 9 Lux typical
	// super low light and low speed motion tracking
	_interval = static_cast<uint64_t>(Interval::SuperLowLight);

	// set performance optimization registers
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x55, 0x01);
	RegisterWrite(0x50, 0x07);
	RegisterWrite(0x7f, 0x0e);
	RegisterWrite(0x43, 0x10);

	RegisterWrite(0x48, 0x04);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x51, 0x7b);
	RegisterWrite(0x50, 0x00);
	RegisterWrite(0x55, 0x00);

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x61, 0xAD);
	RegisterWrite(0x7F, 0x03);
	RegisterWrite(0x40, 0x00);
	RegisterWrite(0x7F, 0x05);
	RegisterWrite(0x41, 0xB3);
	RegisterWrite(0x43, 0xF1);
	RegisterWrite(0x45, 0x14);
	RegisterWrite(0x5F, 0x34);
	RegisterWrite(0x7B, 0x08);
	RegisterWrite(0x5E, 0x34);
	RegisterWrite(0x5B, 0x32);
	RegisterWrite(0x6D, 0x32);
	RegisterWrite(0x45, 0x17);
	RegisterWrite(0x70, 0xE5);
	RegisterWrite(0x71, 0xE5);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x44, 0x1B);
	RegisterWrite(0x40, 0xBF);
	RegisterWrite(0x4E, 0x3F);
	RegisterWrite(0x7F, 0x08);
	RegisterWrite(0x66, 0x44);
	RegisterWrite(0x65, 0x20);
	RegisterWrite(0x6A, 0x3a);
	RegisterWrite(0x61, 0x05);
	RegisterWrite(0x62, 0x05);
	RegisterWrite(0x7F, 0x09);
	RegisterWrite(0x4F, 0xAF);
	RegisterWrite(0x48, 0x80);
	RegisterWrite(0x49, 0x80);
	RegisterWrite(0x57, 0x77);
	RegisterWrite(0x5F, 0x40);
	RegisterWrite(0x60, 0x78);
	RegisterWrite(0x61, 0x78);
	RegisterWrite(0x62, 0x08);
	RegisterWrite(0x63, 0x50);
	RegisterWrite(0x7F, 0x0A);
	RegisterWrite(0x45, 0x60);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x4D, 0x11);
	RegisterWrite(0x55, 0x80);
	RegisterWrite(0x74, 0x21);
	RegisterWrite(0x75, 0x1F);
	RegisterWrite(0x4A, 0x78);
	RegisterWrite(0x4B, 0x78);
	RegisterWrite(0x44, 0x08);
	RegisterWrite(0x45, 0x50);
	RegisterWrite(0x64, 0xCE);
	RegisterWrite(0x65, 0x0B);
	RegisterWrite(0x72, 0x0A);
	RegisterWrite(0x73, 0x00);
	RegisterWrite(0x7F, 0x14);
	RegisterWrite(0x44, 0x84);
	RegisterWrite(0x65, 0x67);
	RegisterWrite(0x66, 0x18);
	RegisterWrite(0x63, 0x70);
	RegisterWrite(0x6f, 0x2c);
	RegisterWrite(0x7F, 0x15);
	RegisterWrite(0x48, 0x48);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x41, 0x0D);
	RegisterWrite(0x43, 0x14);
	RegisterWrite(0x4B, 0x0E);
	RegisterWrite(0x45, 0x0F);
	RegisterWrite(0x44, 0x42);
	RegisterWrite(0x4C, 0x80);
	RegisterWrite(0x7F, 0x10);
	RegisterWrite(0x5B, 0x02);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x41);

	px4_usleep(25_ms); // Delay 25ms

	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x32, 0x44);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x40);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x68, 0x40);
	RegisterWrite(0x69, 0x02);
	RegisterWrite(0x7F, 0x0D);
	RegisterWrite(0x48, 0xC0);
	RegisterWrite(0x6F, 0xD5);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x5B, 0xA0);
	RegisterWrite(0x4E, 0xA8);
	RegisterWrite(0x5A, 0x50);
	RegisterWrite(0x40, 0x80);
	RegisterWrite(0x73, 0x0B);

	px4_usleep(25_ms); // Delay 25ms

	RegisterWrite(0x73, 0x00);
}

uint8_t PAW3902::RegisterRead(uint8_t reg)
{
	uint8_t cmd[2] { reg, 0 };
	transfer(&cmd[0], &cmd[0], sizeof(cmd));
	return cmd[1];
}

void PAW3902::RegisterWrite(uint8_t reg, uint8_t data)
{
	uint8_t cmd[2];
	cmd[0] = reg | DIR_WRITE;
	cmd[1] = data;
	transfer(&cmd[0], nullptr, sizeof(cmd));
}

void PAW3902::Run()
{
	perf_begin(_sample_perf);

	// Reading the Motion_Burst register activates Burst Mode.
	// PAW3902JF will respond with the following motion burst report in order
	struct TransferBuffer {
		uint8_t cmd = Register::Motion_Burst;
		uint8_t Motion;
		uint8_t Observation;
		uint8_t Delta_X_L;
		uint8_t Delta_X_H;
		uint8_t Delta_Y_L;
		uint8_t Delta_Y_H;
		uint8_t SQUAL;
		uint8_t RawData_Sum;
		uint8_t Maximum_RawData;
		uint8_t Minimum_RawData;
		uint8_t Shutter_Upper;
		uint8_t Shutter_Lower;
	} buf{};

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (transfer((uint8_t *)&buf, (uint8_t *)&buf, sizeof(buf)) != PX4_OK) {
		perf_end(_sample_perf);
		return;
	}

	// Check if motion occurred and data ready for reading in Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H registers.
	if (buf.Motion & Bit7) {
		_motion_count++;

	} else {
		perf_count(_no_motion_perf);
		perf_end(_sample_perf);
		return;
	}

	// Discard the first three motion data after mode change
	if (_motion_count < 3) {
		perf_end(_sample_perf);
		return;
	}

	// check SQUAL & Shutter values
	const uint16_t shutter = combine(buf.Shutter_Upper & 0x1F, buf.Shutter_Lower);

	switch (_mode) {
	case Mode::Bright:

		// To suppress false motion reports, discard Delta X and Delta Y values if the SQUAL and Shutter values meet the condition
		// Bright Mode,			SQUAL < 0x19, Shutter ≥ 0x1FF0
		if ((buf.SQUAL < 0x19) && (shutter >= 0x1FF0)) {
			PX4_DEBUG("false motion report, discarding");
			perf_count(_bad_data_perf);
			perf_end(_sample_perf);
			return;
		}

		if ((shutter >= 0x1FFE) && (buf.RawData_Sum < 0x3C)) {
			// Bright -> LowLight
			if (SetMode(Mode::LowLight)) {
				perf_end(_sample_perf);
				return;
			}

		} else {
			_mode_change_count = 0;
		}

		break;

	case Mode::LowLight:

		// To suppress false motion reports, discard Delta X and Delta Y values if the SQUAL and Shutter values meet the condition
		// Low Light Mode,		SQUAL < 0x46, Shutter ≥ 0x1FF0
		if ((buf.SQUAL < 0x46) && (shutter >= 0x1FF0)) {
			PX4_DEBUG("false motion report, discarding");
			perf_count(_bad_data_perf);
			perf_end(_sample_perf);
			return;
		}

		if ((shutter >= 0x1FFE) && (buf.RawData_Sum < 0x5A)) {
			// LowLight -> SuperLowLight
			if (SetMode(Mode::SuperLowLight)) {
				perf_end(_sample_perf);
				return;
			}

		} else if ((shutter >= 0x0BB8)) {
			// LowLight -> Bright
			if (SetMode(Mode::Bright)) {
				perf_end(_sample_perf);
				return;
			}

		} else {
			_mode_change_count = 0;
		}

		break;

	case Mode::SuperLowLight:

		// To suppress false motion reports, discard Delta X and Delta Y values if the SQUAL and Shutter values meet the condition
		// Super Low Light Mode,	SQUAL < 0x55, Shutter ≥ 0x0BC0
		if ((buf.SQUAL < 0x55) && (shutter >= 0x0BC0)) {
			PX4_DEBUG("false motion report, discarding");
			perf_count(_bad_data_perf);
			perf_end(_sample_perf);
			return;
		}

		if ((shutter >= 0x03E8)) {
			// SuperLowLight -> LowLight
			if (SetMode(Mode::LowLight)) {
				perf_end(_sample_perf);
				return;
			}

		} else if (shutter >= 0x01F4) {
			// PAW3902JF should not operate with Shutter < 0x01F4 in Mode 2
			if (SetMode(Mode::LowLight)) {
				perf_end(_sample_perf);
				return;
			}

		} else {
			_mode_change_count = 0;
		}

		break;
	}

	const int16_t x_raw = combine(buf.Delta_X_H, buf.Delta_X_L);
	const int16_t y_raw = combine(buf.Delta_Y_H, buf.Delta_Y_L);

	if ((x_raw == _x_raw_prev) && (y_raw == _y_raw_prev)) {
		perf_count(_duplicate_data_perf);
		perf_end(_sample_perf);
		return;
	}

	_x_raw_prev = x_raw;
	_y_raw_prev = y_raw;

	// If the reported flow is impossibly large, we just got garbage from the SPI
	if (x_raw > 240 || y_raw > 240 || x_raw < -240 || y_raw < -240) {
		PX4_DEBUG("garbage data, discarding");
		perf_count(_bad_data_perf);
		perf_end(_sample_perf);
		return;
	}

	optical_flow_s report{};
	report.timestamp = timestamp_sample;

	float pixel_flow_x_integral = (float)x_raw / 500.0f; // proportional factor + convert from pixels to radians
	float pixel_flow_y_integral = (float)y_raw / 500.0f; // proportional factor + convert from pixels to radians
	const matrix::Vector3f pixel_flow_rotated = _rotation * matrix::Vector3f{pixel_flow_x_integral, pixel_flow_y_integral, 0.0f};
	report.pixel_flow_x_integral = pixel_flow_rotated(0);
	report.pixel_flow_y_integral = pixel_flow_rotated(1);

	report.frame_count_since_last_readout = 1;
	report.integration_timespan = _interval;

	//report.sensor_id = 0; // TODO: device id
	report.quality = buf.SQUAL;

	// No gyro on this board
	report.gyro_x_rate_integral = NAN;
	report.gyro_y_rate_integral = NAN;
	report.gyro_z_rate_integral = NAN;

	// set specs according to datasheet
	report.max_flow_rate = 7.4f;       // Datasheet: 7.4 rad/s
	report.min_ground_distance = 0.08f; // Datasheet: 80mm
	report.max_ground_distance = 30.0f; // Datasheet: infinity

	_optical_flow_pub.publish(report);

	perf_end(_sample_perf);
}

void PAW3902::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_no_motion_perf);
	perf_print_counter(_bad_data_perf);
	perf_print_counter(_duplicate_data_perf);

	PX4_INFO("Mode: %d", (int)_mode);
}
