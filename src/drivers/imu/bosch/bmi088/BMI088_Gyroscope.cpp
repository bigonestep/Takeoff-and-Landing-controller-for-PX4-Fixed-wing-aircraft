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

#include "BMI088_Gyroscope.hpp"

#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;

namespace Bosch_BMI088_Gyroscope
{

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

BMI088_Gyroscope::BMI088_Gyroscope(int bus, uint32_t device, enum Rotation rotation) :
	SPI("BMI088_GYRO", nullptr, bus, device, SPIDEV_MODE3, SPI_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_gyro(get_device_id(), ORB_PRIO_DEFAULT, rotation)
{
	set_device_type(DRV_DEVTYPE_BMI088);

	_px4_gyro.set_device_type(DRV_DEVTYPE_BMI088);

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

BMI088_Gyroscope::~BMI088_Gyroscope()
{
	Stop();

	if (_dma_data_buffer != nullptr) {
		board_dma_free(_dma_data_buffer, FIFO::SIZE);
	}

	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_interval_perf);
}

bool BMI088_Gyroscope::Init()
{
	if (SPI::init() != PX4_OK) {
		PX4_ERR("SPI::init failed");
		return false;
	}

	// allocate DMA capable buffer
	_dma_data_buffer = (uint8_t *)board_dma_alloc(FIFO::SIZE);

	if (_dma_data_buffer == nullptr) {
		PX4_ERR("DMA alloc failed");
		return false;
	}

	return Reset();
}

void BMI088_Gyroscope::Stop()
{
	// wait until stopped
	while (_state.load() != STATE::STOPPED) {
		_state.store(STATE::REQUEST_STOP);
		ScheduleNow();
		px4_usleep(10);
	}
}

bool BMI088_Gyroscope::Reset()
{
	_state.store(STATE::RESET);
	ScheduleClear();
	ScheduleNow();
	return true;
}

void BMI088_Gyroscope::PrintInfo()
{
	PX4_INFO("FIFO empty interval: %d us (%.3f Hz)", _fifo_empty_interval_us,
		 static_cast<double>(1000000 / _fifo_empty_interval_us));

	PX4_INFO("FIFO gyro samples: %d", _fifo_gyro_samples);

	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_interval_perf);

	_px4_gyro.print_status();
}

int BMI088_Gyroscope::probe()
{
	const uint8_t chipid = RegisterRead(Register::GYRO_CHIP_ID);

	if (chipid != ID) {
		PX4_WARN("unexpected GYRO_CHIP_ID 0x%02x", chipid);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI088_Gyroscope::Run()
{
	switch (_state.load()) {
	case STATE::RESET:
		// GYRO_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor.
		// Following a delay of 30 ms, all configuration settings are overwritten with their reset value.
		RegisterWrite(Register::GYRO_SOFTRESET, 0xB6);
		_reset_timestamp = hrt_absolute_time();
		_state.store(STATE::WAIT_FOR_RESET);
		ScheduleDelayed(30_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		//  Document Number:
		if ((RegisterRead(Register::GYRO_CHIP_ID) == ID)) {

			// if reset succeeded then configure
			_state.store(STATE::CONFIGURE);
			ScheduleNow();

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 10_ms) {
				PX4_ERR("Reset failed, retrying");
				_state.store(STATE::RESET);
				ScheduleDelayed(10_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 1 ms");
				ScheduleDelayed(1_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state.store(STATE::FIFO_READ);

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(10_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

		} else {
			PX4_DEBUG("Configure failed, retrying");
			// try again in 1 ms
			ScheduleDelayed(1_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = 0;
			uint8_t samples = 0;

			if (_data_ready_interrupt_enabled) {
				// re-schedule as watchdog timeout
				ScheduleDelayed(10_ms);

				// timestamp set in data ready interrupt
				samples = _fifo_read_samples.load();
				timestamp_sample = _fifo_watermark_interrupt_timestamp;
			}

			bool failure = false;

			// manually check FIFO count if no samples from DRDY or timestamp looks bogus
			if (!_data_ready_interrupt_enabled || (samples == 0)
			    || (hrt_elapsed_time(&timestamp_sample) > (_fifo_empty_interval_us / 2))) {

				// use the time now roughly corresponding with the last sample we'll pull from the FIFO
				timestamp_sample = hrt_absolute_time();

				const uint8_t fifo_status = RegisterRead(Register::FIFO_STATUS);
				const uint8_t fifo_frame_counter = fifo_status & FIFO_STATUS_BIT::Fifo_frame_counter;

				if (Fifo_overrun & FIFO_STATUS_BIT::Fifo_frame_counter) {
					failure = true;
					samples = 0;
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (fifo_frame_counter == 0) {
					failure = true;
					samples = 0;
					perf_count(_fifo_empty_perf);

				} else {
					samples = fifo_frame_counter;
				}
			}

			if (samples > FIFO_MAX_SAMPLES) {
				// not technically an overflow, but more samples than we expected or can publish
				perf_count(_fifo_overflow_perf);
				failure = true;
				FIFOReset();

			} else if (samples >= 1) {
				if (!FIFORead(timestamp_sample, samples)) {
					failure = true;
					_px4_gyro.increase_error_count();
				}
			}

			if (failure || hrt_elapsed_time(&_last_config_check_timestamp) > 10_ms) {
				// check registers incrementally
				if (RegisterCheck(_register_cfg[_checked_register], true)) {
					_last_config_check_timestamp = timestamp_sample;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reconfigure
					PX4_DEBUG("Health check failed, reconfiguring");
					_state.store(STATE::CONFIGURE);
					ScheduleNow();
				}
			}
		}

		break;

	case STATE::REQUEST_STOP:
		DataReadyInterruptDisable();
		ScheduleClear();
		_state.store(STATE::STOPPED);
		break;

	case STATE::STOPPED:
		// DO NOTHING
		break;
	}
}

void BMI088_Gyroscope::ConfigureGyro()
{
	const uint8_t GYRO_RANGE = RegisterRead(Register::GYRO_RANGE);

	switch (GYRO_RANGE) {
	case gyro_range_2000_dps:
		_px4_gyro.set_scale(math::radians(1.0f / 16.384f));
		_px4_gyro.set_range(math::radians(2000.f));
		break;

	case gyro_range_1000_dps:
		_px4_gyro.set_scale(math::radians(1.0f / 32.768f));
		_px4_gyro.set_range(math::radians(1000.f));
		break;

	case gyro_range_500_dps:
		_px4_gyro.set_scale(math::radians(1.0f / 65.536f));
		_px4_gyro.set_range(math::radians(500.f));
		break;

	case gyro_range_250_dps:
		_px4_gyro.set_scale(math::radians(1.0f / 131.072f));
		_px4_gyro.set_range(math::radians(250.f));
		break;

	case gyro_range_125_dps:
		_px4_gyro.set_scale(math::radians(1.0f / 262.144f));
		_px4_gyro.set_range(math::radians(125.f));
		break;
	}
}

void BMI088_Gyroscope::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = 1000; // default to 1 kHz
	}

	_fifo_empty_interval_us = math::max(((1000000 / sample_rate) / 500) * 500, 500); // round down to nearest 500 us
	PX4_INFO("_fifo_empty_interval_us: %d", _fifo_empty_interval_us);
	_fifo_gyro_samples = math::min(_fifo_empty_interval_us / (1000000 / GYRO_RATE), FIFO_MAX_SAMPLES);
	PX4_INFO("_fifo_gyro_samples: %d", _fifo_gyro_samples);

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1000000 / GYRO_RATE);
	PX4_INFO("_fifo_empty_interval_us: %d", _fifo_empty_interval_us);

	_px4_gyro.set_update_rate(1000000 / _fifo_empty_interval_us);
	_px4_gyro.set_integrator_reset_interval(2500); // 400 Hz (2.5 ms)

	// FIFO watermark threshold in number of bytes
	const uint16_t fifo_watermark_threshold = _fifo_gyro_samples;

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CONFIG_0) {
			r.set_bits = fifo_watermark_threshold;
		}
	}
}

bool BMI088_Gyroscope::Configure()
{
	bool success = true;

	for (const auto &reg : _register_cfg) {
		if (!RegisterCheck(reg)) {
			success = false;
		}
	}

	ConfigureGyro();

	return success;
}

int BMI088_Gyroscope::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BMI088_Gyroscope *>(arg)->DataReady();
	return 0;
}

void BMI088_Gyroscope::DataReady()
{
	_fifo_watermark_interrupt_timestamp = hrt_absolute_time();
	_fifo_read_samples.store(_fifo_gyro_samples);
	ScheduleNow();
	perf_count(_drdy_interval_perf);
}

bool BMI088_Gyroscope::DataReadyInterruptConfigure()
{
	int ret_setevent = -1;

	// Setup data ready on rising edge
	// TODO: cleanup horrible DRDY define mess
	// Setup data ready Interrupt on Falling edge for better noise immunity
#if defined(GPIO_SPI3_DRDY2_BMI088_INT3_GYRO) && false
	ret_setevent = px4_arch_gpiosetevent(GPIO_SPI3_DRDY2_BMI088_INT3_GYRO, false, true, false,
					     &BMI088_Gyroscope::DataReadyInterruptCallback, this);
#elif defined(GPIO_DRDY_BMI088_INT3_GYRO) && false
	ret_setevent = px4_arch_gpiosetevent(GPIO_DRDY_BMI088_INT3_GYRO, false, true, falsee,
					     &BMI088_Gyroscope::DataReadyInterruptCallback, this);
#endif
	return (ret_setevent == 0);
}

bool BMI088_Gyroscope::DataReadyInterruptDisable()
{
	int ret_setevent = -1;

	// Disable data ready callback
	// TODO: cleanup horrible DRDY define mess
#if defined(GPIO_SPI3_DRDY2_BMI088_INT3_GYRO) && false
	ret_setevent = px4_arch_gpiosetevent(GPIO_SPI3_DRDY2_BMI088_INT3_GYRO, false, false, false, nullptr, nullptr);
#elif defined(GPIO_DRDY_BMI088_INT3_GYRO) && false
	ret_setevent = px4_arch_gpiosetevent(GPIO_DRDY_BMI088_INT3_GYRO, false, false, false, nullptr, nullptr);
#endif

	return (ret_setevent == 0);
}

bool BMI088_Gyroscope::RegisterCheck(const register_config_t &reg_cfg, bool notify)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) == 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	if (!success) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

		if (reg_cfg.reg == Register::GYRO_RANGE) {
			ConfigureGyro();
		}

		if (notify) {
			perf_count(_bad_register_perf);
			_px4_gyro.increase_error_count();
		}
	}

	return success;
}

uint8_t BMI088_Gyroscope::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void BMI088_Gyroscope::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void BMI088_Gyroscope::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = orig_val;

	if (setbits) {
		val |= setbits;
	}

	if (clearbits) {
		val &= ~clearbits;
	}

	RegisterWrite(reg, val);
}

void BMI088_Gyroscope::RegisterSetBits(Register reg, uint8_t setbits)
{
	RegisterSetAndClearBits(reg, setbits, 0);
}

void BMI088_Gyroscope::RegisterClearBits(Register reg, uint8_t clearbits)
{
	RegisterSetAndClearBits(reg, 0, clearbits);
}

bool BMI088_Gyroscope::FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples)
{
	TransferBuffer *report = (TransferBuffer *)_dma_data_buffer;
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);
	memset(report, 0, transfer_size);
	report->cmd = static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ;

	perf_begin(_transfer_perf);

	if (transfer(_dma_data_buffer, _dma_data_buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		perf_count(_bad_transfer_perf);
		return false;
	}

	perf_end(_transfer_perf);

	PX4Gyroscope::FIFOSample gyro;
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = _fifo_empty_interval_us / _fifo_gyro_samples;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = report->f[i];

		const int16_t gyro_x = combine(fifo_sample.RATE_X_MSB, fifo_sample.RATE_X_LSB);
		const int16_t gyro_y = combine(fifo_sample.RATE_Y_MSB, fifo_sample.RATE_Y_LSB);
		const int16_t gyro_z = combine(fifo_sample.RATE_Z_MSB, fifo_sample.RATE_Z_LSB);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.updateFIFO(gyro);

	return true;
}

void BMI088_Gyroscope::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO overrun condition can only be cleared by writing to the FIFO configuration register FIFO_CONFIG_1
	RegisterWrite(Register::FIFO_CONFIG_1, FIFO_CONFIG_1_BIT::STREAM);

	// reset while FIFO is disabled
	_fifo_watermark_interrupt_timestamp = 0;
	_fifo_read_samples.store(0);

	// Writing to water mark level trigger in register 0x3D clears the FIFO buffer.
	const uint8_t fifo_water_mark_level_trigger_retain = _fifo_gyro_samples;
	RegisterWrite(Register::FIFO_CONFIG_0, fifo_water_mark_level_trigger_retain);
}

} // namespace Bosch_BMI088_Gyroscope
