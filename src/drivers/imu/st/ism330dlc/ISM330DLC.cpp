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

#include "ISM330DLC.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ISM330DLC::ISM330DLC(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		     spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SPI(MODULE_NAME, nullptr, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_drdy_gpio(drdy_gpio),
	_px4_accel(get_device_id(), ORB_PRIO_HIGH, rotation),
	_px4_gyro(get_device_id(), ORB_PRIO_HIGH, rotation)
{
	set_device_type(DRV_IMU_DEVTYPE_ST_ISM330DLC);

	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ST_ISM330DLC);
	_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ST_ISM330DLC);

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ISM330DLC::~ISM330DLC()
{
	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_interval_perf);
}

int ISM330DLC::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ISM330DLC::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ISM330DLC::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ISM330DLC::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("FIFO empty interval: %d us (%.3f Hz)", _fifo_empty_interval_us,
		 static_cast<double>(1000000 / _fifo_empty_interval_us));

	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_interval_perf);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}

int ISM330DLC::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ISM330DLC::RunImpl()
{
	switch (_state) {
	case STATE::RESET:
		// CTRL3_C: SW_RESET
		RegisterSetBits(Register::CTRL3_C, CTRL3_C_BIT::SW_RESET);
		_reset_timestamp = hrt_absolute_time();
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(15_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		if ((RegisterRead(Register::WHO_AM_I) == WHOAMI)
		    && !(RegisterRead(Register::CTRL3_C) & CTRL3_C_BIT::SW_RESET)) {

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 100_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

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
			// try again in 10 ms
			ScheduleDelayed(10_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = 0;
			uint8_t samples = 0;

			if (_data_ready_interrupt_enabled) {
				// re-schedule as watchdog timeout
				ScheduleDelayed(10_ms);

				// timestamp set in data ready interrupt
				if (!_force_fifo_count_check) {
					samples = _fifo_read_samples.load();

				} else {
					// Number of unread words (16-bit axes) stored in FIFO.
					const uint16_t fifo_count = RegisterRead(Register::FIFO_STATUS1);
					samples = (fifo_count / sizeof(FIFO::DATA) / SAMPLES_PER_TRANSFER) * SAMPLES_PER_TRANSFER; // round down to nearest
				}

				timestamp_sample = _fifo_watermark_interrupt_timestamp;
			}

			// Number of unread words (16-bit axes) stored in FIFO.
			//const uint8_t fifo_words = RegisterRead(Register::FIFO_STATUS1);

			// check for FIFO status
			const uint8_t FIFO_STATUS2 = RegisterRead(Register::FIFO_STATUS2);

			if (FIFO_STATUS2 & FIFO_STATUS2_BIT::OVER_RUN) {
				// overflow
				perf_count(_fifo_overflow_perf);
				FIFOReset();
				return;

			} else if (FIFO_STATUS2 & FIFO_STATUS2_BIT::FIFO_EMPTY) {
				// fifo empty could indicate a problem, reset
				perf_count(_fifo_empty_perf);
				FIFOReset();
				return;
			}

			// FIFO pattern: indicates Next reading from FIFO output registers (Gx, Gy, Gz, XLx, XLy, XLz)
			const uint8_t fifo_pattern = RegisterRead(Register::FIFO_STATUS3);

			if (fifo_pattern != 0) {
				PX4_DEBUG("check FIFO pattern: %d", fifo_pattern);
			}



			bool failure = false;

			// manually check FIFO count if no samples from DRDY or timestamp looks bogus
			if (!_data_ready_interrupt_enabled || (samples == 0)
			    || (hrt_elapsed_time(&timestamp_sample) > (_fifo_empty_interval_us / 2))) {

				// use the time now roughly corresponding with the last sample we'll pull from the FIFO
				timestamp_sample = hrt_absolute_time();

				// Number of unread words (16-bit axes) stored in FIFO.
				const uint16_t fifo_count = RegisterRead(Register::FIFO_STATUS1);
				samples = (fifo_count / sizeof(FIFO::DATA) / SAMPLES_PER_TRANSFER) * SAMPLES_PER_TRANSFER; // round down to nearest
			}

			if (samples > FIFO_MAX_SAMPLES) {
				// not technically an overflow, but more samples than we expected or can publish
				perf_count(_fifo_overflow_perf);
				failure = true;
				FIFOReset();

			} else if (samples >= SAMPLES_PER_TRANSFER) {
				// require at least SAMPLES_PER_TRANSFER (we want at least 1 new accel sample per transfer)
				if (!FIFORead(timestamp_sample, samples)) {
					failure = true;
					_px4_accel.increase_error_count();
					_px4_gyro.increase_error_count();
				}

			} else if (samples == 0) {
				failure = true;
				perf_count(_fifo_empty_perf);
			}

			if (failure || hrt_elapsed_time(&_last_config_check_timestamp) > 10_ms) {
				// check registers incrementally
				if (RegisterCheck(_register_cfg[_checked_register], true)) {
					_last_config_check_timestamp = timestamp_sample;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reconfigure
					PX4_DEBUG("Health check failed, reconfiguring");
					_state = STATE::CONFIGURE;
					ScheduleNow();
				}

			} else {
				// periodically update temperature (1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) > 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = timestamp_sample;
				}
			}
		}

		break;
	}
}

void ISM330DLC::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = 1000; // default to 1 kHz
	}

	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = SAMPLES_PER_TRANSFER * FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	_fifo_accel_samples = math::min(_fifo_empty_interval_us / (1e6f / ACCEL_RATE), (float)FIFO_MAX_SAMPLES);

	_px4_accel.set_update_rate(1e6f / _fifo_empty_interval_us);
	_px4_gyro.set_update_rate(1e6f / _fifo_empty_interval_us);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ISM330DLC::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold in number of bytes
	// 1 LSB = 2 bytes
	const uint16_t fifo_watermark_threshold = (samples * sizeof(FIFO::DATA)) / 2;

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CTRL1) {
			// FTH_[7:0]
			r.set_bits = fifo_watermark_threshold & 0xFF;

		} else if (r.reg == Register::FIFO_CTRL2) {
			// FTH_[10:8]
			r.set_bits = (fifo_watermark_threshold >> 8) & 0b00000111;
		}
	}
}

bool ISM330DLC::Configure()
{
	bool success = true;

	for (const auto &reg : _register_cfg) {
		if (!RegisterCheck(reg)) {
			success = false;
		}
	}

	_px4_accel.set_scale(0.488f * (CONSTANTS_ONE_G / 1000.f)); // 0.488 mg/LSB
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);

	_px4_gyro.set_scale(math::radians(70.f / 1000.f)); // 70 mdps/LSB
	_px4_gyro.set_range(math::radians(2000.f));

	return success;
}

int ISM330DLC::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ISM330DLC *>(arg)->DataReady();
	return 0;
}

void ISM330DLC::DataReady()
{
	perf_count(_drdy_interval_perf);
	_fifo_watermark_interrupt_timestamp = hrt_absolute_time();
	_fifo_read_samples.store(_fifo_gyro_samples);
	ScheduleNow();
}

bool ISM330DLC::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &ISM330DLC::DataReadyInterruptCallback, this) == 0;
}

bool ISM330DLC::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool ISM330DLC::RegisterCheck(const register_config_t &reg_cfg, bool notify)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	if (!success) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

		if (notify) {
			perf_count(_bad_register_perf);
			_px4_accel.increase_error_count();
			_px4_gyro.increase_error_count();
		}
	}

	return success;
}

uint8_t ISM330DLC::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void ISM330DLC::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ISM330DLC::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
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

bool ISM330DLC::FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples)
{
	perf_begin(_transfer_perf);
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 3, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		perf_count(_bad_transfer_perf);
		return false;
	}

	perf_end(_transfer_perf);

	const uint16_t fifo_count_bytes = buffer.FIFO_STATUS1; // Number of unread words (16-bit axes) stored in FIFO.
	const uint16_t fifo_count_samples = fifo_count_bytes / sizeof(FIFO::DATA);

	if (fifo_count_samples == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	if (fifo_count_bytes >= FIFO::SIZE) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const uint16_t valid_samples = math::min(samples, fifo_count_samples);

	if (fifo_count_samples < samples) {
		// force check if there is somehow fewer samples actually in the FIFO (potentially a serious error)
		_force_fifo_count_check = true;

	} else if (fifo_count_samples > samples + 4) {
		// if we're more than a few samples behind force FIFO_COUNT check
		_force_fifo_count_check = true;

	} else {
		// skip earlier FIFO_COUNT and trust DRDY count if we're in sync
		_force_fifo_count_check = false;
	}

	if (valid_samples > 0) {
		ProcessGyro(timestamp_sample, buffer, valid_samples);
		ProcessAccel(timestamp_sample, buffer, valid_samples);

		return true;
	}

	return false;
}

void ISM330DLC::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// Bypass mode is also used to reset the FIFO when in FIFO mode.

	// FIFO_CTRL5 - disable FIFO
	RegisterWrite(Register::FIFO_CTRL5, 0);


	// reset while FIFO is disabled
	_fifo_watermark_interrupt_timestamp = 0;
	_fifo_read_samples.store(0);

	// FIFO_CTRL3 & FIFO_CTRL5: enable both gyro and accel
	for (const auto &r : _register_cfg) {
		if ((r.reg == Register::FIFO_CTRL3) || (r.reg == Register::FIFO_CTRL5)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}

}

void ISM330DLC::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFOTransferBuffer &buffer,
			     const uint8_t samples)
{
	PX4Accelerometer::FIFOSample accel;
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = _fifo_empty_interval_us / _fifo_accel_samples;

	int accel_samples = 0;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = buffer.f[i];

		int16_t accel_x = combine(fifo_sample.OUTX_L_XL, fifo_sample.OUTX_H_XL);
		int16_t accel_y = combine(fifo_sample.OUTY_L_XL, fifo_sample.OUTY_H_XL);
		int16_t accel_z = combine(fifo_sample.OUTZ_L_XL, fifo_sample.OUTZ_H_XL);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[accel_samples] = accel_x;
		accel.y[accel_samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.z[accel_samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
	}

	_px4_accel.updateFIFO(accel);
}

void ISM330DLC::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFOTransferBuffer &buffer,
			    const uint8_t samples)
{
	PX4Gyroscope::FIFOSample gyro;
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = _fifo_empty_interval_us / _fifo_gyro_samples;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = buffer.f[i];

		const int16_t gyro_x = combine(fifo_sample.OUTX_L_G, fifo_sample.OUTX_H_G);
		const int16_t gyro_y = combine(fifo_sample.OUTY_L_G, fifo_sample.OUTY_H_G);
		const int16_t gyro_z = combine(fifo_sample.OUTZ_L_G, fifo_sample.OUTZ_H_G);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.updateFIFO(gyro);
}

void ISM330DLC::UpdateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::OUT_TEMP_L) | DIR_READ;

	if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	// 16 bits in two’s complement format with a sensitivity of 256 LSB/°C. The output zero level corresponds to 25 °C.
	const int16_t OUT_TEMP = combine(temperature_buf[1], temperature_buf[2]);
	const float temperature = ((float)OUT_TEMP / 256.0f) + 25.0f;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}
