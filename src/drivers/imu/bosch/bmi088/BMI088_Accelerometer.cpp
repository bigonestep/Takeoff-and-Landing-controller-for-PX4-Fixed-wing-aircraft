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

#include "BMI088_Accelerometer.hpp"

#include <ecl/geo/geo.h> // CONSTANTS_ONE_G
#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;

namespace Bosch_BMI088_Accelerometer
{

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

BMI088_Accelerometer::BMI088_Accelerometer(int bus, uint32_t device, enum Rotation rotation) :
	SPI("BMI088_ACCEL", nullptr, bus, device, SPIDEV_MODE3, SPI_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), ORB_PRIO_DEFAULT, rotation)
{
	set_device_type(DRV_DEVTYPE_BMI088);

	_px4_accel.set_device_type(DRV_DEVTYPE_BMI088);

	ConfigureSampleRate(400);
}

BMI088_Accelerometer::~BMI088_Accelerometer()
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

bool BMI088_Accelerometer::Init()
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

void BMI088_Accelerometer::Stop()
{
	// wait until stopped
	while (_state.load() != STATE::STOPPED) {
		_state.store(STATE::REQUEST_STOP);
		ScheduleNow();
		px4_usleep(10);
	}
}

bool BMI088_Accelerometer::Reset()
{
	_state.store(STATE::RESET);
	ScheduleClear();
	ScheduleNow();
	return true;
}

void BMI088_Accelerometer::PrintInfo()
{
	PX4_INFO("FIFO empty interval: %d us (%.3f Hz)", _fifo_empty_interval_us,
		 static_cast<double>(1000000 / _fifo_empty_interval_us));

	PX4_INFO("FIFO accel samples: %d", _fifo_accel_samples);

	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_interval_perf);

	_px4_accel.print_status();

	for (const auto &reg : _register_cfg) {
		printf("Register: 0x%02hhx - set: 0x%02hhx (%d) clear: 0x%02hhx (%d) value: 0x%02hhx\n", reg.reg, reg.set_bits, (reg.set_bits & RegisterRead(reg.reg)), reg.clear_bits, RegisterRead(reg.reg));
	}

	printf("BMI088 accel registers\n");

	PX4_INFO("ACC_CHIP_ID:\t0x%02hhx", RegisterRead(Register::ACC_CHIP_ID));

	PX4_INFO("FIFO_LENGTH_0:\t0x%02hhx", RegisterRead(Register::FIFO_LENGTH_0));
	PX4_INFO("FIFO_LENGTH_1:\t0x%02hhx", RegisterRead(Register::FIFO_LENGTH_1));

	PX4_INFO("ACC_CONF:\t0x%02hhx", RegisterRead(Register::ACC_CONF));
	PX4_INFO("ACC_RANGE:\t0x%02hhx", RegisterRead(Register::ACC_RANGE));

	PX4_INFO("FIFO_WTM_0:\t0x%02hhx", RegisterRead(Register::FIFO_WTM_0));
	PX4_INFO("FIFO_WTM_1:\t0x%02hhx", RegisterRead(Register::FIFO_WTM_1));

	PX4_INFO("FIFO_CONFIG_0:\t0x%02hhx", RegisterRead(Register::FIFO_CONFIG_0));
	PX4_INFO("FIFO_CONFIG_1:\t0x%02hhx", RegisterRead(Register::FIFO_CONFIG_1));

	PX4_INFO("INT1_IO_CONF:\t0x%02hhx", RegisterRead(Register::INT1_IO_CONF));
	PX4_INFO("INT2_IO_CONF:\t0x%02hhx", RegisterRead(Register::INT2_IO_CONF));

	PX4_INFO("INT1_INT2_MAP_DATA:\t0x%02hhx", RegisterRead(Register::INT1_INT2_MAP_DATA));

	PX4_INFO("ACC_PWR_CONF:\t0x%02hhx", RegisterRead(Register::ACC_PWR_CONF));
	PX4_INFO("ACC_PWR_CTRL:\t0x%02hhx", RegisterRead(Register::ACC_PWR_CTRL));
	PX4_INFO("ACC_SOFTRESET:\t0x%02hhx", RegisterRead(Register::ACC_SOFTRESET));

	printf("\n");
}

int BMI088_Accelerometer::probe()
{
	/* 6.1 Serial Peripheral Interface (SPI)
	 * ... the accelerometer part starts always in I2C mode
	 * (regardless of the level of the PS pin) and needs to be changed to SPI
	 *  mode actively by sending a rising edge on the CSB1 pin
	 *  (chip select of the accelerometer), on which the accelerometer part
	 *  switches to SPI mode and stays in this mode until the next power-up-reset.
	 *
	 *  To change the sensor to SPI mode in the initialization phase, the user
	 *  could perfom a dummy SPI read operation, e.g. of register ACC_CHIP_ID
	 *  (the obtained value will be invalid).In case of read operations,
	 */
	RegisterRead(Register::ACC_CHIP_ID);

	const uint8_t ACC_CHIP_ID = RegisterRead(Register::ACC_CHIP_ID);

	if (ACC_CHIP_ID != ID) {
		PX4_WARN("unexpected ACC_CHIP_ID 0x%02x", ACC_CHIP_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI088_Accelerometer::Run()
{
	switch (_state.load()) {
	case STATE::RESET:
		// ACC_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor
		// Following a delay of 1 ms, all configuration settings are overwritten with their reset value.
		RegisterWrite(Register::ACC_SOFTRESET, 0xB6);
		px4_usleep(2000);

		RegisterWrite(Register::ACC_PWR_CONF, 0);

		// ACC_PWR_CTRL: Enter normal mode by writing ‘4’ to ACC_PWR_CTRL and wait for 50 ms
		RegisterWrite(Register::ACC_PWR_CTRL, 0x04);

		_reset_timestamp = hrt_absolute_time();
		_state.store(STATE::WAIT_FOR_RESET);
		ScheduleDelayed(50_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		//  Document Number:
		if (RegisterRead(Register::ACC_CHIP_ID) == ID) {

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
				PX4_ERR("Reset not complete, check again in 1 ms");
				ScheduleDelayed(1_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			PX4_INFO("Configure");
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
			PX4_ERR("Configure failed, retrying");
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

				// FIFO length registers FIFO_LENGTH_1 and FIFO_LENGTH_0 contain the 14 bit FIFO byte
				uint8_t fifo_len_buf[4] {};
				fifo_len_buf[0] = static_cast<uint8_t>(Register::FIFO_LENGTH_0) | DIR_READ;

				// fifo_len_buf[1] dummy byte
				if (transfer(&fifo_len_buf[0], &fifo_len_buf[0], sizeof(fifo_len_buf)) != PX4_OK) {
					failure = true;
				}

				const uint8_t FIFO_LENGTH_0 = fifo_len_buf[2];        // fifo_byte_counter[7:0]
				const uint8_t FIFO_LENGTH_1 = fifo_len_buf[3] & 0xFD; // fifo_byte_counter[13:8]

				const uint16_t fifo_byte_counter = FIFO_LENGTH_0 + (FIFO_LENGTH_1 << 8);

				// An empty FIFO corresponds to 0x8000
				if ((fifo_byte_counter == 0) || (fifo_byte_counter == 0x8000)) {
					perf_count(_fifo_empty_perf);
					failure = true;
					samples = 0;

				} else {
					samples = fifo_byte_counter / sizeof(FIFO::DATA);
				}

				//printf("fifo_byte_counter: %d samples: %d\n", fifo_byte_counter, samples);
			}

			if (samples > FIFO_MAX_SAMPLES) {
				// not technically an overflow, but more samples than we expected or can publish
				perf_count(_fifo_overflow_perf);
				failure = true;
				FIFOReset();

			} else if (samples >= 1) {
				if (!FIFORead(timestamp_sample, samples)) {
					failure = true;
					_px4_accel.increase_error_count();
				}
			}

			if (failure || hrt_elapsed_time(&_last_config_check_timestamp) > 1_ms) {
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

			} else {
				// periodically update temperature (1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) > 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = timestamp_sample;
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

void BMI088_Accelerometer::ConfigureAccel()
{
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE);

	switch (ACC_RANGE) {
	case acc_range_3g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.0f);
		_px4_accel.set_range(1.0f);
		break;

	case acc_range_6g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.0f);
		_px4_accel.set_range(1.0f);
		break;

	case acc_range_12g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.0f);
		_px4_accel.set_range(1.0f);
		break;

	case acc_range_24g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.0f);
		_px4_accel.set_range(1.0f);
		break;
	}
}

void BMI088_Accelerometer::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = 800; // default to 800 Hz
	}

	_fifo_empty_interval_us = math::max(((1000000 / sample_rate) / 625) * 625, 625); // round down to nearest 625 us
	_fifo_accel_samples = math::min(_fifo_empty_interval_us / (1000000 / ACCEL_RATE), FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual accel sample limit
	_fifo_empty_interval_us = _fifo_accel_samples * (1000000 / ACCEL_RATE);

	_px4_accel.set_update_rate(1000000 / _fifo_empty_interval_us);
	_px4_accel.set_integrator_reset_interval(2500); // 400 Hz (2.5 ms)

	// FIFO_WTM: 13 bit FIFO watermark level value
	// 7 bytes per sensor reading * 4 (2.5 ms of data at 1600 Hz ODR)
	const uint16_t fifo_watermark_threshold = _fifo_accel_samples * sizeof(FIFO::DATA);

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_WTM_0) {
			r.set_bits = fifo_watermark_threshold & 0x00FF;

		} else if (r.reg == Register::FIFO_WTM_1) {
			r.set_bits = (fifo_watermark_threshold & 0x0700) >> 8;
		}
	}
}

bool BMI088_Accelerometer::Configure()
{
	bool success = true;

	for (const auto &reg : _register_cfg) {
		if (!RegisterCheck(reg)) {
			success = false;
		}
	}

	ConfigureAccel();

	return success;
}

int BMI088_Accelerometer::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BMI088_Accelerometer *>(arg)->DataReady();
	return 0;
}

void BMI088_Accelerometer::DataReady()
{
	_fifo_watermark_interrupt_timestamp = hrt_absolute_time();
	_fifo_read_samples.store(_fifo_accel_samples);
	ScheduleNow();
	perf_count(_drdy_interval_perf);
}

bool BMI088_Accelerometer::DataReadyInterruptConfigure()
{
	int ret_setevent = -1;

	// Setup data ready on rising edge
	// TODO: cleanup horrible DRDY define mess
	// Setup data ready Interrupt on Falling edge for better noise immunity

	return (ret_setevent == 0);
}

bool BMI088_Accelerometer::DataReadyInterruptDisable()
{
	int ret_setevent = -1;

	// Disable data ready callback
	// TODO: cleanup horrible DRDY define mess

	return (ret_setevent == 0);
}

bool BMI088_Accelerometer::RegisterCheck(const register_config_t &reg_cfg, bool notify)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_ERR("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) == 0)) {
		PX4_ERR("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	if (!success) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

		if (reg_cfg.reg == Register::ACC_RANGE) {
			ConfigureAccel();
		}

		if (notify) {
			perf_count(_bad_register_perf);
			_px4_accel.increase_error_count();
		}
	}

	return success;
}

uint8_t BMI088_Accelerometer::RegisterRead(Register reg)
{
	// 6.1.2 SPI interface of accelerometer part
	//
	// In case of read operations of the accelerometer part, the requested data
	// is not sent immediately, but instead first a dummy byte is sent, and
	// after this dummy byte the actual requested register content is transmitted.
	uint8_t cmd[3] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	// cmd[1] dummy byte
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[2];
}

void BMI088_Accelerometer::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void BMI088_Accelerometer::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
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

void BMI088_Accelerometer::RegisterSetBits(Register reg, uint8_t setbits)
{
	RegisterSetAndClearBits(reg, setbits, 0);
}

void BMI088_Accelerometer::RegisterClearBits(Register reg, uint8_t clearbits)
{
	RegisterSetAndClearBits(reg, 0, clearbits);
}

bool BMI088_Accelerometer::FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples)
{
	TransferBuffer *report = (TransferBuffer *)_dma_data_buffer;
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 2, FIFO::SIZE);
	memset(report, 0, transfer_size);
	report->cmd = static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ;

	perf_begin(_transfer_perf);

	if (transfer(_dma_data_buffer, _dma_data_buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		perf_count(_bad_transfer_perf);
		return false;
	}

	perf_end(_transfer_perf);

	// first find all sensor data frames in the buffer
	unsigned fifo_buffer_index = 2;

	// array to store the index to every sensor data frame
	int sensor_data_frame_count = 0;
	uint8_t sensor_data_frame_index_array[samples] {};

	while (fifo_buffer_index < transfer_size) {
		// header set by first 6 bits
		switch (_dma_data_buffer[fifo_buffer_index] & 0xFC) {
		case FIFO::header::sensor_data_frame: {
				// Acceleration sensor data frame
				// Frame length: 7 bytes (1 byte header + 6 bytes payload)
				PX4_DEBUG("Acceleration sensor data frame");

				// check for [INT2 tag]
				if (_dma_data_buffer[fifo_buffer_index] & Bit1) {
					//PX4_INFO("INT2 tag");
				}

				sensor_data_frame_index_array[sensor_data_frame_count] = fifo_buffer_index;
				sensor_data_frame_count++;

				fifo_buffer_index += 7;
			}
			break;

		case FIFO::header::skip_frame: {
				// Skip Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("Skip Frame");
				fifo_buffer_index += 2;
			}
			break;

		case FIFO::header::sensor_time_frame: {
				// Sensortime Frame
				// Frame length: 4 bytes (1 byte header + 3 bytes payload)
				PX4_DEBUG("Sensortime Frame");
				fifo_buffer_index += 4;
			}
			break;

		case FIFO::header::FIFO_input_config_frame: {
				// FIFO input config Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("FIFO input config Frame");
				fifo_buffer_index += 2;
			}
			break;

		case FIFO::header::sample_drop_frame: {
				// Sample drop Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("Sample drop Frame");
				fifo_buffer_index += 2;
			}
			break;

		default:
			fifo_buffer_index++;
			break;
		}
	}

	PX4Accelerometer::FIFOSample accel;
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = _fifo_empty_interval_us / _fifo_accel_samples;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = report->f[i];

		const int16_t accel_x = combine(fifo_sample.ACC_X_MSB, fifo_sample.ACC_X_LSB);
		const int16_t accel_y = combine(fifo_sample.ACC_Y_MSB, fifo_sample.ACC_Y_LSB);
		const int16_t accel_z = combine(fifo_sample.ACC_Z_MSB, fifo_sample.ACC_Z_LSB);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel_x;
		accel.y[i] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.z[i] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
	}

	_px4_accel.updateFIFO(accel);

	return true;
}

void BMI088_Accelerometer::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// trigger a FIFO reset by writing 0xB0 to ACC_SOFTRESET (register 0x7E).
	RegisterWrite(Register::ACC_SOFTRESET, 0xB0);

	// reset while FIFO is disabled
	_fifo_watermark_interrupt_timestamp = 0;
	_fifo_read_samples.store(0);
}

void BMI088_Accelerometer::UpdateTemperature()
{
	// stored in an 11-bit value in 2’s complement format
	uint8_t temperature_buf[4] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_MSB) | DIR_READ;
	// temperature_buf[1] dummy byte

	if (transfer(&temperature_buf[0], &temperature_buf[0], sizeof(temperature_buf)) != PX4_OK) {
		return;
	}

	const uint8_t TEMP_MSB = temperature_buf[2];
	const uint8_t TEMP_LSB = temperature_buf[3];

	// Datasheet 5.3.7: Register 0x22 – 0x23: Temperature sensor data
	uint16_t Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32);
	int16_t Temp_int11 = 0;

	if (Temp_uint11 > 1023) {
		Temp_int11 = Temp_uint11 - 2048;

	} else {
		Temp_int11 = Temp_uint11;
	}

	float temperature = (Temp_int11 * 0.125f) + 23.0f;	// Temp_int11 * 0.125°C/LSB + 23°C

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
	}
}

} // namespace Bosch_BMI088_Accelerometer
