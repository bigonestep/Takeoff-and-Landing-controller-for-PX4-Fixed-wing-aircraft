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

#include "BMI088_accel.hpp"

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t BMI088_accel::_checked_registers[BMI088_ACCEL_NUM_CHECKED_REGISTERS] = {    BMI088_ACC_CHIP_ID,
											  BMI088_ACC_BW,
											  BMI088_ACC_RANGE,
											  BMI088_ACC_INT_EN_1,
											  BMI088_ACC_INT_MAP_1,
										     };

BMI088_accel::BMI088_accel(int bus, uint32_t device, enum Rotation rotation) :
	BMI088("BMI088_ACCEL", nullptr, bus, device, SPIDEV_MODE3, BMI088_BUS_SPEED, rotation),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), (external() ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi088: accel read")),
	_measure_interval(perf_alloc(PC_INTERVAL, "bmi088: accel measure interval")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi088: accel bad transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi088: accel bad registers")),
	_duplicates(perf_alloc(PC_COUNT, "bmi088: accelduplicates")),
	_got_duplicate(false)
{
	_px4_accel.set_device_type(DRV_ACC_DEVTYPE_BMI088);
}

BMI088_accel::~BMI088_accel()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_measure_interval);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
BMI088_accel::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	return reset();
}

int BMI088_accel::reset()
{
	write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET); // Soft-reset
	up_udelay(5000);

	write_checked_reg(BMI088_ACC_BW, BMI088_ACCEL_BW_500);		// Write accel bandwidth (DLPF)
	write_checked_reg(BMI088_ACC_RANGE, BMI088_ACCEL_RANGE_3_G);	// Write range
	write_checked_reg(BMI088_ACC_INT_EN_1, BMI088_ACC_DRDY_INT_EN);	// Enable DRDY interrupt
	write_checked_reg(BMI088_ACC_INT_MAP_1, BMI088_ACC_DRDY_INT1);	// Map DRDY interrupt on pin INT1

	set_accel_range(BMI088_ACCEL_DEFAULT_RANGE_G);	// set accel range

	// Enable Accelerometer in normal mode
	write_reg(BMI088_ACC_PMU_LPW, BMI088_ACCEL_NORMAL);
	usleep(1000);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
			if (read_reg(_checked_registers[i]) != _checked_values[i]) {
				write_reg(_checked_registers[i], _checked_values[i]);
				all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}

	return OK;
}

int
BMI088_accel::probe()
{
	/* look for device ID */
	_whoami = read_reg(BMI088_ACC_CHIP_ID);

	// verify product revision
	switch (_whoami) {
	case BMI088_ACC_WHO_AM_I:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
BMI088_accel::test_error()
{
	write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET);
	PX4_INFO("error triggered");
	print_registers();
}

void
BMI088_accel::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
BMI088_accel::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int
BMI088_accel::set_accel_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI088_ACCEL_RANGE_3_G | BMI088_ACCEL_RANGE_24_G;
	float lsb_per_g;

	if (max_g == 0) {
		max_g = 24;
	}

	if (max_g <= 3) {
		//max_accel_g = 3;
		setbits |= BMI088_ACCEL_RANGE_3_G;
		lsb_per_g = 1024;

	} else if (max_g <= 6) {
		//max_accel_g = 6;
		setbits |= BMI088_ACCEL_RANGE_6_G;
		lsb_per_g = 512;

	} else if (max_g <= 12) {
		//max_accel_g = 12;
		setbits |= BMI088_ACCEL_RANGE_12_G;
		lsb_per_g = 256;

	} else if (max_g <= 24) {
		//max_accel_g = 24;
		setbits |= BMI088_ACCEL_RANGE_24_G;
		lsb_per_g = 128;

	} else {
		return -EINVAL;
	}

	_px4_accel.set_scale(CONSTANTS_ONE_G / lsb_per_g);

	modify_reg(BMI088_ACC_RANGE, clearbits, setbits);

	return OK;
}

void
BMI088_accel::start()
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	ScheduleOnInterval(BMI088_ACCEL_DEFAULT_RATE - BMI088_TIMER_REDUCTION, 1000);
}

void
BMI088_accel::stop()
{
	ScheduleClear();
}

void
BMI088_accel::Run()
{
	/* make another measurement */
	measure();
}

void
BMI088_accel::check_registers(void)
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) !=
	    _checked_values[_checked_next]) {
		_checked_bad[_checked_next] = v;

		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET);
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % BMI088_ACCEL_NUM_CHECKED_REGISTERS;
}

void
BMI088_accel::measure()
{
	perf_count(_measure_interval);

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct Report {
		int16_t     accel_x;
		int16_t     accel_y;
		int16_t     accel_z;
		int8_t     temp;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the BMI088 in one pass.
	 */
	uint8_t index = 0;
	uint8_t accel_data[8] {};
	accel_data[index] = BMI088_ACC_X_L | DIR_READ;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != transfer(accel_data, accel_data, sizeof(accel_data))) {
		return;
	}

	check_registers();

	/* Extracting accel data from the read data */
	index = 1;
	uint16_t lsb, msb, msblsb;

	lsb = (uint16_t)accel_data[index++];
	uint8_t status_x = (lsb & BMI088_NEW_DATA_MASK);
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_x = ((int16_t)msblsb >> 4); /* Data in X axis */

	lsb = (uint16_t)accel_data[index++];
	uint8_t status_y = (lsb & BMI088_NEW_DATA_MASK);
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_y = ((int16_t)msblsb >> 4); /* Data in Y axis */

	lsb = (uint16_t)accel_data[index++];
	uint8_t status_z = (lsb & BMI088_NEW_DATA_MASK);
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_z = ((int16_t)msblsb >> 4); /* Data in Z axis */

	// Byte
	report.temp = accel_data[index];

	// Checking the status of new data
	if ((!status_x) || (!status_y) || (!status_z)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return;
	}


	_got_duplicate = false;

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0) {
		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the bmi088 accel does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again, but don't return any data yet
		_register_wait--;
		return;
	}

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	const uint64_t error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);
	_px4_accel.set_error_count(error_count);

	/*
	 * Temperature is reported as Eight-bit 2’s complement sensor temperature value
	 * with 0.5 °C/LSB sensitivity and an offset of 23.0 °C
	 */
	_px4_accel.set_temperature((report.temp * 0.5f) + 23.0f);

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 *   at a nominally 'zero' input. Therefore the offset has to
	 *   be subtracted.
	 *
	 */
	_px4_accel.update(timestamp_sample, report.accel_x, report.accel_y, report.accel_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI088_accel::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_interval);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);

	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}

		if (v != _checked_bad[i]) {
			::printf("reg %02x:%02x was bad %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_bad[i]);
		}
	}

	_px4_accel.print_status();
}

void
BMI088_accel::print_registers()
{
	uint8_t index = 0;
	printf("BMI088 accel registers\n");

	uint8_t reg = _checked_registers[index++];
	uint8_t v = read_reg(reg);
	printf("Accel Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Bw: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Range: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Int-en-1: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Int-Map-1: %02x:%02x ", (unsigned)reg, (unsigned)v);

	printf("\n");
}
