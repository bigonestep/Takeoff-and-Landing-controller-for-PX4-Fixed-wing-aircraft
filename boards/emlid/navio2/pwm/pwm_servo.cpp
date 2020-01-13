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

#include <drivers/drv_pwm_output.h>

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <px4_platform_common/log.h>

static const int MAX_NUM_PWM = 14;
static const int FREQUENCY_PWM = 400;

int _pwm_fd[MAX_NUM_PWM];
static constexpr int _pwm_num{14};

static char _device[64] = "/sys/class/pwm/pwmchip0";

// static int pwm_read_sysfs(char *path)
// {
// 	PX4_DEBUG("pwm_read_sysfs: %s", path);

// 	int fd = ::open(path, O_RDONLY);

// 	if (fd == -1) {
// 		return -errno;
// 	}

// 	char data[16] {};
// 	::read(fd, data, sizeof(data));

// 	::close(fd);

// 	return atoi(data);
// }

static int pwm_write_sysfs(char *path, int value)
{
	PX4_DEBUG("pwm_write_sysfs: %s - %d", path, value);

	int fd = ::open(path, O_WRONLY | O_CLOEXEC);

	if (fd == -1) {
		return -errno;
	}

	char data[16] {};
	int n = ::snprintf(data, sizeof(data), "%u", value);

	if (n > 0) {
		n = ::write(fd, data, n); // This n is not used, but to avoid a compiler error.
	}

	::close(fd);

	return 0;
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	PX4_DEBUG("up_pwm_servo_init: %X", channel_mask);

	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		_pwm_fd[i] = -1;
	}

	// ::snprintf(path, sizeof(path), "%s/export", _device);
	for (int i = 0; i < _pwm_num; ++i) {
		char path[128] {};
		::snprintf(path, sizeof(path), "%s/export", _device);

		if (pwm_write_sysfs(path, i) < 0) {
			PX4_ERR("PWM export failed");
		}
	}

	// ::snprintf(path, sizeof(path), "%s/pwm%u/enable", _device, i);
	for (int i = 0; i < _pwm_num; ++i) {
		char path[128] {};
		::snprintf(path, sizeof(path), "%s/pwm%u/enable", _device, i);

		if (pwm_write_sysfs(path, 1) < 0) {
			PX4_ERR("PWM enable failed");
		}
	}

	// ::snprintf(path, sizeof(path), "%s/pwm%u/period", _device, i);
	for (int i = 0; i < _pwm_num; ++i) {
		char path[128] {};
		::snprintf(path, sizeof(path), "%s/pwm%u/period", _device, i);

		if (pwm_write_sysfs(path, (int)1e9 / FREQUENCY_PWM)) {
			PX4_ERR("PWM period failed");
		}
	}

	// ::snprintf(path, sizeof(path), "%s/pwm%u/duty_cycle", _device, i);
	for (int i = 0; i < _pwm_num; ++i) {
		char path[128] {};
		::snprintf(path, sizeof(path), "%s/pwm%u/duty_cycle", _device, i);
		_pwm_fd[i] = ::open(path, O_WRONLY | O_CLOEXEC);

		if (_pwm_fd[i] == -1) {
			PX4_ERR("PWM: Failed to open duty_cycle.");
			return -errno;
		}
	}

	return 0;
}

void up_pwm_servo_deinit()
{
	PX4_DEBUG("up_pwm_servo_deinit");

	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		if (_pwm_fd[i] != -1) {
			::close(_pwm_fd[i]);
		}
	}
}

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	PX4_DEBUG("up_pwm_servo_set: %X %d", channel, value);

	// write to %s/pwm%u/duty_cycle
	char data[16] {};
	int n = ::snprintf(data, sizeof(data), "%u", value * 1000); // convert to nanoseconds
	int write_ret = ::write(_pwm_fd[channel], data, n);

	if (n != write_ret) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

servo_position_t up_pwm_servo_get(unsigned channel)
{
	PX4_DEBUG("up_pwm_servo_get: %X", channel);

	// read from %s/pwm%u/duty_cycle
	char data[16] {};
	int ret = ::read(_pwm_fd[channel], data, sizeof(data));

	if (ret < 0) {
		return 0;
	}

	return atoi(data) / 1000;
}

void up_pwm_update()
{
	PX4_DEBUG("up_pwm_update");
	// Trigger all timer's channels in Oneshot mode to fire the oneshots with updated values.
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	PX4_DEBUG("up_pwm_servo_get_rate_group: %X", group);

	return 1;
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	PX4_DEBUG("up_pwm_servo_set_rate_group_update: %X %d", group, rate);

	int frequency = (int)1e9 / rate;

	// ::snprintf(path, sizeof(path), "%s/pwm%u/period", _device, i);
	for (int i = 0; i < _pwm_num; ++i) {
		char path[128] {};
		::snprintf(path, sizeof(path), "%s/pwm%u/period", _device, i);

		if (pwm_write_sysfs(path, frequency)) {
			PX4_ERR("PWM period failed");
		}
	}

	return 0;
}

void up_pwm_servo_arm(bool armed)
{
	PX4_DEBUG("up_pwm_servo_arm: %d", armed);

	// %s/pwm%u/enable
	// %s/export
}
