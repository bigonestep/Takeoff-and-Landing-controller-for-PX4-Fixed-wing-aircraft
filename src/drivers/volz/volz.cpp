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

#include <float.h>
#include <math.h>

#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_arch/dshot.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/esc_status.h>

using namespace time_literals;

class VolzOutput : public cdev::CDev, public ModuleBase<VolzOutput>, public OutputModuleInterface
{
public:

	VolzOutput();
	virtual ~VolzOutput();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** change the mode of the running module */
	static int module_new_mode(PortMode new_mode);

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override;

	/**
	 * Send a dshot command to one or all motors
	 * This is expected to be called from another thread.
	 * @param num_repetitions number of times to repeat, set at least to 1
	 * @param motor_index index or -1 for all
	 * @return 0 on success, <0 error otherwise
	 */
	int sendCommandThreadSafe(dshot_command_t command, int num_repetitions, int motor_index);

	void retrieveAndPrintESCInfoThreadSafe(int motor_index);

private:
	static constexpr uint16_t DISARMED_VALUE = 0;

	struct Command {
		dshot_command_t command;
		int num_repetitions{0};
		uint8_t motor_mask{0xff};

		bool valid() const { return num_repetitions > 0; }
		void clear() { num_repetitions = 0; }
	};

	MixingOutput _mixing_output{DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	bool _waiting_for_esc_info{false};

	uORB::Subscription _param_sub{ORB_ID(parameter_update)};

	Command _current_command;
	px4::atomic<Command *> _new_command{nullptr};

	unsigned	_num_outputs{0};
	int		_class_instance{-1};

	bool		_outputs_on{false};
	uint32_t	_output_mask{0};
	bool		_outputs_initialized{false};

	perf_counter_t	_cycle_perf;

	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);
	void		update_dshot_out_state(bool on);

	void		update_params();

	VolzOutput(const VolzOutput &) = delete;
	VolzOutput operator=(const VolzOutput &) = delete;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VOLZ_CONFIG>) _param_dshot_config,
		(ParamFloat<px4::params::VOLZ_MIN>) _param_dshot_min
	)
};

VolzOutput::VolzOutput() :
	CDev("/dev/volz"),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_mixing_output.setAllDisarmedValues(DISARMED_VALUE);
	_mixing_output.setAllMinValues(DISARMED_VALUE + 1);
	_mixing_output.setAllMaxValues(DSHOT_MAX_THROTTLE);

}

VolzOutput::~VolzOutput()
{
	/* make sure outputs are off */
	up_dshot_arm(false);

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
}

int
VolzOutput::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return 0;
}

int
VolzOutput::task_spawn(int argc, char *argv[])
{
	VolzOutput *instance = new VolzOutput();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void
VolzOutput::update_dshot_out_state(bool on)
{
	if (on && !_outputs_initialized && _output_mask != 0) {
		DShotConfig config = (DShotConfig)_param_dshot_config.get();
		unsigned dshot_frequency;

		switch (config) {
		case DShotConfig::DShot150:
			dshot_frequency = DSHOT150;
			break;

		case DShotConfig::DShot300:
			dshot_frequency = DSHOT300;
			break;

		case DShotConfig::DShot1200:
			dshot_frequency = DSHOT1200;
			break;

		case DShotConfig::DShot600:
		default:
			dshot_frequency = DSHOT600;
			break;
		}

		int ret = up_dshot_init(_output_mask, dshot_frequency);

		if (ret != 0) {
			PX4_ERR("up_dshot_init failed (%i)", ret);
			return;
		}

		_outputs_initialized = true;
	}

	if (_outputs_initialized) {
		up_dshot_arm(on);
		_outputs_on = on;
	}
}

int VolzOutput::sendCommandThreadSafe(dshot_command_t command, int num_repetitions, int motor_index)
{
	Command cmd;
	cmd.command = command;

	if (motor_index == -1) {
		cmd.motor_mask = 0xff;

	} else {
		cmd.motor_mask = 1 << _mixing_output.reorderedMotorIndex(motor_index);
	}

	cmd.num_repetitions = num_repetitions;
	_new_command.store(&cmd);

	// wait until main thread processed it
	while (_new_command.load()) {
		px4_usleep(1000);
	}

	return 0;
}

void VolzOutput::mixerChanged()
{
}

bool VolzOutput::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			       unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!_outputs_on) {
		return false;
	}

	int requested_telemetry_index = -1;

	if (stop_motors) {

		// when motors are stopped we check if we have other commands to send
		for (int i = 0; i < (int)num_outputs; i++) {
			if (_current_command.valid() && (_current_command.motor_mask & (1 << i))) {
				// for some reason we need to always request telemetry when sending a command
				up_dshot_motor_command(i, _current_command.command, true);

			} else {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, i == requested_telemetry_index);
			}
		}

		if (_current_command.valid()) {
			--_current_command.num_repetitions;
		}

	} else {
		for (int i = 0; i < (int)num_outputs; i++) {
			if (outputs[i] == DISARMED_VALUE) {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, i == requested_telemetry_index);

			} else {
				up_dshot_motor_data_set(i, math::min(outputs[i], (uint16_t)DSHOT_MAX_THROTTLE), i == requested_telemetry_index);
			}
		}

		// clear commands when motors are running
		_current_command.clear();
	}

	if (stop_motors || num_control_groups_updated > 0) {
		up_dshot_trigger();
	}

	return true;
}

void
VolzOutput::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	/* update output status if armed or if mixer is loaded */
	bool outputs_on = _mixing_output.armed().armed || _mixing_output.mixers();

	if (_outputs_on != outputs_on) {
		update_dshot_out_state(outputs_on);
	}

	if (_param_sub.updated()) {
		update_params();
	}

	// new command?
	if (!_current_command.valid()) {
		Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;
			_new_command.store(nullptr);
		}
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void VolzOutput::update_params()
{
	parameter_update_s pupdate;
	_param_sub.update(&pupdate);

	updateParams();

	// we use a minimum value of 1, since 0 is for disarmed
	_mixing_output.setAllMinValues(math::constrain((int)(_param_dshot_min.get() * (float)DSHOT_MAX_THROTTLE),
				       DISARMED_VALUE + 1, DSHOT_MAX_THROTTLE));
}

int
VolzOutput::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("dshot ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		ret = -EINVAL;
		break;

	case PWM_SERVO_SET_COUNT:
		ret = -EINVAL;
		break;

	case PWM_SERVO_SET_MODE:
		break;

	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

int
VolzOutput::module_new_mode(PortMode new_mode)
{
	if (!is_running()) {
		return -1;
	}

	VolzOutput::Mode mode;

	VolzOutput *object = get_instance();

	return OK;
}

int VolzOutput::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	int motor_index = -1; // select motor index, default: -1=all
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			motor_index = strtol(myoptarg, nullptr, 10) - 1;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	struct Command {
		const char *name;
		dshot_command_t command;
		int num_repetitions;
	};

	constexpr Command commands[] = {
		{"reverse", DShot_cmd_spin_direction_reversed, 10},
		{"normal", DShot_cmd_spin_direction_normal, 10},
		{"save", DShot_cmd_save_settings, 10},
		{"3d_on", DShot_cmd_3d_mode_on, 10},
		{"3d_off", DShot_cmd_3d_mode_off, 10},
		{"beep1", DShot_cmd_beacon1, 1},
		{"beep2", DShot_cmd_beacon2, 1},
		{"beep3", DShot_cmd_beacon3, 1},
		{"beep4", DShot_cmd_beacon4, 1},
		{"beep5", DShot_cmd_beacon5, 1},
	};

	for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
		if (!strcmp(verb, commands[i].name)) {
			if (!is_running()) {
				PX4_ERR("module not running");
				return -1;
			}

			return get_instance()->sendCommandThreadSafe(commands[i].command, commands[i].num_repetitions, motor_index);
		}
	}

	if (!is_running()) {
		int ret = VolzOutput::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int VolzOutput::print_status()
{

	PX4_INFO("Outputs initialized: %s", _outputs_initialized ? "yes" : "no");
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	perf_print_counter(_cycle_perf);
	_mixing_output.printStatus();

	return 0;
}

int VolzOutput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the DShot output driver. It is similar to the fmu driver, and can be used as drop-in replacement
to use DShot as ESC communication protocol instead of PWM.

It supports:
- DShot150, DShot300, DShot600, DShot1200
- telemetry via separate UART and publishing as esc_status message
- sending DShot commands via CLI

### Examples
Permanently reverse motor 1:
$ dshot reverse -m 1
$ dshot save -m 1
After saving, the reversed direction will be regarded as the normal one. So to reverse again repeat the same commands.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");

	PRINT_MODULE_USAGE_PARAM_COMMENT("All of the mode_* commands will start the module if not running already");

	PRINT_MODULE_USAGE_COMMAND("mode_gpio");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode_pwm", "Select all available pins as PWM");
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8
  PRINT_MODULE_USAGE_COMMAND("mode_pwm8");
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6
  PRINT_MODULE_USAGE_COMMAND("mode_pwm6");
  PRINT_MODULE_USAGE_COMMAND("mode_pwm5");
  PRINT_MODULE_USAGE_COMMAND("mode_pwm5cap1");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm4");
  PRINT_MODULE_USAGE_COMMAND("mode_pwm4cap1");
  PRINT_MODULE_USAGE_COMMAND("mode_pwm4cap2");
  PRINT_MODULE_USAGE_COMMAND("mode_pwm3");
  PRINT_MODULE_USAGE_COMMAND("mode_pwm3cap1");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm2");
  PRINT_MODULE_USAGE_COMMAND("mode_pwm2cap2");
#endif
#if defined(BOARD_HAS_PWM)
  PRINT_MODULE_USAGE_COMMAND("mode_pwm1");
#endif

	// DShot commands
	PRINT_MODULE_USAGE_COMMAND_DESCR("reverse", "Reverse motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("normal", "Normal motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save current settings");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_on", "Enable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_off", "Disable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep1", "Send Beep pattern 1");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep2", "Send Beep pattern 2");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep3", "Send Beep pattern 3");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep4", "Send Beep pattern 4");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep5", "Send Beep pattern 5");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("esc_info", "Request ESC information");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based)", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dshot_main(int argc, char *argv[])
{
	return VolzOutput::main(argc, argv);
}
