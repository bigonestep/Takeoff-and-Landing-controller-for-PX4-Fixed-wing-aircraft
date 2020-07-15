
# Excelsior is the code name of a board currently in development.
#
# This cmake config builds for QURT which is the operating system running on
# the DSP side.

set(QC_SOC_TARGET "APQ8096")

include(px4_git)
px4_add_git_submodule(TARGET git_cmake_hexagon PATH "${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon")
list(APPEND CMAKE_MODULE_PATH "${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon")

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "Enviroment variable HEXAGON_SDK_ROOT must be set")
else()
	set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
endif()

include(toolchain/Toolchain-qurt)
include(qurt_flags)
include_directories(${HEXAGON_SDK_INCLUDES})

set(CONFIG_SHMEM "1")
add_definitions(-DORB_COMMUNICATOR)

# Disable the creation of the parameters.xml file by scanning individual
# source files, and scan all source files.  This will create a parameters.xml
# file that contains all possible parameters, even if the associated module
# is not used.  This is necessary for parameter synchronization between the
# ARM and DSP processors.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

# add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>)


# This definition allows to differentiate the specific board.
add_definitions(
	-D__PX4_QURT_EXCELSIOR
	-DDEBUG_BUILD
	)

px4_add_board(
	PLATFORM qurt
	VENDOR atlflight
	MODEL excelsior
	LABEL qurt
	DRIVERS
		barometer/bmp280
		gps
		imu/invensense/mpu9250
		magnetometer/hmc5883
		qshell/qurt
		snapdragon_pwm_out
		spektrum_rc
	MODULES
		airspeed_selector
		attitude_estimator_q
		commander
		ekf2
		fw_att_control
		fw_pos_control_l1
		land_detector
		landing_target_estimator
		local_position_estimator
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		muorb/adsp
		rc_update
		rover_pos_control
		sensors
		temperature_compensation
		vmount
		vtol_att_control
	SYSTEMCMDS
		led_control
		mixer
		#motor_ramp
		motor_test
		param
		perf
		#pwm
		#topic_listener
		ver
		work_queue
	)
