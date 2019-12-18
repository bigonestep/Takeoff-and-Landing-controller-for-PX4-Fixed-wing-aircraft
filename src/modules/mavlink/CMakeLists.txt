############################################################################
#
#   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(MAVLINK_INCLUDE_PATH)

if(config_mavlink_git_repo)
	include(FetchContent)
	FetchContent_Declare(
		mavlink
		GIT_REPOSITORY ${config_mavlink_git_repo}
		GIT_TAG        ${config_mavlink_git_tag}
	)

	FetchContent_GetProperties(mavlink)
	if(NOT mavlink_POPULATED)
		FetchContent_Populate(mavlink)
	endif()

	set(MAVLINK_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/mavlink)
	add_custom_command(
		OUTPUT ${MAVLINK_INCLUDE_PATH}/standard/mavlink.h
		COMMAND ${PYTHON_EXECUTABLE} ${mavlink_SOURCE_DIR}/pymavlink/tools/mavgen.py
			--output ${MAVLINK_INCLUDE_PATH}
			--lang C
			--wire-protocol 2.0
			${mavlink_SOURCE_DIR}/message_definitions/v1.0/standard.xml
	)
	add_custom_target(git_mavlink DEPENDS ${MAVLINK_INCLUDE_PATH}/standard/mavlink.h)

else()
	px4_add_git_submodule(TARGET git_mavlink PATH "${PX4_SOURCE_DIR}/mavlink/include/mavlink/v2.0")
	set(MAVLINK_INCLUDE_PATH "${PX4_SOURCE_DIR}/mavlink/include/mavlink/v2.0")
endif()

px4_add_module(
	MODULE modules__mavlink
	MAIN mavlink
	COMPILE_FLAGS
		-Wno-cast-align # TODO: fix and enable
		-Wno-address-of-packed-member # TODO: fix in c_library_v2
	INCLUDES
		${MAVLINK_INCLUDE_PATH}
	SRCS
		${MAVLINK_INCLUDE_PATH}/standard/mavlink.h

		mavlink.c
		mavlink_command_sender.cpp
		mavlink_ftp.cpp
		mavlink_high_latency2.cpp
		mavlink_log_handler.cpp
		mavlink_main.cpp
		mavlink_messages.cpp
		mavlink_mission.cpp
		mavlink_orb_subscription.cpp
		mavlink_parameters.cpp
		mavlink_rate_limiter.cpp
		mavlink_receiver.cpp
		mavlink_shell.cpp
		mavlink_simple_analyzer.cpp
		mavlink_stream.cpp
		mavlink_ulog.cpp
		mavlink_timesync.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		airspeed
		git_mavlink
		conversion
		git_ecl
		ecl_geo
		version
	UNITY_BUILD
	)

if(PX4_TESTING)
	add_subdirectory(mavlink_tests)
endif()
