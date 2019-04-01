############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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


# TODO:
#  NuttX cmake support custom boards
#   library aliases? nuttx::arch, etc
#   apps handle builtin commands
#

add_subdirectory(nuttx)

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config)

# If the board provides a Kconfig Use it or create an empty one
if(EXISTS ${NUTTX_CONFIG_DIR}/Kconfig)
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/configs/dummy/Kconfig
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/Kconfig ${NUTTX_DIR}/configs/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		DEPENDS
		)
else()
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/configs/dummy/Kconfig
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_DIR}/configs/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		DEPENDS

		)
endif()

###############################################################################
# NuttX configure
###############################################################################

# copy NuttX config directory
add_custom_command(
	OUTPUT
		${NUTTX_DIR}/arch/arm/include/math.h
		${PX4_BINARY_DIR}/NuttX/nuttx-config/include/board.h
		${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h
		${PX4_BINARY_DIR}/NuttX/nuttx-config/scripts/script.ld
		${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/include/nuttx/config.h
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_CONFIG_DIR}/ ${PX4_BINARY_DIR}/NuttX/nuttx-config
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/src
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/math.h ${NUTTX_DIR}/arch/arm/include/math.h # copy arm math.h into NuttX source
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/nsh_romfsimg.h ${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h
	COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
	DEPENDS
		${NUTTX_SRC_DIR}/math.h
		${NUTTX_SRC_DIR}/nsh_romfsimg.h
		${NUTTX_CONFIG_DIR}/include/board.h
		${NUTTX_CONFIG_DIR}/scripts/script.ld
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Copying NuttX config ${NUTTX_CONFIG}"
)

# NuttX defconfig
#  cmake should trigger reconfigure if defconfig changes
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${NUTTX_DEFCONFIG})
file(STRINGS ${NUTTX_DEFCONFIG} config_expanded REGEX "# Automatically generated file; DO NOT EDIT.")
if (NOT config_expanded)
	# copy compressed PX4 defconfig into nuttx and inflate
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/.config
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/px4_nuttx_make_olddefconfig.sh ${NUTTX_DIR}
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		DEPENDS
			${NUTTX_DEFCONFIG}
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		WORKING_DIRECTORY ${NUTTX_DIR}
		COMMENT "Copying NuttX compressed config ${NUTTX_CONFIG} and inflating"
	)
else()
	# copy uncompressed PX4 defconfig into nuttx
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/.config
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		DEPENDS
			${NUTTX_DEFCONFIG}
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		WORKING_DIRECTORY ${NUTTX_DIR}
		COMMENT "Copying NuttX uncompressed config ${NUTTX_CONFIG}"
	)
endif()
