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

if(NOT PX4_BOARD)
	message(FATAL_ERROR "PX4_BOARD must be set (eg px4_fmu-v2)")
endif()

if(NOT PX4_BINARY_DIR)
	message(FATAL_ERROR "PX4_BINARY_DIR must be set")
endif()

if(NOT PX4_BOARD_DIR)
	message(FATAL_ERROR "PX4_BOARD_DIR must be set")
endif()

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config CACHE FILEPATH "PX4 NuttX config" FORCE)

set(NUTTX_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../NuttX)
set(NUTTX_DIR ${PX4_BINARY_DIR}/NuttX/nuttx CACHE FILEPATH "NuttX directory" FORCE)
set(NUTTX_APPS_DIR ${PX4_BINARY_DIR}/NuttX/apps CACHE FILEPATH "NuttX apps directory" FORCE)

px4_add_git_submodule(TARGET git_nuttx PATH "${NUTTX_SRC_DIR}/nuttx")
px4_add_git_submodule(TARGET git_nuttx_apps PATH "${NUTTX_SRC_DIR}/apps")

###############################################################################
# nuttx-config: copy to build directory
###############################################################################

set(NUTTX_DEFCONFIG ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig CACHE FILEPATH "path to defconfig" FORCE)

# If the board provides a Kconfig Use it or create an empty one
if(EXISTS ${NUTTX_CONFIG_DIR}/Kconfig)
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_CONFIG_DIR}/Kconfig ${NUTTX_DIR}/configs/dummy/Kconfig)
else()
	execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_DIR}/configs/dummy/Kconfig)
endif()

execute_process(
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/src
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_CONFIG_DIR}/ ${PX4_BINARY_DIR}/NuttX/nuttx-config
)

# NuttX extra files
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/math.h ${NUTTX_DIR}/arch/arm/include/math.h) # copy arm math.h into NuttX source
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/nsh_romfsimg.h ${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h)

# copy defconfig
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config)
