/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

/**
 * @file PixArt_PAW3902JF_registers.hpp
 *
 * PixArt PAW3902JF registers
 *
 */

#pragma once

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);


namespace PixArt_PAW3902JF
{

static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000; // 2 MHz SPI clock frequency

static constexpr uint8_t DIR_WRITE = Bit7;

static constexpr uint8_t PRODUCT_ID = 0x49; // shared with PMW3901/PAW3902/PAW3903
static constexpr uint8_t REVISION_ID = 0x01;
static constexpr uint8_t INVERSE_PRODUCT_ID = 0xB6;

enum Register : uint8_t {
	Product_ID         = 0x00,
	Revision_ID        = 0x01,

	Motion_Burst       = 0x16,

	Power_Up_Reset     = 0x3A,

	Resolution         = 0x4E,

	Inverse_Product_ID = 0x5F
};

enum class Mode {
	Bright        = 0,
	LowLight      = 1,
	SuperLowLight = 2,
};

enum class Interval {
	Bright        = 1000000 / 126, // Mode 0: Bright (126 fps) 60 Lux
	LowLight      = 1000000 / 126, // Mode 1: Low Light (126 fps) 30 Lux
	SuperLowLight = 1000000 / 50,  // Mode 2: Super Low Light (50 fps) 9 Lux
};

}
