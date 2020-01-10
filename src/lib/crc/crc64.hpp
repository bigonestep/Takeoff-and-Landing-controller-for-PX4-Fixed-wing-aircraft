/************************************************************************************************
 * libc/misc/lib_crc32.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *
 * The logic in this file was developed by Gary S. Brown:
 *
 *   COPYRIGHT (C) 1986 Gary S. Brown.  You may use this program, or code or tables
 *   extracted from it, as desired without restriction.
 *
 * First, the polynomial itself and its table of feedback terms.  The polynomial is:
 *
 *    X^32+X^26+X^23+X^22+X^16+X^12+X^11+X^10+X^8+X^7+X^5+X^4+X^2+X^1+X^0
 *
 * Note that we take it "backwards" and put the highest-order term in the lowest-order bit.
 * The X^32 term is "implied"; the LSB is the X^31 term, etc.  The X^0 term (usually shown
 * as "+1") results in the MSB being 1
 *
 * Note that the usual hardware shift register implementation, which is what we're using
 * (we're merely optimizing it by doing eight-bit chunks at a time) shifts bits into the
 * lowest-order term.  In our implementation, that means shifting towards the right.  Why
 * do we do it this way?  Because the calculated CRC must be transmitted in order from
 * highest-order term to lowest-order term.  UARTs transmit characters in order from LSB
 * to MSB.  By storing the CRC this way we hand it to the UART in the order low-byte to
 * high-byte; the UART sends each low-bit to hight-bit; and the result is transmission bit
 * by bit from highest- to lowest-order term without requiring any bit shuffling on our
 * part.  Reception works similarly
 *
 * The feedback terms table consists of 256, 32-bit entries.  Notes
 *
 * - The table can be generated at runtime if desired; code to do so is shown later.  It
 *   might not be obvious, but the feedback terms simply represent the results of eight
 *   shift/xor operations for all combinations of data and CRC register values
 *
 * - The values must be right-shifted by eight bits by the updcrc logic; the shift must
 *   be u_(bring in zeroes).  On some hardware you could probably optimize the shift in
 *   assembler by using byte-swap instructions polynomial $edb88320
  ************************************************************************************************/

#pragma once

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Name: crc64_add_word
 *
 * Description:
 *   Calculates a CRC-64-WE using the polynomial of 0x42F0E1EBA9EA3693
 *   See http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
 *   Check: 0x62EC59E3F1A4F00A
 *
 * Input Parameters:
 *    crc   - The running total of the crc 64
 *    value - The value to add
 *
 * Returned Value:
 *   The current crc64 with the value processed.
 *
 ****************************************************************************/
static constexpr uint64_t CRC64_INITIAL = 0xFFFFFFFFFFFFFFFFull;
static constexpr uint64_t CRC64_OUTPUT_XOR = 0xFFFFFFFFFFFFFFFFull;

uint64_t crc64_add_word(uint64_t crc, uint32_t value);
