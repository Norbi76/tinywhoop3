#pragma once

// pmw3901_init_registers.h - PixArt's power-up register sequence for the PMW3901.
//
// WHAT THIS IS
//   The ~73 register writes PixArt require after power-up to configure the sensor's analogue
//   front end and frame processing. Without them the part answers its product-ID check and then
//   reports no useful motion.
//
// WHERE IT CAME FROM - THIRD PARTY, DO NOT "TIDY" THE VALUES
//   Transcribed verbatim from Bitcraze's Arduino driver, which is the reference implementation
//   everyone uses:
//       https://github.com/bitcraze/Bitcraze_PMW3901
//       src/Bitcraze_PMW3901.cpp -> Bitcraze_PMW3901::initRegisters()
//       fetched from master, 2026-08-10
//
//   These are undocumented magic numbers from a PixArt application note. They cannot be derived,
//   and a "plausible" edit produces a sensor that returns convincing garbage rather than an error.
//   The only correct way to change this table is to re-transcribe it from upstream.
//
//   Register 0x7F is a BANK/PAGE SELECT: it switches which register page the following writes
//   land on. That is why the same addresses recur with different values, and why the order is
//   load-bearing. The bank comments below are ours, added for navigation only - Bitcraze's
//   source is a single flat list.
//
// LICENCE (MIT) - reproduced as the licence requires:
//
//   PMW3901 Arduino driver
//   Copyright (c) 2017 Bitcraze AB
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy
//   of this software and associated documentation files (the "Software"), to
//   deal in the Software without restriction, including without limitation the
//   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//   sell copies of the Software, and to permit persons to whom the Software is
//   furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in
//   all copies or substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//   SOFTWARE.

// SINGLE INCLUDER BY DESIGN
//   These are `static const` tables defined in a header. That is safe here because exactly one
//   translation unit - pmw3901_driver.c - includes this file. Do not include it anywhere else:
//   every additional includer gets its own private copy of both arrays.

#include <stdint.h>

typedef struct {
    uint8_t reg;
    uint8_t value;
} pmw3901_reg_write_t;

// Part 1: everything before PixArt's mandatory 100 ms pause.
static const pmw3901_reg_write_t pmw3901_init_part1[] = {
    { 0x7F, 0x00 },
    { 0x61, 0xAD },
    { 0x7F, 0x03 },
    { 0x40, 0x00 },
    { 0x7F, 0x05 },
    { 0x41, 0xB3 },
    { 0x43, 0xF1 },
    { 0x45, 0x14 },
    { 0x5B, 0x32 },
    { 0x5F, 0x34 },
    { 0x7B, 0x08 },
    { 0x7F, 0x06 },
    { 0x44, 0x1B },
    { 0x40, 0xBF },
    { 0x4E, 0x3F },
    { 0x7F, 0x08 },
    { 0x65, 0x20 },
    { 0x6A, 0x18 },
    { 0x7F, 0x09 },
    { 0x4F, 0xAF },
    { 0x5F, 0x40 },
    { 0x48, 0x80 },
    { 0x49, 0x80 },
    { 0x57, 0x77 },
    { 0x60, 0x78 },
    { 0x61, 0x78 },
    { 0x62, 0x08 },
    { 0x63, 0x50 },
    { 0x7F, 0x0A },
    { 0x45, 0x60 },
    { 0x7F, 0x00 },
    { 0x4D, 0x11 },
    { 0x55, 0x80 },
    { 0x74, 0x1F },
    { 0x75, 0x1F },
    { 0x4A, 0x78 },
    { 0x4B, 0x78 },
    { 0x44, 0x08 },
    { 0x45, 0x50 },
    { 0x64, 0xFF },
    { 0x65, 0x1F },
    { 0x7F, 0x14 },
    { 0x65, 0x60 },
    { 0x66, 0x08 },
    { 0x63, 0x78 },
    { 0x7F, 0x15 },
    { 0x48, 0x58 },
    { 0x7F, 0x07 },
    { 0x41, 0x0D },
    { 0x43, 0x14 },
    { 0x4B, 0x0E },
    { 0x45, 0x0F },
    { 0x44, 0x42 },
    { 0x4C, 0x80 },
    { 0x7F, 0x10 },
    { 0x5B, 0x02 },
    { 0x7F, 0x07 },
    { 0x40, 0x41 },
    { 0x70, 0x00 },
};

// PixArt require a settling pause here, between the two halves of the sequence.
// Bitcraze use delay(100). This is NOT optional and NOT a Bitcraze embellishment.
#define PMW3901_INIT_PAUSE_MS 100

// Part 2: everything after the pause.
static const pmw3901_reg_write_t pmw3901_init_part2[] = {
    { 0x32, 0x44 },
    { 0x7F, 0x07 },
    { 0x40, 0x40 },
    { 0x7F, 0x06 },
    { 0x62, 0xF0 },
    { 0x63, 0x00 },
    { 0x7F, 0x0D },
    { 0x48, 0xC0 },
    { 0x6F, 0xD5 },
    { 0x7F, 0x00 },
    { 0x5B, 0xA0 },
    { 0x4E, 0xA8 },
    { 0x5A, 0x50 },
    { 0x40, 0x80 },
};
