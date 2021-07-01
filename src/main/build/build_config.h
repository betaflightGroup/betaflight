/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef UNIT_TEST
// make these visible to unit test
#define STATIC_UNIT_TESTED
#define STATIC_INLINE_UNIT_TESTED
#define INLINE_UNIT_TESTED
#define UNIT_TESTED
#else
#define STATIC_UNIT_TESTED static
#define STATIC_INLINE_UNIT_TESTED static inline
#define INLINE_UNIT_TESTED inline
#define UNIT_TESTED
#endif

#ifndef __CC_ARM
#define REQUIRE_CC_ARM_PRINTF_SUPPORT
#define REQUIRE_PRINTF_LONG_SUPPORT
#endif

// MCU type IDs.
// IDs are permanent as they have a dependency to configurator through MSP reporting

typedef enum {
    MCU_TYPE_SIMULATOR = 0,
    MCU_TYPE_F103,
    MCU_TYPE_F303,
    MCU_TYPE_F40X,
    MCU_TYPE_F411,
    MCU_TYPE_F446,
    MCU_TYPE_F722,
    MCU_TYPE_F745,
    MCU_TYPE_F746,
    MCU_TYPE_F765,
    MCU_TYPE_H750,
    MCU_TYPE_H743_REV_UNKNOWN,
    MCU_TYPE_H743_REV_Y,
    MCU_TYPE_H743_REV_X,
    MCU_TYPE_H743_REV_V,
    MCU_TYPE_H7A3,
    MCU_TYPE_H723_725,
    MCU_TYPE_G474,
    MCU_TYPE_H730,
    MCU_TYPE_UNKNOWN = 255,
} mcuTypeId_e;

mcuTypeId_e getMcuTypeId(void);
