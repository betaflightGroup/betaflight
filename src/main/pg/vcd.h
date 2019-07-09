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

#include "pg/pg.h"

// Video Character Display parameters

enum VIDEO_SYSTEMS {
    VIDEO_SYSTEM_AUTO = 0,
    VIDEO_SYSTEM_PAL,
    VIDEO_SYSTEM_NTSC
};

enum VIDEO_SYSTEM_EXTENDEDS {
    VIDEO_SYSTEM_EXTENDED_AUTO = 0,
    VIDEO_SYSTEM_EXTENDED_ON,
    VIDEO_SYSTEM_EXTENDED_OFF
};

typedef struct vcdProfile_s {
    uint8_t video_system;
    int8_t h_offset;
    int8_t v_offset;
#ifdef USE_MAX7456_EXTENDED
    uint8_t video_system_extended;
#endif
} vcdProfile_t;

PG_DECLARE(vcdProfile_t, vcdProfile);
