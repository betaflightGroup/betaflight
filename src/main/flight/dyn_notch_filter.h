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

#include <stdint.h>

#include "common/axis.h"
#include "pg/pg.h"

#define DYN_NOTCH_COUNT_MAX 5

typedef struct dynNotchConfig_s
{
    uint16_t dyn_notch_min_hz;
    uint16_t dyn_notch_max_hz;
    uint16_t dyn_notch_q;
    uint8_t  dyn_notch_count;

} dynNotchConfig_t;

PG_DECLARE(dynNotchConfig_t, dynNotchConfig);

void dynNotchInit(const dynNotchConfig_t *config, const uint32_t targetLooptimeUs);
void dynNotchPush(const int axis, const float sample);
void dynNotchUpdate(void);
float dynNotchFilter(const int axis, float value);
bool isDynamicFilterActive(void);
uint16_t getMaxFFT(void);
void resetMaxFFT(void);
