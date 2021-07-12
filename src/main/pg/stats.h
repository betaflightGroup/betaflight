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
#include "drivers/io_types.h"

#define STATS_OFF (-1)

typedef struct statsConfig_s {
    uint32_t stats_total_flights;
    uint32_t stats_total_time_s;
    uint32_t stats_total_dist_m;
    int8_t stats_min_armed_time_s;
    uint32_t stats_total_packs;
} statsConfig_t;

PG_DECLARE(statsConfig_t, statsConfig);
