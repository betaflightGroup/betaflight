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

#include "platform.h"

#ifdef USE_PERSISTENT_STATS

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "stats.h"

PG_REGISTER_WITH_RESET_TEMPLATE(statsConfig_t, statsConfig, PG_STATS_CONFIG, 3);

PG_RESET_TEMPLATE(statsConfig_t, statsConfig,
    .stats_min_armed_time_s = STATS_OFF,
    .stats_total_flights = 0,
    .stats_total_time_s = 0,
    .stats_total_dist_m = 0,
    .stats_total_packs = 0,
);
#endif
