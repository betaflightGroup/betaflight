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

#include "drivers/time.h"

#include "config/config.h"
#include "fc/dispatch.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "pg/stats.h"


#define STATS_SAVE_DELAY_US 500000 // Let disarming complete and save stats after this time

static timeMs_t arm_millis;
static uint32_t arm_distance_cm;

static bool statsNotSavedYet = true;

static bool saveRequired = false;

#ifdef USE_GPS
    #define DISTANCE_FLOWN_CM (GPS_distanceFlownInCm)
#else
    #define DISTANCE_FLOWN_CM (0)
#endif

void statsInit(void)
{
    dispatchEnable();
}

void writeStats(struct dispatchEntry_s* self)
{
    UNUSED(self);

    if (!ARMING_FLAG(ARMED)) {
        // Don't save if the user made config changes that have not yet been saved.
        if (!isConfigDirty()) {
            writeEEPROM();

            // Repeat disarming beep indicating the stats save is complete
            beeper(BEEPER_DISARMING);
        }

        saveRequired = false;
    }
}

dispatchEntry_t writeStatsEntry =
{
    writeStats, 0, NULL, false
};


void statsOnArm(void)
{
    arm_millis      = millis();
    arm_distance_cm = DISTANCE_FLOWN_CM;
}

void statsOnDisarm(void)
{
    int8_t minArmedTimeS = statsConfig()->stats_min_armed_time_s;
    if (minArmedTimeS >= 0) {
        uint32_t dtS = (millis() - arm_millis) / 1000;
        if (dtS >= (uint8_t)minArmedTimeS) {
            statsConfigMutable()->stats_total_flights += 1;    // arm / flight counter
            statsConfigMutable()->stats_total_time_s += dtS;
            statsConfigMutable()->stats_total_dist_m += (DISTANCE_FLOWN_CM - arm_distance_cm) / 100;
                if (statsNotSavedYet) {
                  statsConfigMutable()->stats_total_packs += 1;    // packs counter
                  statsNotSavedYet = false;
                }

            saveRequired = true;
        }

        if (saveRequired) {
            /* signal that stats need to be saved but don't execute time consuming flash operation
               now - let the disarming process complete and then execute the actual save */
            dispatchAdd(&writeStatsEntry, STATS_SAVE_DELAY_US);
        }
    }
}
#endif
