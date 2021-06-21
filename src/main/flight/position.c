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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 1);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altSource = DEFAULT,
);

static int32_t estimatedAltitudeCm = 0;                // in cm

#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#ifdef USE_VARIO
static int16_t estimatedVario = 0;                   // in cm/s

int16_t calculateEstimatedVario(int32_t baroAlt, const uint32_t dTime) {
    static float vel = 0;
    static int32_t lastBaroAlt = 0;

    int32_t baroVel = 0;

    baroVel = (baroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = baroAlt;

    baroVel = constrain(baroVel, -1500.0f, 1500.0f);
    baroVel = applyDeadband(baroVel, 10.0f);

    vel = vel * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel) + baroVel * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel));
    int32_t vel_tmp = lrintf(vel);
    vel_tmp = applyDeadband(vel_tmp, 5.0f);

    return constrain(vel_tmp, SHRT_MIN, SHRT_MAX);
}
#endif

#if defined(USE_BARO) || defined(USE_GPS)
static bool altitudeOffsetSet = false;

void calculateEstimatedAltitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUs = 0;
    static int32_t baroAltOffset = 0;
    static int32_t gpsAltOffset = 0;

    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ) {
        ignoreTaskShortExecTime();
        return;
    }
    previousTimeUs = currentTimeUs;

    int32_t baroAlt = 0;
    int32_t gpsAlt = 0;

#if defined(USE_GPS) && defined(USE_VARIO)
    int16_t gpsVertSpeed = 0;
#endif
    float gpsTrust = 0.3; //conservative default
    bool haveBaroAlt = false;
    bool haveGpsAlt = false;
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (!baroIsCalibrationComplete()) {
            performBaroCalibrationCycle();
        } else {
            baroAlt = baroCalculateAltitude();
            haveBaroAlt = true;
        }
    }
#endif

#ifdef USE_GPS
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
        gpsAlt = gpsSol.llh.altCm;
#ifdef USE_VARIO
        gpsVertSpeed = GPS_verticalSpeedInCmS;
#endif
        haveGpsAlt = true;

        if (gpsSol.hdop != 0) {
            gpsTrust = 100.0 / gpsSol.hdop;
        }
        // always use at least 10% of other sources besides gps if available
        gpsTrust = MIN(gpsTrust, 0.9f);
    }
#endif

    if (ARMING_FLAG(ARMED) && !altitudeOffsetSet) {
        baroAltOffset = baroAlt;
        gpsAltOffset = gpsAlt;
        altitudeOffsetSet = true;
    } else if (!ARMING_FLAG(ARMED) && altitudeOffsetSet) {
        altitudeOffsetSet = false;
    }
    baroAlt -= baroAltOffset;
    gpsAlt -= gpsAltOffset;


    if (haveGpsAlt && haveBaroAlt && positionConfig()->altSource == DEFAULT) {
        if (ARMING_FLAG(ARMED)) {
            estimatedAltitudeCm = gpsAlt * gpsTrust + baroAlt * (1 - gpsTrust);
        } else {
            estimatedAltitudeCm = gpsAlt; //absolute altitude is shown before arming, ignore baro
        }
#ifdef USE_VARIO
        // baro is a better source for vario, so ignore gpsVertSpeed
        estimatedVario = calculateEstimatedVario(baroAlt, dTime);
#endif
    } else if (haveGpsAlt && (positionConfig()->altSource == GPS_ONLY || positionConfig()->altSource == DEFAULT )) {
        estimatedAltitudeCm = gpsAlt;
#if defined(USE_VARIO) && defined(USE_GPS)
        estimatedVario = gpsVertSpeed;
#endif
    } else if (haveBaroAlt && (positionConfig()->altSource == BARO_ONLY || positionConfig()->altSource == DEFAULT)) {
        estimatedAltitudeCm = baroAlt;
#ifdef USE_VARIO
        estimatedVario = calculateEstimatedVario(baroAlt, dTime);
#endif
    }

    

    DEBUG_SET(DEBUG_ALTITUDE, 0, (int32_t)(100 * gpsTrust));
    DEBUG_SET(DEBUG_ALTITUDE, 1, baroAlt);
    DEBUG_SET(DEBUG_ALTITUDE, 2, gpsAlt);
#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedVario);
#endif
}

bool isAltitudeOffset(void)
{
    return altitudeOffsetSet;
}
#endif

int32_t getEstimatedAltitudeCm(void)
{
    return estimatedAltitudeCm;
}

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
