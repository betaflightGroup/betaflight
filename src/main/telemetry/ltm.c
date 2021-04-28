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

/*
 * LightTelemetry from KipK
 *
 * Minimal one way telemetry protocol for really bitrates (1200/2400 bauds).
 * Effective for ground OSD, groundstation HUD and Antenna tracker
 * http://www.wedontneednasa.com/2014/02/lighttelemetry-v2-en-route-to-ground-osd.html
 *
 * This implementation is for LTM v2 > 2400 baud rates
 *
 * Cleanflight implementation by Jonathan Hudson
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY_LTM

#include "build/build_config.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/beeper.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "telemetry/telemetry.h"
#include "telemetry/ltm.h"


#define TELEMETRY_LTM_INITIAL_PORT_MODE MODE_TX
#define LTM_CYCLETIME   100

static serialPort_t *ltmPort;
static const serialPortConfig_t *portConfig;
static bool ltmEnabled;
static portSharing_e ltmPortSharing;
static uint8_t ltm_crc;

static void ltm_initialise_packet(uint8_t ltm_id)
{
    ltm_crc = 0;
    serialWrite(ltmPort, '$');
    serialWrite(ltmPort, 'T');
    serialWrite(ltmPort, ltm_id);
}

static void ltm_serialise_8(uint8_t v)
{
    serialWrite(ltmPort, v);
    ltm_crc ^= v;
}

static void ltm_serialise_16(uint16_t v)
{
    ltm_serialise_8((uint8_t)v);
    ltm_serialise_8((v >> 8));
}

static void ltm_serialise_32(uint32_t v)
{
    ltm_serialise_8((uint8_t)v);
    ltm_serialise_8((v >> 8));
    ltm_serialise_8((v >> 16));
    ltm_serialise_8((v >> 24));
}

static void ltm_finalise(void)
{
    serialWrite(ltmPort, ltm_crc);
}

/*
 * GPS G-frame 5Hhz at > 2400 baud
 * LAT LON SPD ALT SAT/FIX
 */
static void ltm_gframe(void)
{
#if defined(USE_GPS)
    uint8_t gps_fix_type = 0;
    int32_t ltm_alt;

    if (!sensors(SENSOR_GPS))
        return;

    if (!STATE(GPS_FIX))
        gps_fix_type = 1;
    else if (gpsSol.numSat < 5)
        gps_fix_type = 2;
    else
        gps_fix_type = 3;

    ltm_initialise_packet('G');
    ltm_serialise_32(gpsSol.llh.lat);
    ltm_serialise_32(gpsSol.llh.lon);
    ltm_serialise_8((uint8_t)(gpsSol.groundSpeed / 100));
    ltm_alt = getEstimatedAltitudeCm();
    ltm_serialise_32(ltm_alt);
    ltm_serialise_8((gpsSol.numSat << 2) | gps_fix_type);
    ltm_finalise();
#endif
}

/*
 * Sensors S-frame 5Hhz at > 2400 baud
 * VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD
 * Flight mode(0-19):
 *     0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon,
 *     4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
 *     8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints,
 *     11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe,
 *     15: LAND, 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
 */

static void ltm_sframe(void)
{
    uint8_t lt_flightmode;
    uint8_t lt_statemode;
    if (FLIGHT_MODE(PASSTHRU_MODE))
        lt_flightmode = 0;
    else if (FLIGHT_MODE(HEADFREE_MODE))
        lt_flightmode = 4;
    else if (FLIGHT_MODE(ANGLE_MODE))
        lt_flightmode = 2;
    else if (FLIGHT_MODE(HORIZON_MODE))
        lt_flightmode = 3;
    else
        lt_flightmode = 1;      // Rate mode

    lt_statemode = (ARMING_FLAG(ARMED)) ? 1 : 0;
    if (failsafeIsActive())
        lt_statemode |= 2;
    ltm_initialise_packet('S');
    ltm_serialise_16(getBatteryVoltage() * 10); // vbat converted to mV
    ltm_serialise_16((uint16_t)constrain(getMAhDrawn(), 0, UINT16_MAX)); // consumption in mAh (65535 mAh max)
    ltm_serialise_8(constrain(scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 255), 0, 255));        // scaled RSSI (uchar)
    ltm_serialise_8(0);              // no airspeed
    ltm_serialise_8((lt_flightmode << 2) | lt_statemode);
    ltm_finalise();
}

/*
 * Attitude A-frame - 10 Hz at > 2400 baud
 *  PITCH ROLL HEADING
 */
static void ltm_aframe(void)
{
    ltm_initialise_packet('A');
    ltm_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.pitch));
    ltm_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.roll));
    ltm_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    ltm_finalise();
}

/*
 * OSD additional data frame, 1 Hz rate
 *  This frame will be ignored by Ghettostation, but processed by GhettOSD if it is used as standalone onboard OSD
 *  home pos, home alt, direction to home
 */
static void ltm_oframe(void)
{
    ltm_initialise_packet('O');
#if defined(USE_GPS)
    ltm_serialise_32(GPS_home[GPS_LATITUDE]);
    ltm_serialise_32(GPS_home[GPS_LONGITUDE]);
#else
    ltm_serialise_32(0);
    ltm_serialise_32(0);
#endif
    ltm_serialise_32(0);                // Don't have GPS home altitude
    ltm_serialise_8(1);                 // OSD always ON
    ltm_serialise_8(STATE(GPS_FIX_HOME) ? 1 : 0);
    ltm_finalise();
}

static void process_ltm(void)
{
    static uint8_t ltm_scheduler;
    ltm_aframe();
    if (ltm_scheduler & 1)
        ltm_gframe();
    else
        ltm_sframe();
    if (ltm_scheduler == 0)
        ltm_oframe();
    ltm_scheduler++;
    ltm_scheduler %= 10;
}

void handleLtmTelemetry(void)
{
    static uint32_t ltm_lastCycleTime;
    uint32_t now;
    if (!ltmEnabled)
        return;
    if (!ltmPort)
        return;
    now = millis();
    if ((now - ltm_lastCycleTime) >= LTM_CYCLETIME) {
        process_ltm();
        ltm_lastCycleTime = now;
    }
}

void freeLtmTelemetryPort(void)
{
    closeSerialPort(ltmPort);
    ltmPort = NULL;
    ltmEnabled = false;
}

void initLtmTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_LTM);
    ltmPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_LTM);
}

void configureLtmTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_19200;
    }
    ltmPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_LTM, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_LTM_INITIAL_PORT_MODE, telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
    if (!ltmPort)
        return;
    ltmEnabled = true;
}

void checkLtmTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!ltmEnabled && telemetrySharedPort != NULL) {
            ltmPort = telemetrySharedPort;
            ltmEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(ltmPortSharing);
        if (newTelemetryEnabledValue == ltmEnabled)
            return;
        if (newTelemetryEnabledValue)
            configureLtmTelemetryPort();
        else
            freeLtmTelemetryPort();
    }
}
#endif
