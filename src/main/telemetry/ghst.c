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
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY_GHST

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/version.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "cms/cms.h"

#include "drivers/nvic.h"

#include "config/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"

#include "io/gps.h"
#include "io/serial.h"

#include "rx/ghst.h"
#include "rx/ghst_protocol.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"
#include "telemetry/msp_shared.h"

#include "telemetry/ghst.h"

#define GHST_CYCLETIME_US                   100000      // 10x/sec
#define GHST_FRAME_PACK_PAYLOAD_SIZE        10
#define GHST_FRAME_GPS_PAYLOAD_SIZE         10
#define GHST_FRAME_LENGTH_CRC               1
#define GHST_FRAME_LENGTH_TYPE              1

static bool ghstTelemetryEnabled;
static uint8_t ghstFrame[GHST_FRAME_SIZE_MAX];

static void ghstInitializeFrame(sbuf_t *dst)
{
    dst->ptr = ghstFrame;
    dst->end = ARRAYEND(ghstFrame);

    sbufWriteU8(dst, GHST_ADDR_RX);
}

static void ghstFinalize(sbuf_t *dst)
{
    crc8_dvb_s2_sbuf_append(dst, &ghstFrame[2]); // start at byte 2, since CRC does not include device address and frame length
    sbufSwitchToReader(dst, ghstFrame);
    // write the telemetry frame to the receiver.
    ghstRxWriteTelemetryData(sbufPtr(dst), sbufBytesRemaining(dst));
}

// Battery (Pack) status
void ghstFramePackTelemetry(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, GHST_FRAME_PACK_PAYLOAD_SIZE + GHST_FRAME_LENGTH_CRC + GHST_FRAME_LENGTH_TYPE);
    sbufWriteU8(dst, 0x23);                     // GHST_DL_PACK_STAT

    if (telemetryConfig()->report_cell_voltage) {
        sbufWriteU16(dst, getBatteryAverageCellVoltage());      // units of 10mV
    } else {
        sbufWriteU16(dst, getBatteryVoltage());
    }
    sbufWriteU16(dst, getAmperage());                           // units of 10mA

    sbufWriteU16(dst, getMAhDrawn() / 10);                      // units of 10mAh (range of 0-655.36Ah)

    sbufWriteU8(dst, 0x00);                     // Rx Voltage, units of 100mV (not passed from BF, added in Ghost Rx)

    sbufWriteU8(dst, 0x00);                     // tbd1
    sbufWriteU8(dst, 0x00);                     // tbd2
    sbufWriteU8(dst, 0x00);                     // tbd3
}

// GPS data, primary, positional data
void ghstFrameGpsPrimaryTelemetry(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, GHST_FRAME_GPS_PAYLOAD_SIZE + GHST_FRAME_LENGTH_CRC + GHST_FRAME_LENGTH_TYPE);
    sbufWriteU8(dst, GHST_DL_GPS_PRIMARY);

    sbufWriteU32(dst, gpsSol.llh.lat);
    sbufWriteU32(dst, gpsSol.llh.lon);

    int32_t altitudeCm = gpsSol.llh.altCm;      // gps Altitude (absolute)
    if (!STATE(GPS_FIX)) {
        altitudeCm = 0;
    }

    const int16_t altitude = altitudeCm / 100;
    sbufWriteU16(dst, altitude);
}

// GPS data, secondary, auxiliary data
void ghstFrameGpsSecondaryTelemetry(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, GHST_FRAME_GPS_PAYLOAD_SIZE + GHST_FRAME_LENGTH_CRC + GHST_FRAME_LENGTH_TYPE);
    sbufWriteU8(dst, GHST_DL_GPS_SECONDARY);

    sbufWriteU16(dst, gpsSol.groundSpeed);      // speed in 0.1m/s
    sbufWriteU16(dst, gpsSol.groundCourse);     // degrees * 10
    sbufWriteU8(dst, gpsSol.numSat);
	
    sbufWriteU16(dst, (uint16_t) (GPS_distanceToHome / 10));    // use units of 10m to increase range of U16 to 655.36km
    sbufWriteU16(dst, GPS_directionToHome);

    uint8_t gpsFlags = 0;
    if (STATE(GPS_FIX)) {
        gpsFlags |= GPS_FLAGS_FIX;
    }
    if (STATE(GPS_FIX_HOME)) {
        gpsFlags |= GPS_FLAGS_FIX_HOME;
    }
    sbufWriteU8(dst, gpsFlags);
}

// Mag, Baro (and Vario) data
void ghstFrameMagBaro(sbuf_t *dst)
{
    int16_t vario = 0;
    int16_t altitude = 0;
    int16_t yaw = 0;
    uint8_t flags = 0;

#ifdef USE_VARIO
    if (sensors(SENSOR_VARIO) && telemetryIsSensorEnabled(SENSOR_VARIO)) {
        vario = getEstimatedVario();       // vario, cm/s
        flags |= MISC_FLAGS_VARIO;
    }
#endif

#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && telemetryIsSensorEnabled(SENSOR_ALTITUDE)) {
        flags |= MISC_FLAGS_BAROALT;
        altitude = (constrain(getEstimatedAltitudeCm(), -32000 * 100, 32000 * 100) / 100);
    }
#endif

#ifdef USE_MAG
    if (sensors(SENSOR_MAG) && telemetryIsSensorEnabled(SENSOR_HEADING)) {
        flags |= MISC_FLAGS_MAGHEAD;
        yaw = attitude.values.yaw;
    }
#endif

    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, GHST_FRAME_GPS_PAYLOAD_SIZE + GHST_FRAME_LENGTH_CRC + GHST_FRAME_LENGTH_TYPE);
    sbufWriteU8(dst, GHST_DL_MAGBARO);

    sbufWriteU16(dst, yaw);                 // magHeading, deci-degrees
    sbufWriteU16(dst, altitude);            // baroAltitude, m
    sbufWriteU8(dst, vario);                // cm/s
	
    sbufWriteU16(dst, 0);
    sbufWriteU16(dst, 0);

    sbufWriteU8(dst, flags);
}

// schedule array to decide how often each type of frame is sent
typedef enum {
    GHST_FRAME_START_INDEX = 0,
    GHST_FRAME_PACK_INDEX = GHST_FRAME_START_INDEX, // Battery (Pack) data
    GHST_FRAME_GPS_PRIMARY_INDEX,                   // GPS, primary values (Lat, Long, Alt)
    GHST_FRAME_GPS_SECONDARY_INDEX,                 // GPS, secondary values (Sat Count, HDOP, etc.)
    GHST_FRAME_MAGBARO_INDEX,                       // Magnetometer/Baro values
    GHST_SCHEDULE_COUNT_MAX
} ghstFrameTypeIndex_e;

static uint8_t ghstScheduleCount;
static uint8_t ghstSchedule[GHST_SCHEDULE_COUNT_MAX];

static void processGhst(void)
{
    static uint8_t ghstScheduleIndex = 0;

    const uint8_t currentSchedule = ghstSchedule[ghstScheduleIndex];

    sbuf_t ghstPayloadBuf;
    sbuf_t *dst = &ghstPayloadBuf;

    if (currentSchedule & BIT(GHST_FRAME_PACK_INDEX)) {
        ghstInitializeFrame(dst);
        ghstFramePackTelemetry(dst);
        ghstFinalize(dst);
    }

    if (currentSchedule & BIT(GHST_FRAME_GPS_PRIMARY_INDEX)) {
        ghstInitializeFrame(dst);
        ghstFrameGpsPrimaryTelemetry(dst);
        ghstFinalize(dst);
    }

    if (currentSchedule & BIT(GHST_FRAME_GPS_SECONDARY_INDEX)) {
        ghstInitializeFrame(dst);
        ghstFrameGpsSecondaryTelemetry(dst);
        ghstFinalize(dst);
    }

    if (currentSchedule & BIT(GHST_FRAME_MAGBARO_INDEX)) {
        ghstInitializeFrame(dst);
        ghstFrameMagBaro(dst);
        ghstFinalize(dst);
    }

    ghstScheduleIndex = (ghstScheduleIndex + 1) % ghstScheduleCount;
}

void initGhstTelemetry(void)
{
    // If the GHST Rx driver is active, since tx and rx share the same pin, assume telemetry is enabled.
    ghstTelemetryEnabled = ghstRxIsActive();

    if (!ghstTelemetryEnabled) {
        return;
    }

    int index = 0;
    if ((isBatteryVoltageConfigured() && telemetryIsSensorEnabled(SENSOR_VOLTAGE))
        || (isAmperageConfigured() && telemetryIsSensorEnabled(SENSOR_CURRENT | SENSOR_FUEL))) {
        ghstSchedule[index++] = BIT(GHST_FRAME_PACK_INDEX);
    }

#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)
       && telemetryIsSensorEnabled(SENSOR_ALTITUDE | SENSOR_LAT_LONG)) {
        ghstSchedule[index++] = BIT(GHST_FRAME_GPS_PRIMARY_INDEX);
    }

    if (featureIsEnabled(FEATURE_GPS)
       && telemetryIsSensorEnabled(SENSOR_GROUND_SPEED | SENSOR_HEADING)) {
        ghstSchedule[index++] = BIT(GHST_FRAME_GPS_SECONDARY_INDEX);
     }
#endif

#if defined(USE_BARO) || defined(USE_MAG) || defined(USE_VARIO)
    if ((sensors(SENSOR_BARO) && telemetryIsSensorEnabled(SENSOR_ALTITUDE)) 
        || (sensors(SENSOR_MAG) && telemetryIsSensorEnabled(SENSOR_HEADING)) 
        || (sensors(SENSOR_VARIO) && telemetryIsSensorEnabled(SENSOR_VARIO))) {
        ghstSchedule[index++] = BIT(GHST_FRAME_MAGBARO_INDEX);
    }
#endif

    ghstScheduleCount = (uint8_t)index;
 }

bool checkGhstTelemetryState(void)
{
    return ghstTelemetryEnabled;
}

// Called periodically by the scheduler
 void handleGhstTelemetry(timeUs_t currentTimeUs)
{
    static uint32_t ghstLastCycleTime;

    if (!ghstTelemetryEnabled) {
        return;
    }

    // Ready to send telemetry?
    if (currentTimeUs >= ghstLastCycleTime + (GHST_CYCLETIME_US / ghstScheduleCount)) {
        ghstLastCycleTime = currentTimeUs;
        processGhst();
    }

    // telemetry is sent from the Rx driver, ghstProcessFrame
}

#endif
