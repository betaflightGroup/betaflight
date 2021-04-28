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
 * Initial FrSky Hub Telemetry implementation by silpstream @ rcgroups.
 * Addition protocol work by airmamaf @ github.
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_TELEMETRY_FRSKY_HUB)

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/gps.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/position.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#if defined(USE_ESC_SENSOR_TELEMETRY)
#include "sensors/esc_sensor.h"
#endif

#include "frsky_hub.h"

static serialPort_t *frSkyHubPort = NULL;
static const serialPortConfig_t *portConfig = NULL;

#define FRSKY_HUB_BAUDRATE 9600
#define FRSKY_HUB_INITIAL_PORT_MODE MODE_TX

static portSharing_e frSkyHubPortSharing;

static frSkyHubWriteByteFn *frSkyHubWriteByte = NULL;

#define FRSKY_HUB_CYCLETIME_US    125000

#define PROTOCOL_HEADER       0x5E
#define PROTOCOL_TAIL         0x5E

// Data Ids  (bp = before decimal point; af = after decimal point)
// Official data IDs
#define ID_GPS_ALTIDUTE_BP    0x01
#define ID_GPS_ALTIDUTE_AP    0x09
#define ID_TEMPRATURE1        0x02
#define ID_RPM                0x03
#define ID_FUEL_LEVEL         0x04
#define ID_TEMPRATURE2        0x05
#define ID_VOLT               0x06
#define ID_ALTITUDE_BP        0x10
#define ID_ALTITUDE_AP        0x21
#define ID_GPS_SPEED_BP       0x11
#define ID_GPS_SPEED_AP       0x19
#define ID_LONGITUDE_BP       0x12
#define ID_LONGITUDE_AP       0x1A
#define ID_E_W                0x22
#define ID_LATITUDE_BP        0x13
#define ID_LATITUDE_AP        0x1B
#define ID_N_S                0x23
#define ID_COURSE_BP          0x14
#define ID_COURSE_AP          0x1C
#define ID_DATE_MONTH         0x15
#define ID_YEAR               0x16
#define ID_HOUR_MINUTE        0x17
#define ID_SECOND             0x18
#define ID_ACC_X              0x24
#define ID_ACC_Y              0x25
#define ID_ACC_Z              0x26
#define ID_VOLTAGE_AMP        0x39
#define ID_VOLTAGE_AMP_BP     0x3A
#define ID_VOLTAGE_AMP_AP     0x3B
#define ID_CURRENT            0x28
// User defined data IDs
#define ID_GYRO_X             0x40
#define ID_GYRO_Y             0x41
#define ID_GYRO_Z             0x42

#define ID_VERT_SPEED         0x30 // opentx vario

#define GPS_BAD_QUALITY       300
#define GPS_MAX_HDOP_VAL      9999
#define DELAY_FOR_BARO_INITIALISATION_US 5000000
#define BLADE_NUMBER_DIVIDER  5 // should set 12 blades in Taranis

enum
{
    TELEMETRY_STATE_UNINITIALIZED,
    TELEMETRY_STATE_INITIALIZED_SERIAL,
    TELEMETRY_STATE_INITIALIZED_EXTERNAL,
};

static uint8_t telemetryState = TELEMETRY_STATE_UNINITIALIZED;

static void serializeFrSkyHub(uint8_t data)
{
    // take care of byte stuffing
    if (data == 0x5e) {
        frSkyHubWriteByte(0x5d);
        frSkyHubWriteByte(0x3e);
    } else if (data == 0x5d) {
        frSkyHubWriteByte(0x5d);
        frSkyHubWriteByte(0x3d);
    } else{
        frSkyHubWriteByte(data);
    }
}

static void frSkyHubWriteFrame(const uint8_t id, const int16_t data)
{
    frSkyHubWriteByte(PROTOCOL_HEADER);
    frSkyHubWriteByte(id);

    serializeFrSkyHub((uint8_t)data);
    serializeFrSkyHub(data >> 8);
}

static void sendTelemetryTail(void)
{
    frSkyHubWriteByte(PROTOCOL_TAIL);
}

static void frSkyHubWriteByteInternal(const char data)
 {
   serialWrite(frSkyHubPort, data);
 }

#if defined(USE_ACC)
static void sendAccel(void)
{
    for (unsigned i = 0; i < 3; i++) {
        frSkyHubWriteFrame(ID_ACC_X + i, ((int16_t)(acc.accADC[i] * acc.dev.acc_1G_rec) * 1000));
    }
}
#endif

static void sendThrottleOrBatterySizeAsRpm(void)
{
    int16_t data = 0;
#if defined(USE_ESC_SENSOR_TELEMETRY)
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData) {
        data = escData->dataAge < ESC_DATA_INVALID ? (calcEscRpm(escData->rpm) / 10) : 0;
    }
#else
    if (ARMING_FLAG(ARMED)) {
        const throttleStatus_e throttleStatus = calculateThrottleStatus();
        uint16_t throttleForRPM = rcCommand[THROTTLE] / BLADE_NUMBER_DIVIDER;
        if (throttleStatus == THROTTLE_LOW && featureIsEnabled(FEATURE_MOTOR_STOP)) {
            throttleForRPM = 0;
        }
        data = throttleForRPM;
    } else {
        data = (batteryConfig()->batteryCapacity / BLADE_NUMBER_DIVIDER);
    }
#endif

    frSkyHubWriteFrame(ID_RPM, data);
}

static void sendTemperature1(void)
{
    int16_t data = 0;
#if defined(USE_ESC_SENSOR_TELEMETRY)
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData) {
        data = escData->dataAge < ESC_DATA_INVALID ? escData->temperature : 0;
    }
#elif defined(USE_BARO)
    data = (baro.baroTemperature + 50)/ 100; // Airmamaf
#else
    data = gyroGetTemperature() / 10;
#endif
    frSkyHubWriteFrame(ID_TEMPRATURE1, data);
}

static void sendTime(void)
{
    uint32_t seconds = millis() / 1000;
    uint8_t minutes = (seconds / 60) % 60;

    // if we fly for more than an hour, something's wrong anyway
    frSkyHubWriteFrame(ID_HOUR_MINUTE, minutes << 8);
    frSkyHubWriteFrame(ID_SECOND, seconds % 60);
}

#if defined(USE_GPS) || defined(USE_MAG)
// Frsky pdf: dddmm.mmmm
// .mmmm is returned in decimal fraction of minutes.
static void GPStoDDDMM_MMMM(int32_t mwiigps, gpsCoordinateDDDMMmmmm_t *result)
{
    int32_t absgps, deg, min;

    absgps = ABS(mwiigps);
    deg    = absgps / GPS_DEGREES_DIVIDER;
    absgps = (absgps - deg * GPS_DEGREES_DIVIDER) * 60;        // absgps = Minutes left * 10^7
    min    = absgps / GPS_DEGREES_DIVIDER;                     // minutes left

    if (telemetryConfig()->frsky_coordinate_format == FRSKY_FORMAT_DMS) {
        result->dddmm = deg * 100 + min;
    } else {
        result->dddmm = deg * 60 + min;
    }

    result->mmmm  = (absgps - min * GPS_DEGREES_DIVIDER) / 1000;
}

static void sendLatLong(int32_t coord[2])
{
    gpsCoordinateDDDMMmmmm_t coordinate;
    GPStoDDDMM_MMMM(coord[GPS_LATITUDE], &coordinate);
    frSkyHubWriteFrame(ID_LATITUDE_BP, coordinate.dddmm);
    frSkyHubWriteFrame(ID_LATITUDE_AP, coordinate.mmmm);
    frSkyHubWriteFrame(ID_N_S, coord[GPS_LATITUDE] < 0 ? 'S' : 'N');

    GPStoDDDMM_MMMM(coord[GPS_LONGITUDE], &coordinate);
    frSkyHubWriteFrame(ID_LONGITUDE_BP, coordinate.dddmm);
    frSkyHubWriteFrame(ID_LONGITUDE_AP, coordinate.mmmm);
    frSkyHubWriteFrame(ID_E_W, coord[GPS_LONGITUDE] < 0 ? 'W' : 'E');
}

#if defined(USE_GPS)
static void sendGpsAltitude(void)
{
    int32_t altitudeCm = gpsSol.llh.altCm;

    // Send real GPS altitude only if it's reliable (there's a GPS fix)
    if (!STATE(GPS_FIX)) {
        altitudeCm = 0;
    }
    frSkyHubWriteFrame(ID_GPS_ALTIDUTE_BP, altitudeCm / 100); // meters: integer part, eg. 123 from 123.45m
    frSkyHubWriteFrame(ID_GPS_ALTIDUTE_AP, altitudeCm % 100); // meters: fractional part, eg. 45 from 123.45m
}

static void sendSatalliteSignalQualityAsTemperature2(uint8_t cycleNum)
{
    uint16_t satellite = gpsSol.numSat;

    if (gpsSol.hdop > GPS_BAD_QUALITY && ( (cycleNum % 16 ) < 8)) { // Every 1s
        satellite = constrain(gpsSol.hdop, 0, GPS_MAX_HDOP_VAL);
    }
    int16_t data;
    if (telemetryConfig()->frsky_unit == UNIT_IMPERIAL) {
        float tmp = (satellite - 32) / 1.8f;
        // Round the value
        tmp += (tmp < 0) ? -0.5f : 0.5f;
        data = tmp;
    } else {
        data = satellite;
    }
    frSkyHubWriteFrame(ID_TEMPRATURE2, data);
}

static void sendSpeed(void)
{
    if (!STATE(GPS_FIX)) {
        return;
    }
    // Speed should be sent in knots (GPS speed is in cm/s)
    // convert to knots: 1cm/s = 0.0194384449 knots
    frSkyHubWriteFrame(ID_GPS_SPEED_BP, gpsSol.groundSpeed * 1944 / 100000);
    frSkyHubWriteFrame(ID_GPS_SPEED_AP, (gpsSol.groundSpeed * 1944 / 100) % 100);
}

static void sendFakeLatLong(void)
{
    // Heading is only displayed on OpenTX if non-zero lat/long is also sent
    int32_t coord[2] = {0,0};

    coord[GPS_LATITUDE] = ((0.01f * telemetryConfig()->gpsNoFixLatitude) * GPS_DEGREES_DIVIDER);
    coord[GPS_LONGITUDE] = ((0.01f * telemetryConfig()->gpsNoFixLongitude) * GPS_DEGREES_DIVIDER);

    sendLatLong(coord);
}

static void sendGPSLatLong(void)
{
    static uint8_t gpsFixOccured = 0;
    int32_t coord[2] = {0,0};

    if (STATE(GPS_FIX) || gpsFixOccured == 1) {
        // If we have ever had a fix, send the last known lat/long
        gpsFixOccured = 1;
        coord[GPS_LATITUDE] = gpsSol.llh.lat;
        coord[GPS_LONGITUDE] = gpsSol.llh.lon;
        sendLatLong(coord);
    } else {
        // otherwise send fake lat/long in order to display compass value
        sendFakeLatLong();
    }
}

#endif
#endif

/*
 * Send voltage via ID_VOLT
 *
 * NOTE: This sends voltage divided by batteryCellCount. To get the real
 * battery voltage, you need to multiply the value by batteryCellCount.
 */
static void sendVoltageCells(void)
{
    static uint16_t currentCell;
    uint32_t cellVoltage = 0;
    const uint8_t cellCount = getBatteryCellCount();

    if (cellCount) {
        currentCell %= cellCount;
        /*
        * Format for Voltage Data for single cells is like this:
        *
        *  llll llll cccc hhhh
        *  l: Low voltage bits
        *  h: High voltage bits
        *  c: Cell number (starting at 0)
        *
        * The actual value sent for cell voltage has resolution of 0.002 volts
        * Since vbat has resolution of 0.1 volts it has to be multiplied by 50
        */
        cellVoltage = ((uint32_t)getBatteryVoltage() * 100 + cellCount) / (cellCount * 2);
    } else {
        currentCell = 0;
    }

    // Cell number is at bit 9-12
    uint16_t data = (currentCell << 4);

    // Lower voltage bits are at bit 0-8
    data |= ((cellVoltage & 0x0ff) << 8);

    // Higher voltage bits are at bits 13-15
    data |= ((cellVoltage & 0xf00) >> 8);

    frSkyHubWriteFrame(ID_VOLT, data);

    currentCell++;
}

/*
 * Send voltage with ID_VOLTAGE_AMP
 */
static void sendVoltageAmp(void)
{
    uint16_t voltage = getLegacyBatteryVoltage();
    const uint8_t cellCount = getBatteryCellCount();

    if (telemetryConfig()->frsky_vfas_precision == FRSKY_VFAS_PRECISION_HIGH) {
        // Use new ID 0x39 to send voltage directly in 0.1 volts resolution
        if (telemetryConfig()->report_cell_voltage && cellCount) {
            voltage /= cellCount;
        }
        frSkyHubWriteFrame(ID_VOLTAGE_AMP, voltage);
    } else {
        // send in 0.2 volts resolution
        voltage *= 110 / 21;
        if (telemetryConfig()->report_cell_voltage && cellCount) {
            voltage /= cellCount;
        }

        frSkyHubWriteFrame(ID_VOLTAGE_AMP_BP, voltage / 100);
        frSkyHubWriteFrame(ID_VOLTAGE_AMP_AP, ((voltage % 100) + 5) / 10);
    }
}

static void sendAmperage(void)
{
    frSkyHubWriteFrame(ID_CURRENT, (uint16_t)(getAmperage() / 10));
}

static void sendFuelLevel(void)
{
    int16_t data;
    if (batteryConfig()->batteryCapacity > 0) {
        data = (uint16_t)calculateBatteryPercentageRemaining();
    } else {
        data = (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF);
    }
    frSkyHubWriteFrame(ID_FUEL_LEVEL, data);
}

#if defined(USE_MAG)
static void sendFakeLatLongThatAllowsHeadingDisplay(void)
{
    // Heading is only displayed on OpenTX if non-zero lat/long is also sent
    int32_t coord[2] = {
        1 * GPS_DEGREES_DIVIDER,
        1 * GPS_DEGREES_DIVIDER
    };

    sendLatLong(coord);
}

static void sendHeading(void)
{
    frSkyHubWriteFrame(ID_COURSE_BP, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    frSkyHubWriteFrame(ID_COURSE_AP, 0);
}
#endif

bool initFrSkyHubTelemetry(void)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_FRSKY_HUB);
        if (portConfig) {
            frSkyHubPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_FRSKY_HUB);

            frSkyHubWriteByte = frSkyHubWriteByteInternal;

            telemetryState = TELEMETRY_STATE_INITIALIZED_SERIAL;
        }

        return true;
    }

    return false;
}

bool initFrSkyHubTelemetryExternal(frSkyHubWriteByteFn *frSkyHubWriteByteExternal)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        frSkyHubWriteByte = frSkyHubWriteByteExternal;

        telemetryState = TELEMETRY_STATE_INITIALIZED_EXTERNAL;

        return true;
    }

    return false;
}

void freeFrSkyHubTelemetryPort(void)
{
    closeSerialPort(frSkyHubPort);
    frSkyHubPort = NULL;
}

static void configureFrSkyHubTelemetryPort(void)
{
    if (portConfig) {
        frSkyHubPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_FRSKY_HUB, NULL, NULL, FRSKY_HUB_BAUDRATE, FRSKY_HUB_INITIAL_PORT_MODE, telemetryConfig()->telemetry_inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED);
    }
}

void checkFrSkyHubTelemetryState(void)
{
    if (telemetryState == TELEMETRY_STATE_INITIALIZED_SERIAL) {
        if (telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
            if (frSkyHubPort == NULL && telemetrySharedPort != NULL) {
                frSkyHubPort = telemetrySharedPort;
            }
        } else {
            bool enableSerialTelemetry = telemetryDetermineEnabledState(frSkyHubPortSharing);
            if (enableSerialTelemetry && !frSkyHubPort) {
                configureFrSkyHubTelemetryPort();
            } else if (!enableSerialTelemetry && frSkyHubPort) {
                freeFrSkyHubTelemetryPort();
            }
        }
    }
}

void processFrSkyHubTelemetry(timeUs_t currentTimeUs)
{
    static uint32_t frSkyHubLastCycleTime = 0;
    static uint8_t cycleNum = 0;

    if (cmpTimeUs(currentTimeUs, frSkyHubLastCycleTime) < FRSKY_HUB_CYCLETIME_US) {
        return;
    }
    frSkyHubLastCycleTime = currentTimeUs;

    cycleNum++;

#if defined(USE_ACC)
    if (sensors(SENSOR_ACC) && telemetryIsSensorEnabled(SENSOR_ACC_X | SENSOR_ACC_Y | SENSOR_ACC_Z)) {
        // Sent every 125ms
        sendAccel();
    }
#endif

#if defined(USE_BARO) || defined(USE_RANGEFINDER) || defined(USE_GPS)
    if (sensors(SENSOR_BARO | SENSOR_RANGEFINDER) | sensors(SENSOR_GPS)) {
        // Sent every 125ms
        // Send vertical speed for opentx. ID_VERT_SPEED
        // Unit is cm/s
#ifdef USE_VARIO
        if (telemetryIsSensorEnabled(SENSOR_VARIO)) {
            frSkyHubWriteFrame(ID_VERT_SPEED, getEstimatedVario());
        }
#endif

        // Sent every 500ms
        if ((cycleNum % 4) == 0 && telemetryIsSensorEnabled(SENSOR_ALTITUDE)) {
            int32_t altitudeCm = getEstimatedAltitudeCm();

            /* Allow 5s to boot correctly othervise send zero to prevent OpenTX
             * sensor lost notifications after warm boot. */
            if (frSkyHubLastCycleTime < DELAY_FOR_BARO_INITIALISATION_US) {
                altitudeCm = 0;
            }

            frSkyHubWriteFrame(ID_ALTITUDE_BP, altitudeCm / 100); // meters: integer part, eg. 123 from 123.45m
            frSkyHubWriteFrame(ID_ALTITUDE_AP, altitudeCm % 100); // meters: fractional part, eg. 45 from 123.45m
        }
    }
#endif

#if defined(USE_MAG)
    if (sensors(SENSOR_MAG) && telemetryIsSensorEnabled(SENSOR_HEADING)) {
        // Sent every 500ms
        if ((cycleNum % 4) == 0) {
            sendHeading();
        }
    }
#endif

    // Sent every 1s
    if ((cycleNum % 8) == 0) {
        sendTemperature1();
        sendThrottleOrBatterySizeAsRpm();

        if (isBatteryVoltageConfigured()) {
            if (telemetryIsSensorEnabled(SENSOR_VOLTAGE)) {
                sendVoltageCells();
                sendVoltageAmp();
            }

            if (isAmperageConfigured()) {
                if (telemetryIsSensorEnabled(SENSOR_CURRENT)) {
                    sendAmperage();
                }
                if (telemetryIsSensorEnabled(SENSOR_FUEL)) {
                    sendFuelLevel();
                }
            }
        }

#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) {
            if (telemetryIsSensorEnabled(SENSOR_GROUND_SPEED)) {
                sendSpeed();
            }
            if (telemetryIsSensorEnabled(SENSOR_ALTITUDE)) {
                sendGpsAltitude();
            }
            sendSatalliteSignalQualityAsTemperature2(cycleNum);
            if (telemetryIsSensorEnabled(SENSOR_LAT_LONG)) {
                sendGPSLatLong();
            }
        } else
#endif
#if defined(USE_MAG)
        if (sensors(SENSOR_MAG)) {
            sendFakeLatLongThatAllowsHeadingDisplay();
        }
#else
        {}
#endif
    }

    // Sent every 5s
    if (cycleNum == 40) {
        cycleNum = 0;
        sendTime();
    }

    sendTelemetryTail();
}

void handleFrSkyHubTelemetry(timeUs_t currentTimeUs)
{
    if (telemetryState == TELEMETRY_STATE_INITIALIZED_SERIAL && frSkyHubPort) {
        processFrSkyHubTelemetry(currentTimeUs);
    }
}
#endif
