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
 * SmartPort Telemetry implementation by frank26080115
 * see https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#if defined(USE_TELEMETRY_SMARTPORT)

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "rx/frsky_crc.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/serial.h"

#include "msp/msp.h"

#include "rx/rx.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/msp_shared.h"
#include "telemetry/smartport.h"
#include "telemetry/telemetry.h"

#define SMARTPORT_MIN_TELEMETRY_RESPONSE_DELAY_US 500

// these data identifiers are obtained from https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky_hub.h
enum
{
    FSSP_DATAID_SPEED      = 0x0830 ,
    FSSP_DATAID_VFAS       = 0x0210 ,
    FSSP_DATAID_VFAS1      = 0x0211 ,
    FSSP_DATAID_VFAS2      = 0x0212 ,
    FSSP_DATAID_VFAS3      = 0x0213 ,
    FSSP_DATAID_VFAS4      = 0x0214 ,
    FSSP_DATAID_VFAS5      = 0x0215 ,
    FSSP_DATAID_VFAS6      = 0x0216 ,
    FSSP_DATAID_VFAS7      = 0x0217 ,
    FSSP_DATAID_VFAS8      = 0x0218 ,
    FSSP_DATAID_CURRENT    = 0x0200 ,
    FSSP_DATAID_CURRENT1   = 0x0201 ,
    FSSP_DATAID_CURRENT2   = 0x0202 ,
    FSSP_DATAID_CURRENT3   = 0x0203 ,
    FSSP_DATAID_CURRENT4   = 0x0204 ,
    FSSP_DATAID_CURRENT5   = 0x0205 ,
    FSSP_DATAID_CURRENT6   = 0x0206 ,
    FSSP_DATAID_CURRENT7   = 0x0207 ,
    FSSP_DATAID_CURRENT8   = 0x0208 ,
    FSSP_DATAID_RPM        = 0x0500 ,
    FSSP_DATAID_RPM1       = 0x0501 ,
    FSSP_DATAID_RPM2       = 0x0502 ,
    FSSP_DATAID_RPM3       = 0x0503 ,
    FSSP_DATAID_RPM4       = 0x0504 ,
    FSSP_DATAID_RPM5       = 0x0505 ,
    FSSP_DATAID_RPM6       = 0x0506 ,
    FSSP_DATAID_RPM7       = 0x0507 ,
    FSSP_DATAID_RPM8       = 0x0508 ,
    FSSP_DATAID_ALTITUDE   = 0x0100 ,
    FSSP_DATAID_FUEL       = 0x0600 ,
    FSSP_DATAID_ADC1       = 0xF102 ,
    FSSP_DATAID_ADC2       = 0xF103 ,
    FSSP_DATAID_LATLONG    = 0x0800 ,
    FSSP_DATAID_VARIO      = 0x0110 ,
    FSSP_DATAID_CELLS      = 0x0300 ,
    FSSP_DATAID_CELLS_LAST = 0x030F ,
    FSSP_DATAID_HEADING    = 0x0840 ,
// DIY range 0x5100 to 0x52FF
    FSSP_DATAID_CAP_USED   = 0x5250 ,
#if defined(USE_ACC)
    FSSP_DATAID_PITCH      = 0x5230 , // custom
    FSSP_DATAID_ROLL       = 0x5240 , // custom
    FSSP_DATAID_ACCX       = 0x0700 ,
    FSSP_DATAID_ACCY       = 0x0710 ,
    FSSP_DATAID_ACCZ       = 0x0720 ,
#endif
    FSSP_DATAID_T1         = 0x0400 ,
    FSSP_DATAID_T11        = 0x0401 ,
    FSSP_DATAID_T1GYRO     = 0x0402 ,
    FSSP_DATAID_T2         = 0x0410 ,
    FSSP_DATAID_HOME_DIST  = 0x0420 ,
    FSSP_DATAID_GPS_ALT    = 0x0820 ,
    FSSP_DATAID_ASPD       = 0x0A00 ,
    FSSP_DATAID_TEMP       = 0x0B70 ,
    FSSP_DATAID_TEMP1      = 0x0B71 ,
    FSSP_DATAID_TEMP2      = 0x0B72 ,
    FSSP_DATAID_TEMP3      = 0x0B73 ,
    FSSP_DATAID_TEMP4      = 0x0B74 ,
    FSSP_DATAID_TEMP5      = 0x0B75 ,
    FSSP_DATAID_TEMP6      = 0x0B76 ,
    FSSP_DATAID_TEMP7      = 0x0B77 ,
    FSSP_DATAID_TEMP8      = 0x0B78 ,
    FSSP_DATAID_A3         = 0x0900 ,
    FSSP_DATAID_A4         = 0x0910
};

// if adding more sensors then increase this value (should be equal to the maximum number of ADD_SENSOR calls)
#define MAX_DATAIDS 20

static uint16_t frSkyDataIdTable[MAX_DATAIDS];

#ifdef USE_ESC_SENSOR_TELEMETRY
// number of sensors to send between sending the ESC sensors
#define ESC_SENSOR_PERIOD 7

// if adding more esc sensors then increase this value
#define MAX_ESC_DATAIDS 4

static uint16_t frSkyEscDataIdTable[MAX_ESC_DATAIDS];
#endif

typedef struct frSkyTableInfo_s {
    uint16_t * table;
    uint8_t size;
    uint8_t index;
} frSkyTableInfo_t;

static frSkyTableInfo_t frSkyDataIdTableInfo = { frSkyDataIdTable, 0, 0 };
#ifdef USE_ESC_SENSOR_TELEMETRY
static frSkyTableInfo_t frSkyEscDataIdTableInfo = {frSkyEscDataIdTable, 0, 0};
#endif

#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX
#define SMARTPORT_SERVICE_TIMEOUT_US 1000 // max allowed time to find a value to send

static serialPort_t *smartPortSerialPort = NULL; // The 'SmartPort'(tm) Port.
static const serialPortConfig_t *portConfig;

static portSharing_e smartPortPortSharing;

enum
{
    TELEMETRY_STATE_UNINITIALIZED,
    TELEMETRY_STATE_INITIALIZED_SERIAL,
    TELEMETRY_STATE_INITIALIZED_EXTERNAL,
};

static uint8_t telemetryState = TELEMETRY_STATE_UNINITIALIZED;

typedef struct smartPortFrame_s {
    uint8_t  sensorId;
    smartPortPayload_t payload;
    uint8_t  crc;
} __attribute__((packed)) smartPortFrame_t;

#define SMARTPORT_MSP_PAYLOAD_SIZE (sizeof(smartPortPayload_t) - sizeof(uint8_t))

static smartPortWriteFrameFn *smartPortWriteFrame;

#if defined(USE_MSP_OVER_TELEMETRY)
static bool smartPortMspReplyPending = false;
#endif

smartPortPayload_t *smartPortDataReceive(uint16_t c, bool *clearToSend, smartPortReadyToSendFn *readyToSend, bool useChecksum)
{
    static uint8_t rxBuffer[sizeof(smartPortPayload_t)];
    static uint8_t smartPortRxBytes = 0;
    static bool skipUntilStart = true;
    static bool awaitingSensorId = false;
    static bool byteStuffing = false;
    static uint16_t checksum = 0;

    if (c == FSSP_START_STOP) {
        *clearToSend = false;
        smartPortRxBytes = 0;
        awaitingSensorId = true;
        skipUntilStart = false;

        return NULL;
    } else if (skipUntilStart) {
        return NULL;
    }

    if (awaitingSensorId) {
        awaitingSensorId = false;
        if ((c == FSSP_SENSOR_ID1) && readyToSend()) {
            // our slot is starting, start sending
            *clearToSend = true;
            // no need to decode more
            skipUntilStart = true;
        } else if (c == FSSP_SENSOR_ID2) {
            checksum = 0;
        } else {
            skipUntilStart = true;
        }
    } else {
        if (c == FSSP_DLE) {
            byteStuffing = true;

            return NULL;
        } else if (byteStuffing) {
            c ^= FSSP_DLE_XOR;
            byteStuffing = false;
        }

        if (smartPortRxBytes < sizeof(smartPortPayload_t)) {
            rxBuffer[smartPortRxBytes++] = (uint8_t)c;
            checksum += c;

            if (!useChecksum && (smartPortRxBytes == sizeof(smartPortPayload_t))) {
                skipUntilStart = true;

                return (smartPortPayload_t *)&rxBuffer;
            }
        } else {
            skipUntilStart = true;

            checksum += c;
            checksum = (checksum & 0xFF) + (checksum >> 8);
            if (checksum == 0xFF) {
                return (smartPortPayload_t *)&rxBuffer;
            }
        }
    }

    return NULL;
}

void smartPortSendByte(uint8_t c, uint16_t *checksum, serialPort_t *port)
{
    // smart port escape sequence
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
        serialWrite(port, FSSP_DLE);
        serialWrite(port, c ^ FSSP_DLE_XOR);
    } else {
        serialWrite(port, c);
    }

    if (checksum != NULL) {
        frskyCheckSumStep(checksum, c);
    }
}

bool smartPortPayloadContainsMSP(const smartPortPayload_t *payload)
{
    return payload->frameId == FSSP_MSPC_FRAME_SMARTPORT || payload->frameId == FSSP_MSPC_FRAME_FPORT;
}

void smartPortWriteFrameSerial(const smartPortPayload_t *payload, serialPort_t *port, uint16_t checksum)
{
    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); i++) {
        smartPortSendByte(*data++, &checksum, port);
    }
    frskyCheckSumFini(&checksum);
    smartPortSendByte(checksum, NULL, port);
}

static void smartPortWriteFrameInternal(const smartPortPayload_t *payload)
{
    smartPortWriteFrameSerial(payload, smartPortSerialPort, 0);
}

static void smartPortSendPackage(uint16_t id, uint32_t val)
{
    smartPortPayload_t payload;
    payload.frameId = FSSP_DATA_FRAME;
    payload.valueId = id;
    payload.data = val;

    smartPortWriteFrame(&payload);
}

#define ADD_SENSOR(dataId) frSkyDataIdTableInfo.table[frSkyDataIdTableInfo.index++] = dataId
#define ADD_ESC_SENSOR(dataId) frSkyEscDataIdTableInfo.table[frSkyEscDataIdTableInfo.index++] = dataId

static void initSmartPortSensors(void)
{
    frSkyDataIdTableInfo.index = 0;

    if (telemetryIsSensorEnabled(SENSOR_MODE)) {
        ADD_SENSOR(FSSP_DATAID_T1);
        ADD_SENSOR(FSSP_DATAID_T2);
    }

#if defined(USE_ADC_INTERNAL)
    if (telemetryIsSensorEnabled(SENSOR_TEMPERATURE)) {
        ADD_SENSOR(FSSP_DATAID_T11);
    }
#endif

#if defined(USE_GYRO)
    if (gyroHasTemperatureSensor() && telemetryIsSensorEnabled(SENSOR_TEMPERATURE)) {
        ADD_SENSOR(FSSP_DATAID_T1GYRO);
    }
#endif

    if (isBatteryVoltageConfigured() && telemetryIsSensorEnabled(SENSOR_VOLTAGE)) {
#ifdef USE_ESC_SENSOR_TELEMETRY
        if (!telemetryIsSensorEnabled(ESC_SENSOR_VOLTAGE))
#endif
        {
            ADD_SENSOR(FSSP_DATAID_VFAS);
        }

        ADD_SENSOR(FSSP_DATAID_A4);
    }

    if (isAmperageConfigured() && telemetryIsSensorEnabled(SENSOR_CURRENT)) {
#ifdef USE_ESC_SENSOR_TELEMETRY
        if (!telemetryIsSensorEnabled(ESC_SENSOR_CURRENT))
#endif
        {
            ADD_SENSOR(FSSP_DATAID_CURRENT);
        }

        if (telemetryIsSensorEnabled(SENSOR_FUEL)) {
            ADD_SENSOR(FSSP_DATAID_FUEL);
        }

        if (telemetryIsSensorEnabled(SENSOR_CAP_USED)) {
            ADD_SENSOR(FSSP_DATAID_CAP_USED);
        }
    }

    if (telemetryIsSensorEnabled(SENSOR_HEADING)) {
        ADD_SENSOR(FSSP_DATAID_HEADING);
    }

#if defined(USE_ACC)
    if (sensors(SENSOR_ACC)) {
        if (telemetryIsSensorEnabled(SENSOR_PITCH)) {
            ADD_SENSOR(FSSP_DATAID_PITCH);
        }
        if (telemetryIsSensorEnabled(SENSOR_ROLL)) {
            ADD_SENSOR(FSSP_DATAID_ROLL);
        }
        if (telemetryIsSensorEnabled(SENSOR_ACC_X)) {
            ADD_SENSOR(FSSP_DATAID_ACCX);
        }
        if (telemetryIsSensorEnabled(SENSOR_ACC_Y)) {
            ADD_SENSOR(FSSP_DATAID_ACCY);
        }
        if (telemetryIsSensorEnabled(SENSOR_ACC_Z)) {
            ADD_SENSOR(FSSP_DATAID_ACCZ);
        }
    }
#endif

    if (sensors(SENSOR_BARO)) {
        if (telemetryIsSensorEnabled(SENSOR_ALTITUDE)) {
            ADD_SENSOR(FSSP_DATAID_ALTITUDE);
        }
        if (telemetryIsSensorEnabled(SENSOR_VARIO)) {
            ADD_SENSOR(FSSP_DATAID_VARIO);
        }
    }

#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)) {
        if (telemetryIsSensorEnabled(SENSOR_GROUND_SPEED)) {
            ADD_SENSOR(FSSP_DATAID_SPEED);
        }
        if (telemetryIsSensorEnabled(SENSOR_LAT_LONG)) {
            ADD_SENSOR(FSSP_DATAID_LATLONG);
            ADD_SENSOR(FSSP_DATAID_LATLONG); // twice (one for lat, one for long)
        }
        if (telemetryIsSensorEnabled(SENSOR_DISTANCE)) {
            ADD_SENSOR(FSSP_DATAID_HOME_DIST);
        }
        if (telemetryIsSensorEnabled(SENSOR_ALTITUDE)) {
            ADD_SENSOR(FSSP_DATAID_GPS_ALT);
        }
    }
#endif

    frSkyDataIdTableInfo.size = frSkyDataIdTableInfo.index;
    frSkyDataIdTableInfo.index = 0;

#ifdef USE_ESC_SENSOR_TELEMETRY
    frSkyEscDataIdTableInfo.index = 0;

    if (telemetryIsSensorEnabled(ESC_SENSOR_VOLTAGE)) {
        ADD_ESC_SENSOR(FSSP_DATAID_VFAS);
    }
    if (telemetryIsSensorEnabled(ESC_SENSOR_CURRENT)) {
        ADD_ESC_SENSOR(FSSP_DATAID_CURRENT);
    }
    if (telemetryIsSensorEnabled(ESC_SENSOR_RPM)) {
        ADD_ESC_SENSOR(FSSP_DATAID_RPM);
    }
    if (telemetryIsSensorEnabled(ESC_SENSOR_TEMPERATURE)) {
        ADD_ESC_SENSOR(FSSP_DATAID_TEMP);
    }

    frSkyEscDataIdTableInfo.size = frSkyEscDataIdTableInfo.index;
    frSkyEscDataIdTableInfo.index = 0;
#endif
}

bool initSmartPortTelemetry(void)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT);
        if (portConfig) {
            smartPortPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_SMARTPORT);

            smartPortWriteFrame = smartPortWriteFrameInternal;

            initSmartPortSensors();

            telemetryState = TELEMETRY_STATE_INITIALIZED_SERIAL;
        }

        return true;
    }

    return false;
}

bool initSmartPortTelemetryExternal(smartPortWriteFrameFn *smartPortWriteFrameExternal)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        smartPortWriteFrame = smartPortWriteFrameExternal;

        initSmartPortSensors();

        telemetryState = TELEMETRY_STATE_INITIALIZED_EXTERNAL;

        return true;
    }

    return false;
}

static void freeSmartPortTelemetryPort(void)
{
    closeSerialPort(smartPortSerialPort);
    smartPortSerialPort = NULL;
}

static void configureSmartPortTelemetryPort(void)
{
    if (portConfig) {
        portOptions_e portOptions = (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) | (telemetryConfig()->telemetry_inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED);

        smartPortSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT, NULL, NULL, SMARTPORT_BAUD, SMARTPORT_UART_MODE, portOptions);
    }
}

void checkSmartPortTelemetryState(void)
{
    if (telemetryState == TELEMETRY_STATE_INITIALIZED_SERIAL) {
        bool enableSerialTelemetry = telemetryDetermineEnabledState(smartPortPortSharing);

        if (enableSerialTelemetry && !smartPortSerialPort) {
            configureSmartPortTelemetryPort();
        } else if (!enableSerialTelemetry && smartPortSerialPort) {
            freeSmartPortTelemetryPort();
        }
    }
}

#if defined(USE_MSP_OVER_TELEMETRY)
static void smartPortSendMspResponse(uint8_t *data) {
    smartPortPayload_t payload;
    payload.frameId = FSSP_MSPS_FRAME;
    memcpy(&payload.valueId, data, SMARTPORT_MSP_PAYLOAD_SIZE);

    smartPortWriteFrame(&payload);
}
#endif

void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *clearToSend, const timeUs_t *requestTimeout)
{
    static uint8_t smartPortIdCycleCnt = 0;
    static uint8_t t1Cnt = 0;
    static uint8_t t2Cnt = 0;
    static uint8_t skipRequests = 0;
#ifdef USE_ESC_SENSOR_TELEMETRY
    static uint8_t smartPortIdOffset = 0;
#endif

#if defined(USE_MSP_OVER_TELEMETRY)
    if (skipRequests) {
        skipRequests--;
    } else if (payload && smartPortPayloadContainsMSP(payload)) {
        // Do not check the physical ID here again
        // unless we start receiving other sensors' packets
        // Pass only the payload: skip frameId
        uint8_t *frameStart = (uint8_t *)&payload->valueId;
        smartPortMspReplyPending = handleMspFrame(frameStart, SMARTPORT_MSP_PAYLOAD_SIZE, &skipRequests);

        // Don't send MSP response after write to eeprom
        // CPU just got out of suspended state after writeEEPROM()
        // We don't know if the receiver is listening again
        // Skip a few telemetry requests before sending response
        if (skipRequests) {
            *clearToSend = false;
        }
    }
#else
    UNUSED(payload);
#endif

    bool doRun = true;
    while (doRun && *clearToSend && !skipRequests) {
        // Ensure we won't get stuck in the loop if there happens to be nothing available to send in a timely manner - dump the slot if we loop in there for too long.
        if (requestTimeout) {
            if (cmpTimeUs(micros(), *requestTimeout) >= 0) {
                *clearToSend = false;

                return;
            }
        } else {
            doRun = false;
        }

#if defined(USE_MSP_OVER_TELEMETRY)
        if (smartPortMspReplyPending) {
            smartPortMspReplyPending = sendMspReply(SMARTPORT_MSP_PAYLOAD_SIZE, &smartPortSendMspResponse);
            *clearToSend = false;

            return;
        }
#endif

        // we can send back any data we want, our tables keep track of the order and frequency of each data type we send
        frSkyTableInfo_t * tableInfo = &frSkyDataIdTableInfo;

#ifdef USE_ESC_SENSOR_TELEMETRY
        if (smartPortIdCycleCnt >= ESC_SENSOR_PERIOD) {
            // send ESC sensors
            tableInfo = &frSkyEscDataIdTableInfo;
            if (tableInfo->index == tableInfo->size) { // end of ESC table, return to other sensors
                tableInfo->index = 0;
                smartPortIdCycleCnt = 0;
                smartPortIdOffset++;
                if (smartPortIdOffset == getMotorCount() + 1) { // each motor and ESC_SENSOR_COMBINED
                    smartPortIdOffset = 0;
                }
            }
        }
        if (smartPortIdCycleCnt < ESC_SENSOR_PERIOD) {
            // send other sensors
            tableInfo = &frSkyDataIdTableInfo;
#endif
            if (tableInfo->index == tableInfo->size) { // end of table reached, loop back
                tableInfo->index = 0;
            }
#ifdef USE_ESC_SENSOR_TELEMETRY
        }
#endif
        uint16_t id = tableInfo->table[tableInfo->index];
#ifdef USE_ESC_SENSOR_TELEMETRY
        if (smartPortIdCycleCnt >= ESC_SENSOR_PERIOD) {
            id += smartPortIdOffset;
        }
#endif
        smartPortIdCycleCnt++;
        tableInfo->index++;

        int32_t tmpi;
        uint32_t tmp2 = 0;
        uint16_t vfasVoltage;

#ifdef USE_ESC_SENSOR_TELEMETRY
        escSensorData_t *escData;
#endif

        switch (id) {
            case FSSP_DATAID_VFAS       :
                vfasVoltage = telemetryConfig()->report_cell_voltage ? getBatteryAverageCellVoltage() : getBatteryVoltage();
                smartPortSendPackage(id, vfasVoltage); // in 0.01V according to SmartPort spec
                *clearToSend = false;
                break;
#ifdef USE_ESC_SENSOR_TELEMETRY
            case FSSP_DATAID_VFAS1      :
            case FSSP_DATAID_VFAS2      :
            case FSSP_DATAID_VFAS3      :
            case FSSP_DATAID_VFAS4      :
            case FSSP_DATAID_VFAS5      :
            case FSSP_DATAID_VFAS6      :
            case FSSP_DATAID_VFAS7      :
            case FSSP_DATAID_VFAS8      :
                escData = getEscSensorData(id - FSSP_DATAID_VFAS1);
                if (escData != NULL) {
                    smartPortSendPackage(id, escData->voltage);
                    *clearToSend = false;
                }
                break;
#endif
            case FSSP_DATAID_CURRENT    :
                smartPortSendPackage(id, getAmperage() / 10); // in 0.1A according to SmartPort spec
                *clearToSend = false;
                break;
#ifdef USE_ESC_SENSOR_TELEMETRY
            case FSSP_DATAID_CURRENT1   :
            case FSSP_DATAID_CURRENT2   :
            case FSSP_DATAID_CURRENT3   :
            case FSSP_DATAID_CURRENT4   :
            case FSSP_DATAID_CURRENT5   :
            case FSSP_DATAID_CURRENT6   :
            case FSSP_DATAID_CURRENT7   :
            case FSSP_DATAID_CURRENT8   :
                escData = getEscSensorData(id - FSSP_DATAID_CURRENT1);
                if (escData != NULL) {
                    smartPortSendPackage(id, escData->current);
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_RPM        :
                escData = getEscSensorData(ESC_SENSOR_COMBINED);
                if (escData != NULL) {
                    smartPortSendPackage(id, calcEscRpm(escData->rpm));
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_RPM1       :
            case FSSP_DATAID_RPM2       :
            case FSSP_DATAID_RPM3       :
            case FSSP_DATAID_RPM4       :
            case FSSP_DATAID_RPM5       :
            case FSSP_DATAID_RPM6       :
            case FSSP_DATAID_RPM7       :
            case FSSP_DATAID_RPM8       :
                escData = getEscSensorData(id - FSSP_DATAID_RPM1);
                if (escData != NULL) {
                    smartPortSendPackage(id, calcEscRpm(escData->rpm));
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_TEMP        :
                escData = getEscSensorData(ESC_SENSOR_COMBINED);
                if (escData != NULL) {
                    smartPortSendPackage(id, escData->temperature);
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_TEMP1      :
            case FSSP_DATAID_TEMP2      :
            case FSSP_DATAID_TEMP3      :
            case FSSP_DATAID_TEMP4      :
            case FSSP_DATAID_TEMP5      :
            case FSSP_DATAID_TEMP6      :
            case FSSP_DATAID_TEMP7      :
            case FSSP_DATAID_TEMP8      :
                escData = getEscSensorData(id - FSSP_DATAID_TEMP1);
                if (escData != NULL) {
                    smartPortSendPackage(id, escData->temperature);
                    *clearToSend = false;
                }
                break;
#endif
            case FSSP_DATAID_ALTITUDE   :
                smartPortSendPackage(id, getEstimatedAltitudeCm()); // in cm according to SmartPort spec
                *clearToSend = false;
                break;
            case FSSP_DATAID_FUEL       :
                {
                    uint32_t data;
                    if (batteryConfig()->batteryCapacity > 0) {
                        data = calculateBatteryPercentageRemaining();
                    } else {
                        data = getMAhDrawn();
                    }
                    smartPortSendPackage(id, data);
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_CAP_USED   :
                smartPortSendPackage(id, getMAhDrawn()); // given in mAh, should be in percent according to SmartPort spec
                *clearToSend = false;
                break;
#if defined(USE_VARIO)
            case FSSP_DATAID_VARIO      :
                smartPortSendPackage(id, getEstimatedVario()); // in cm/s according to SmartPort spec
                *clearToSend = false;
                break;
#endif
            case FSSP_DATAID_HEADING    :
                smartPortSendPackage(id, attitude.values.yaw * 10); // in degrees * 100 according to SmartPort spec
                *clearToSend = false;
                break;
#if defined(USE_ACC)
            case FSSP_DATAID_PITCH      :
                smartPortSendPackage(id, attitude.values.pitch); // given in 10*deg
                *clearToSend = false;
                break;
            case FSSP_DATAID_ROLL       :
                smartPortSendPackage(id, attitude.values.roll); // given in 10*deg
                *clearToSend = false;
                break;
            case FSSP_DATAID_ACCX       :
                smartPortSendPackage(id, lrintf(100 * acc.accADC[X] * acc.dev.acc_1G_rec)); // Multiply by 100 to show as x.xx g on Taranis
                *clearToSend = false;
                break;
            case FSSP_DATAID_ACCY       :
                smartPortSendPackage(id, lrintf(100 * acc.accADC[Y] * acc.dev.acc_1G_rec));
                *clearToSend = false;
                break;
            case FSSP_DATAID_ACCZ       :
                smartPortSendPackage(id, lrintf(100 * acc.accADC[Z] * acc.dev.acc_1G_rec));
                *clearToSend = false;
                break;
#endif
            case FSSP_DATAID_T1         :
                // we send all the flags as decimal digits for easy reading

                // the t1Cnt simply allows the telemetry view to show at least some changes
                t1Cnt++;
                if (t1Cnt == 4) {
                    t1Cnt = 1;
                }
                tmpi = t1Cnt * 10000; // start off with at least one digit so the most significant 0 won't be cut off
                // the Taranis seems to be able to fit 5 digits on the screen
                // the Taranis seems to consider this number a signed 16 bit integer

                if (!isArmingDisabled()) {
                    tmpi += 1;
                } else {
                    tmpi += 2;
                }
                if (ARMING_FLAG(ARMED)) {
                    tmpi += 4;
                }

                if (FLIGHT_MODE(ANGLE_MODE)) {
                    tmpi += 10;
                }
                if (FLIGHT_MODE(HORIZON_MODE)) {
                    tmpi += 20;
                }
                if (FLIGHT_MODE(PASSTHRU_MODE)) {
                    tmpi += 40;
                }

                if (FLIGHT_MODE(MAG_MODE)) {
                    tmpi += 100;
                }

                if (FLIGHT_MODE(HEADFREE_MODE)) {
                    tmpi += 4000;
                }

                smartPortSendPackage(id, (uint32_t)tmpi);
                *clearToSend = false;
                break;
            case FSSP_DATAID_T2         :
#ifdef USE_GPS
                if (sensors(SENSOR_GPS)) {
                    // satellite accuracy HDOP: 0 = worst [HDOP > 5.5m], 9 = best [HDOP <= 1.0m]
                    uint16_t hdop = constrain(scaleRange(gpsSol.hdop, 100, 550, 9, 0), 0, 9) * 100;
                    smartPortSendPackage(id, (STATE(GPS_FIX) ? 1000 : 0) + (STATE(GPS_FIX_HOME) ? 2000 : 0) + hdop + gpsSol.numSat);
                    *clearToSend = false;
                } else if (featureIsEnabled(FEATURE_GPS)) {
                    smartPortSendPackage(id, 0);
                    *clearToSend = false;
                } else
#endif
                if (telemetryConfig()->pidValuesAsTelemetry) {
                    switch (t2Cnt) {
                        case 0:
                            tmp2 = currentPidProfile->pid[PID_ROLL].P;
                            tmp2 += (currentPidProfile->pid[PID_PITCH].P<<8);
                            tmp2 += (currentPidProfile->pid[PID_YAW].P<<16);
                        break;
                        case 1:
                            tmp2 = currentPidProfile->pid[PID_ROLL].I;
                            tmp2 += (currentPidProfile->pid[PID_PITCH].I<<8);
                            tmp2 += (currentPidProfile->pid[PID_YAW].I<<16);
                        break;
                        case 2:
                            tmp2 = currentPidProfile->pid[PID_ROLL].D;
                            tmp2 += (currentPidProfile->pid[PID_PITCH].D<<8);
                            tmp2 += (currentPidProfile->pid[PID_YAW].D<<16);
                        break;
                        case 3:
                            tmp2 = currentControlRateProfile->rates[FD_ROLL];
                            tmp2 += (currentControlRateProfile->rates[FD_PITCH]<<8);
                            tmp2 += (currentControlRateProfile->rates[FD_YAW]<<16);
                        break;
                    }
                    tmp2 += t2Cnt<<24;
                    t2Cnt++;
                    if (t2Cnt == 4) {
                        t2Cnt = 0;
                    }
                    smartPortSendPackage(id, tmp2);
                    *clearToSend = false;
                }

                break;
#if defined(USE_ADC_INTERNAL)
            case FSSP_DATAID_T11        :
                smartPortSendPackage(id, getCoreTemperatureCelsius());
                *clearToSend = false;
                break;
#endif
#if defined(USE_GYRO)
            case FSSP_DATAID_T1GYRO        :
                smartPortSendPackage(id, gyroGetTemperature());
                *clearToSend = false;
                break;
#endif
#ifdef USE_GPS
            case FSSP_DATAID_SPEED      :
                if (STATE(GPS_FIX)) {
                    //convert to knots: 1cm/s = 0.0194384449 knots
                    //Speed should be sent in knots/1000 (GPS speed is in cm/s)
                    uint32_t tmpui = gpsSol.groundSpeed * 1944 / 100;
                    smartPortSendPackage(id, tmpui);
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_LATLONG    :
                if (STATE(GPS_FIX)) {
                    uint32_t tmpui = 0;
                    // the same ID is sent twice, one for longitude, one for latitude
                    // the MSB of the sent uint32_t helps FrSky keep track
                    // the even/odd bit of our counter helps us keep track
                    if (tableInfo->index & 1) {
                        tmpui = abs(gpsSol.llh.lon);  // now we have unsigned value and one bit to spare
                        tmpui = (tmpui + tmpui / 2) / 25 | 0x80000000;  // 6/100 = 1.5/25, division by power of 2 is fast
                        if (gpsSol.llh.lon < 0) tmpui |= 0x40000000;
                    }
                    else {
                        tmpui = abs(gpsSol.llh.lat);  // now we have unsigned value and one bit to spare
                        tmpui = (tmpui + tmpui / 2) / 25;  // 6/100 = 1.5/25, division by power of 2 is fast
                        if (gpsSol.llh.lat < 0) tmpui |= 0x40000000;
                    }
                    smartPortSendPackage(id, tmpui);
                    *clearToSend = false;
                }
                break;
            case FSSP_DATAID_HOME_DIST  :
                if (STATE(GPS_FIX)) {
                    smartPortSendPackage(id, GPS_distanceToHome);
                     *clearToSend = false;
                }
                break;
            case FSSP_DATAID_GPS_ALT    :
                if (STATE(GPS_FIX)) {
                    smartPortSendPackage(id, gpsSol.llh.altCm); // in cm according to SmartPort spec
                    *clearToSend = false;
                }
                break;
#endif
            case FSSP_DATAID_A4         :
                vfasVoltage = getBatteryAverageCellVoltage(); // in 0.01V according to SmartPort spec
                smartPortSendPackage(id, vfasVoltage);
                *clearToSend = false;
                break;
            default:
                break;
                // if nothing is sent, hasRequest isn't cleared, we already incremented the counter, just loop back to the start
        }
    }
}

static bool serialReadyToSend(void)
{
    return (serialRxBytesWaiting(smartPortSerialPort) == 0);
}

void handleSmartPortTelemetry(void)
{
    const timeUs_t requestTimeout = micros() + SMARTPORT_SERVICE_TIMEOUT_US;

    if (telemetryState == TELEMETRY_STATE_INITIALIZED_SERIAL && smartPortSerialPort) {
        smartPortPayload_t *payload = NULL;
        bool clearToSend = false;
        while (serialRxBytesWaiting(smartPortSerialPort) > 0 && !payload) {
            uint8_t c = serialRead(smartPortSerialPort);
            payload = smartPortDataReceive(c, &clearToSend, serialReadyToSend, true);
        }

            processSmartPortTelemetry(payload, &clearToSend, &requestTimeout);
    }
}
#endif
