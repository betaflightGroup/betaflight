/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <limits.h>

extern "C" {
    #include <platform.h>

    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/crc.h"
    #include "common/filter.h"
    #include "common/gps_conversion.h"
    #include "common/maths.h"
    #include "common/printf.h"
    #include "common/typeconversion.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "drivers/serial.h"
    #include "drivers/system.h"

    #include "config/config.h"
    #include "fc/runtime_config.h"

    #include "flight/pid.h"
    #include "flight/imu.h"

    #include "io/gps.h"
    #include "io/serial.h"

    #include "rx/rx.h"
    #include "rx/crsf.h"

    #include "sensors/battery.h"
    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "telemetry/crsf.h"
    #include "telemetry/telemetry.h"
    #include "telemetry/msp_shared.h"

    rssiSource_e rssiSource;
    bool airMode;

    uint16_t testBatteryVoltage = 0;
    int32_t testAmperage = 0;
    int32_t testmAhDrawn = 0;

    serialPort_t *telemetrySharedPort;

    int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType);

    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);

    timeUs_t rxFrameTimeUs(void) { return 0; }
    rxRuntimeState_t rxRuntimeState;
    bool rxSerialPortIsActive(SerialRXType) { return true; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint8_t crfsCrc(uint8_t *frame, int frameLen)
{
    uint8_t crc = 0;
    for (int ii = 2; ii < frameLen - 1; ++ii) {
        crc = crc8_dvb_s2(crc, frame[ii]);
    }
    return crc;
}

/*
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 10 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­ 1000m offset )
uint8_t     Satellites in use ( counter )
uint16_t GPS_distanceToHome;        // distance to home point in meters
*/
#define FRAME_HEADER_FOOTER_LEN 4

TEST(TelemetryCrsfTest, TestGPS)
{
    uint8_t frame[CRSF_FRAME_SIZE_MAX];

    int frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_GPS);
    EXPECT_EQ(CRSF_FRAME_GPS_PAYLOAD_SIZE + FRAME_HEADER_FOOTER_LEN, frameLen);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]); // address
    EXPECT_EQ(17, frame[1]); // length
    EXPECT_EQ(0x02, frame[2]); // type
    int32_t lattitude = frame[3] << 24 | frame[4] << 16 | frame[5] << 8 | frame[6];
    EXPECT_EQ(0, lattitude);
    int32_t longitude = frame[7] << 24 | frame[8] << 16 | frame[9] << 8 | frame[10];
    EXPECT_EQ(0, longitude);
    uint16_t groundSpeed = frame[11] << 8 | frame[12];
    EXPECT_EQ(0, groundSpeed);
    uint16_t GPSheading = frame[13] << 8 | frame[14];
    EXPECT_EQ(0, GPSheading);
    uint16_t altitude = frame[15] << 8 | frame[16];
    EXPECT_EQ(1000, altitude);
    uint8_t satelliteCount = frame[17];
    EXPECT_EQ(0, satelliteCount);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[18]);

    gpsSol.llh.lat = 56 * GPS_DEGREES_DIVIDER;
    gpsSol.llh.lon = 163 * GPS_DEGREES_DIVIDER;
    ENABLE_STATE(GPS_FIX);
    gpsSol.llh.altCm = 2345 * 100;            // altitude in cm / 100 + 1000m offset, so CRSF value should be 3345
    gpsSol.groundSpeed = 1630;                // speed in cm/s, 16.3 m/s = 58.68 km/h, so CRSF (km/h *10) value is 587
    gpsSol.numSat = 9;
    gpsSol.groundCourse = 1479;     // degrees * 10
    frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_GPS);
    lattitude = frame[3] << 24 | frame[4] << 16 | frame[5] << 8 | frame[6];
    EXPECT_EQ(560000000, lattitude);
    longitude = frame[7] << 24 | frame[8] << 16 | frame[9] << 8 | frame[10];
    EXPECT_EQ(1630000000, longitude);
    groundSpeed = frame[11] << 8 | frame[12];
    EXPECT_EQ(587, groundSpeed);
    GPSheading = frame[13] << 8 | frame[14];
    EXPECT_EQ(14790, GPSheading);
    altitude = frame[15] << 8 | frame[16];
    EXPECT_EQ(3345, altitude);
    satelliteCount = frame[17];
    EXPECT_EQ(9, satelliteCount);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[18]);
}

TEST(TelemetryCrsfTest, TestBattery)
{
    uint8_t frame[CRSF_FRAME_SIZE_MAX];

    testBatteryVoltage = 0; // 0.1V units
    int frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_BATTERY_SENSOR);
    EXPECT_EQ(CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + FRAME_HEADER_FOOTER_LEN, frameLen);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]); // address
    EXPECT_EQ(10, frame[1]); // length
    EXPECT_EQ(0x08, frame[2]); // type
    uint16_t voltage = frame[3] << 8 | frame[4]; // mV * 100
    EXPECT_EQ(0, voltage);
    uint16_t current = frame[5] << 8 | frame[6]; // mA * 100
    EXPECT_EQ(0, current);
    uint32_t capacity = frame[7] << 16 | frame[8] << 8 | frame [9]; // mAh
    EXPECT_EQ(0, capacity);
    uint16_t remaining = frame[10]; // percent
    EXPECT_EQ(67, remaining);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[11]);

    testBatteryVoltage = 330; // 3.3V = 3300 mv
    testAmperage = 2960; // = 29.60A = 29600mA - amperage is in 0.01A steps
    testmAhDrawn = 1234;
    frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_BATTERY_SENSOR);
    voltage = frame[3] << 8 | frame[4]; // mV * 100
    EXPECT_EQ(33, voltage);
    current = frame[5] << 8 | frame[6]; // mA * 100
    EXPECT_EQ(296, current);
    capacity = frame[7] << 16 | frame[8] << 8 | frame [9]; // mAh
    EXPECT_EQ(1234, capacity);
    remaining = frame[10]; // percent
    EXPECT_EQ(67, remaining);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[11]);
}

TEST(TelemetryCrsfTest, TestAttitude)
{
    uint8_t frame[CRSF_FRAME_SIZE_MAX];

    attitude.values.pitch = 0;
    attitude.values.roll = 0;
    attitude.values.yaw = 0;
    int frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_ATTITUDE);
    EXPECT_EQ(CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + FRAME_HEADER_FOOTER_LEN, frameLen);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]); // address
    EXPECT_EQ(8, frame[1]); // length
    EXPECT_EQ(0x1e, frame[2]); // type
    int16_t pitch = frame[3] << 8 | frame[4]; // rad / 10000
    EXPECT_EQ(0, pitch);
    int16_t roll = frame[5] << 8 | frame[6];
    EXPECT_EQ(0, roll);
    int16_t yaw = frame[7] << 8 | frame[8];
    EXPECT_EQ(0, yaw);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[9]);

    attitude.values.pitch = 678; // decidegrees == 1.183333232852155 rad
    attitude.values.roll = 1495; // 2.609267231731523 rad
    attitude.values.yaw = -1799; //3.139847324337799 rad
    frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_ATTITUDE);
    pitch = frame[3] << 8 | frame[4]; // rad / 10000
    EXPECT_EQ(11833, pitch);
    roll = frame[5] << 8 | frame[6];
    EXPECT_EQ(26092, roll);
    yaw = frame[7] << 8 | frame[8];
    EXPECT_EQ(-31398, yaw);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[9]);
}

TEST(TelemetryCrsfTest, TestFlightMode)
{
    uint8_t frame[CRSF_FRAME_SIZE_MAX];

    ENABLE_STATE(GPS_FIX);
    ENABLE_STATE(GPS_FIX_HOME);

    airMode = false;

    DISABLE_ARMING_FLAG(ARMED);

    // nothing set, so ACRO mode
    int frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_FLIGHT_MODE);
    EXPECT_EQ(6 + FRAME_HEADER_FOOTER_LEN, frameLen);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]); // address
    EXPECT_EQ(8, frame[1]); // length
    EXPECT_EQ(0x21, frame[2]); // type
    EXPECT_EQ('A', frame[3]);
    EXPECT_EQ('C', frame[4]);
    EXPECT_EQ('R', frame[5]);
    EXPECT_EQ('O', frame[6]);
    EXPECT_EQ('*', frame[7]);
    EXPECT_EQ(0, frame[8]);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[9]);

    ENABLE_ARMING_FLAG(ARMED);

    frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_FLIGHT_MODE);
    EXPECT_EQ(5 + FRAME_HEADER_FOOTER_LEN, frameLen);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]); // address
    EXPECT_EQ(7, frame[1]); // length
    EXPECT_EQ(0x21, frame[2]); // type
    EXPECT_EQ('A', frame[3]);
    EXPECT_EQ('C', frame[4]);
    EXPECT_EQ('R', frame[5]);
    EXPECT_EQ('O', frame[6]);
    EXPECT_EQ(0, frame[7]);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[8]);

    enableFlightMode(ANGLE_MODE);
    EXPECT_EQ(ANGLE_MODE, FLIGHT_MODE(ANGLE_MODE));
    frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_FLIGHT_MODE);
    EXPECT_EQ(5 + FRAME_HEADER_FOOTER_LEN, frameLen);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]); // address
    EXPECT_EQ(7, frame[1]); // length
    EXPECT_EQ(0x21, frame[2]); // type
    EXPECT_EQ('S', frame[3]);
    EXPECT_EQ('T', frame[4]);
    EXPECT_EQ('A', frame[5]);
    EXPECT_EQ('B', frame[6]);
    EXPECT_EQ(0, frame[7]);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[8]);

    disableFlightMode(ANGLE_MODE);
    enableFlightMode(HORIZON_MODE);
    EXPECT_EQ(HORIZON_MODE, FLIGHT_MODE(HORIZON_MODE));
    frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_FLIGHT_MODE);
    EXPECT_EQ(4 + FRAME_HEADER_FOOTER_LEN, frameLen);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]); // address
    EXPECT_EQ(6, frame[1]); // length
    EXPECT_EQ(0x21, frame[2]); // type
    EXPECT_EQ('H', frame[3]);
    EXPECT_EQ('O', frame[4]);
    EXPECT_EQ('R', frame[5]);
    EXPECT_EQ(0, frame[6]);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[7]);

    disableFlightMode(HORIZON_MODE);
    airMode = true;
    frameLen = getCrsfFrame(frame, CRSF_FRAMETYPE_FLIGHT_MODE);
    EXPECT_EQ(4 + FRAME_HEADER_FOOTER_LEN, frameLen);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]); // address
    EXPECT_EQ(6, frame[1]); // length
    EXPECT_EQ(0x21, frame[2]); // type
    EXPECT_EQ('A', frame[3]);
    EXPECT_EQ('I', frame[4]);
    EXPECT_EQ('R', frame[5]);
    EXPECT_EQ(0, frame[6]);
    EXPECT_EQ(crfsCrc(frame, frameLen), frame[7]);
}

// STUBS

extern "C" {

int16_t debug[DEBUG16_VALUE_COUNT];

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000}; // see baudRate_e

uint16_t batteryWarningVoltage;
uint8_t useHottAlarmSoundPeriod (void) { return 0; }

attitudeEulerAngles_t attitude = { { 0, 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

uint16_t GPS_distanceToHome;        // distance to home point in meters
gpsSolutionData_t gpsSol;

void beeperConfirmationBeeps(uint8_t beepCount) {UNUSED(beepCount);}

uint32_t micros(void) {return 0;}
uint32_t microsISR(void) {return micros();}

bool featureIsEnabled(uint32_t) {return true;}

uint32_t serialRxBytesWaiting(const serialPort_t *) {return 0;}
uint32_t serialTxBytesFree(const serialPort_t *) {return 0;}
uint8_t serialRead(serialPort_t *) {return 0;}
void serialWrite(serialPort_t *, uint8_t) {}
void serialWriteBuf(serialPort_t *, const uint8_t *, int) {}
void serialSetMode(serialPort_t *, portMode_e) {}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
void closeSerialPort(serialPort_t *) {}
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }

const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) {return NULL;}

bool telemetryDetermineEnabledState(portSharing_e) {return true;}
bool telemetryCheckRxPortShared(const serialPortConfig_t *, SerialRXType) {return true;}
bool telemetryIsSensorEnabled(sensor_e) {return true;}

portSharing_e determinePortSharing(const serialPortConfig_t *, serialPortFunction_e) {return PORTSHARING_NOT_SHARED;}

bool airmodeIsEnabled(void) {return airMode;}

int32_t getAmperage(void) {
    return testAmperage;
}

uint16_t getBatteryVoltage(void) {
    return testBatteryVoltage;
}

uint16_t getLegacyBatteryVoltage(void) {
    return (testBatteryVoltage + 5) / 10;
}

uint16_t getBatteryAverageCellVoltage(void) {
    return 0;
}

batteryState_e getBatteryState(void) {
    return BATTERY_OK;
}

uint8_t calculateBatteryPercentageRemaining(void) {
    return 67;
}

int32_t getEstimatedAltitudeCm(void) {
	return gpsSol.llh.altCm;    // function returns cm not m.
}
    
int32_t getMAhDrawn(void){
  return testmAhDrawn;
}

bool sendMspReply(uint8_t, mspResponseFnPtr) { return false; }
bool handleMspFrame(uint8_t *, int, uint8_t *)  { return false; }
void crsfScheduleMspResponse(void) {};
bool isBatteryVoltageConfigured(void) { return true; }
bool isAmperageConfigured(void) { return true; }

}
