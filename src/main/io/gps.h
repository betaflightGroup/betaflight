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

#include "common/axis.h"
#include "common/time.h"

#include "pg/pg.h"

#define GPS_DEGREES_DIVIDER 10000000L
#define GPS_X 1
#define GPS_Y 0

typedef enum {
    GPS_LATITUDE,
    GPS_LONGITUDE
} gpsCoordinateType_e;

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MSP
} gpsProvider_e;

typedef enum {
    SBAS_AUTO = 0,
    SBAS_EGNOS,
    SBAS_WAAS,
    SBAS_MSAS,
    SBAS_GAGAN,
    SBAS_NONE
} sbasMode_e;

#define SBAS_MODE_MAX SBAS_GAGAN

typedef enum {
    UBLOX_AIRBORNE = 0,
    UBLOX_PEDESTRIAN,
    UBLOX_DYNAMIC
} ubloxMode_e;

typedef enum {
    GPS_BAUDRATE_115200 = 0,
    GPS_BAUDRATE_57600,
    GPS_BAUDRATE_38400,
    GPS_BAUDRATE_19200,
    GPS_BAUDRATE_9600
} gpsBaudRate_e;

typedef enum {
    GPS_AUTOCONFIG_OFF = 0,
    GPS_AUTOCONFIG_ON
} gpsAutoConfig_e;

typedef enum {
    GPS_AUTOBAUD_OFF = 0,
    GPS_AUTOBAUD_ON
} gpsAutoBaud_e;

typedef enum {
    UBLOX_ACK_IDLE = 0,
    UBLOX_ACK_WAITING,
    UBLOX_ACK_GOT_ACK,
    UBLOX_ACK_GOT_NACK,
    UBLOX_ACK_GOT_TIMEOUT
} ubloxAckState_e;

#define GPS_BAUDRATE_MAX GPS_BAUDRATE_9600

typedef struct gpsConfig_s {
    gpsProvider_e provider;
    sbasMode_e sbasMode;
    gpsAutoConfig_e autoConfig;
    gpsAutoBaud_e autoBaud;
    uint8_t gps_ublox_use_galileo;
    ubloxMode_e gps_ublox_mode;
    uint8_t gps_set_home_point_once;
    uint8_t gps_use_3d_speed;
    uint8_t sbas_integrity;
} gpsConfig_t;

PG_DECLARE(gpsConfig_t, gpsConfig);

typedef struct gpsCoordinateDDDMMmmmm_s {
    int16_t dddmm;
    int16_t mmmm;
} gpsCoordinateDDDMMmmmm_t;

/* LLH Location in NEU axis system */
typedef struct gpsLocation_s {
    int32_t lat;                    // latitude * 1e+7
    int32_t lon;                    // longitude * 1e+7
    int32_t altCm;                  // altitude in 0.01m
} gpsLocation_t;

typedef struct gpsSolutionData_s {
    gpsLocation_t llh;
    uint16_t speed3d;              // speed in 0.1m/s
    uint16_t groundSpeed;           // speed in 0.1m/s
    uint16_t groundCourse;          // degrees * 10
    uint16_t hdop;                  // generic HDOP value (*100)
    uint8_t numSat;
} gpsSolutionData_t;

typedef enum {
    GPS_MESSAGE_STATE_IDLE = 0,
    GPS_MESSAGE_STATE_INIT,
    GPS_MESSAGE_STATE_SBAS,
    GPS_MESSAGE_STATE_GNSS,
    GPS_MESSAGE_STATE_INITIALIZED,
    GPS_MESSAGE_STATE_PEDESTRIAN_TO_AIRBORNE,
    GPS_MESSAGE_STATE_ENTRY_COUNT
} gpsMessageState_e;

typedef struct gpsData_s {
    uint32_t errors;                // gps error counter - crc error/lost of data/sync etc..
    uint32_t timeouts;
    uint32_t lastMessage;           // last time valid GPS data was received (millis)
    uint32_t lastLastMessage;       // last-last valid GPS message. Used to calculate delta.

    uint32_t state_position;        // incremental variable for loops
    uint32_t state_ts;              // timestamp for last state_position increment
    uint8_t state;                  // GPS thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t baudrateIndex;          // index into auto-detecting or current baudrate
    gpsMessageState_e messageState;

    uint8_t ackWaitingMsgId;        // Message id when waiting for ACK
    uint8_t ackTimeoutCounter;      // Ack timeout counter
    ubloxAckState_e ackState;
} gpsData_t;

#define GPS_PACKET_LOG_ENTRY_COUNT 21 // To make this useful we should log as many packets as we can fit characters a single line of a OLED display.
extern char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];

extern int32_t GPS_home[2];
extern uint16_t GPS_distanceToHome;        // distance to home point in meters
extern int16_t GPS_directionToHome;        // direction to home or hol point in degrees
extern uint32_t GPS_distanceFlownInCm;     // distance flown since armed in centimeters
extern int16_t GPS_verticalSpeedInCmS;     // vertical speed in cm/s
extern int16_t GPS_angle[ANGLE_INDEX_COUNT];                // it's the angles that must be applied for GPS correction
extern float dTnav;             // Delta Time in milliseconds for navigation computations, updated with every good GPS read
extern float GPS_scaleLonDown;  // this is used to offset the shrinking longitude as we go towards the poles
extern int16_t nav_takeoff_bearing;

typedef enum {
    GPS_DIRECT_TICK = 1 << 0,
    GPS_MSP_UPDATE = 1 << 1
} gpsUpdateToggle_e;

extern gpsData_t gpsData;
extern gpsSolutionData_t gpsSol;

extern uint8_t GPS_update;       // toogle to distinct a GPS position update (directly or via MSP)
extern uint32_t GPS_packetCount;
extern uint32_t GPS_svInfoReceivedCount;
extern uint8_t GPS_numCh;                  // Number of channels
extern uint8_t GPS_svinfo_chn[16];         // Channel number
extern uint8_t GPS_svinfo_svid[16];        // Satellite ID
extern uint8_t GPS_svinfo_quality[16];     // Bitfield Qualtity
extern uint8_t GPS_svinfo_cno[16];         // Carrier to Noise Ratio (Signal Strength)

#define GPS_DBHZ_MIN 0
#define GPS_DBHZ_MAX 55

void gpsInit(void);
void gpsUpdate(timeUs_t currentTimeUs);
bool gpsNewFrame(uint8_t c);
bool gpsIsHealthy(void); // Check for healthy communications
struct serialPort_s;
void gpsEnablePassthrough(struct serialPort_s *gpsPassthroughPort);
void onGpsNewData(void);
void GPS_reset_home_position(void);
void GPS_calc_longitude_scaling(int32_t lat);
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing);
void gpsSetFixState(bool state);
