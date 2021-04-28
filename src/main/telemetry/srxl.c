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

#if defined(USE_TELEMETRY_SRXL)

#include "build/version.h"

#include "cms/cms.h"

#include "common/crc.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/vtx_common.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"

#include "io/displayport_srxl.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_tramp.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"
#include "rx/spektrum.h"
#include "io/spektrum_vtx_control.h"
#include "rx/srxl2.h"

#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "telemetry/telemetry.h"

#include "srxl.h"

#define SRXL_ADDRESS_FIRST          0xA5
#define SRXL_ADDRESS_SECOND         0x80
#define SRXL_PACKET_LENGTH          0x15

#define SRXL_FRAMETYPE_TELE_QOS     0x7F
#define SRXL_FRAMETYPE_TELE_RPM     0x7E
#define SRXL_FRAMETYPE_POWERBOX     0x0A
#define SRXL_FRAMETYPE_TELE_FP_MAH  0x34
#define TELE_DEVICE_VTX             0x0D   // Video Transmitter Status
#define SRXL_FRAMETYPE_SID          0x00
#define SRXL_FRAMETYPE_GPS_LOC      0x16   // GPS Location Data (Eagle Tree)
#define SRXL_FRAMETYPE_GPS_STAT     0x17

static bool srxlTelemetryEnabled;
static bool srxl2 = false;
static uint8_t srxlFrame[SRXL_FRAME_SIZE_MAX];

static void srxlInitializeFrame(sbuf_t *dst)
{
    if (srxl2) {
#if defined(USE_SERIALRX_SRXL2)
      srxl2InitializeFrame(dst);
#endif
    } else {
        dst->ptr = srxlFrame;
        dst->end = ARRAYEND(srxlFrame);

        sbufWriteU8(dst, SRXL_ADDRESS_FIRST);
        sbufWriteU8(dst, SRXL_ADDRESS_SECOND);
        sbufWriteU8(dst, SRXL_PACKET_LENGTH);
    }
}

static void srxlFinalize(sbuf_t *dst)
{
    if (srxl2) {
#if defined(USE_SERIALRX_SRXL2)
      srxl2FinalizeFrame(dst);
#endif
    } else {
        crc16_ccitt_sbuf_append(dst, &srxlFrame[3]); // start at byte 3, since CRC does not include device address and packet length
        sbufSwitchToReader(dst, srxlFrame);
        // write the telemetry frame to the receiver.
        srxlRxWriteTelemetryData(sbufPtr(dst), sbufBytesRemaining(dst));
    }
}

/*
SRXL frame has the structure:
<0xA5><0x80><Length><16-byte telemetry packet><2 Byte CRC of payload>
The <Length> shall be 0x15 (length of the 16-byte telemetry packet + overhead).
*/

/*
typedef struct
{
    UINT8 identifier; // Source device = 0x7F
    UINT8 sID; // Secondary ID
    UINT16 A;
    UINT16 B;
    UINT16 L;
    UINT16 R;
    UINT16 F;
    UINT16 H;
    UINT16 rxVoltage; // Volts, 0.01V increments
} STRU_TELE_QOS;
*/

#define STRU_TELE_QOS_EMPTY_FIELDS_COUNT 14
#define STRU_TELE_QOS_EMPTY_FIELDS_VALUE 0xff

bool srxlFrameQos(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_QOS);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);

    sbufFill(dst, STRU_TELE_QOS_EMPTY_FIELDS_VALUE, STRU_TELE_QOS_EMPTY_FIELDS_COUNT); // Clear remainder

    // Mandatory frame, send it unconditionally.
    return true;
}


/*
typedef struct
{
    UINT8 identifier; // Source device = 0x7E
    UINT8 sID; // Secondary ID
    UINT16 microseconds; // microseconds between pulse leading edges
    UINT16 volts; // 0.01V increments
    INT16 temperature; // degrees F
    INT8 dBm_A, // Average signal for A antenna in dBm
    INT8 dBm_B; // Average signal for B antenna in dBm.
    // If only 1 antenna, set B = A
} STRU_TELE_RPM;
*/

#define STRU_TELE_RPM_EMPTY_FIELDS_COUNT 8
#define STRU_TELE_RPM_EMPTY_FIELDS_VALUE 0xff

#define SPEKTRUM_RPM_UNUSED 0xffff
#define SPEKTRUM_TEMP_UNUSED 0x7fff
#define MICROSEC_PER_MINUTE 60000000

//Original range of 1 - 65534 uSec gives an RPM range of 915 - 60000000rpm, 60MegaRPM
#define SPEKTRUM_MIN_RPM 999      // Min RPM to show the user, indicating RPM is really below 999
#define SPEKTRUM_MAX_RPM 60000000

uint16_t getMotorAveragePeriod(void)
{

#if defined( USE_ESC_SENSOR_TELEMETRY) || defined( USE_DSHOT_TELEMETRY)
    uint32_t rpm = 0;
    uint16_t period_us = SPEKTRUM_RPM_UNUSED;

#if defined( USE_ESC_SENSOR_TELEMETRY)
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData != NULL) {
        rpm = escData->rpm;
    }
#endif

#if defined(USE_DSHOT_TELEMETRY)
    if (useDshotTelemetry) {
        uint16_t motors = getMotorCount();

        if (motors > 0) {
            for (int motor = 0; motor < motors; motor++) {
                rpm += getDshotTelemetry(motor);
            }
            rpm = 100.0f / (motorConfig()->motorPoleCount / 2.0f) * rpm;  // convert erpm freq to RPM.
            rpm /= motors;           // Average combined rpm
        }
    }
#endif

    if (rpm > SPEKTRUM_MIN_RPM && rpm < SPEKTRUM_MAX_RPM) {
        period_us = MICROSEC_PER_MINUTE / rpm; // revs/minute -> microSeconds
    } else {
        period_us = MICROSEC_PER_MINUTE / SPEKTRUM_MIN_RPM;
    }

    return period_us;
#else
    return SPEKTRUM_RPM_UNUSED;
#endif
}

bool srxlFrameRpm(sbuf_t *dst, timeUs_t currentTimeUs)
{
    int16_t coreTemp = SPEKTRUM_TEMP_UNUSED;
#if defined(USE_ADC_INTERNAL)
    coreTemp = getCoreTemperatureCelsius();
    coreTemp = coreTemp * 9 / 5 + 32; // C -> F
#endif

    UNUSED(currentTimeUs);

    sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_RPM);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU16BigEndian(dst, getMotorAveragePeriod());    // pulse leading edges
    if (telemetryConfig()->report_cell_voltage) {
        sbufWriteU16BigEndian(dst, getBatteryAverageCellVoltage()); // Cell voltage is in units of 0.01V
    } else {
        sbufWriteU16BigEndian(dst, getBatteryVoltage());   // vbat is in units of 0.01V
    }
    sbufWriteU16BigEndian(dst, coreTemp);                   // temperature
    sbufFill(dst, STRU_TELE_RPM_EMPTY_FIELDS_VALUE, STRU_TELE_RPM_EMPTY_FIELDS_COUNT);

    // Mandatory frame, send it unconditionally.
    return true;
}

#if defined(USE_GPS)

// From Frsky implementation
static void GPStoDDDMM_MMMM(int32_t mwiigps, gpsCoordinateDDDMMmmmm_t *result)
{
    int32_t absgps, deg, min;
    absgps = ABS(mwiigps);
    deg = absgps / GPS_DEGREES_DIVIDER;
    absgps = (absgps - deg * GPS_DEGREES_DIVIDER) * 60;     // absgps = Minutes left * 10^7
    min = absgps / GPS_DEGREES_DIVIDER;                     // minutes left
    result->dddmm = deg * 100 + min;
    result->mmmm = (absgps - min * GPS_DEGREES_DIVIDER) / 1000;
}

// BCD conversion
static uint32_t dec2bcd(uint16_t dec)
{
    uint32_t result = 0;
    uint8_t counter = 0;

    while (dec) {
        result |= (dec % 10) << counter * 4;
        counter++;
        dec /= 10;
    }
    return result;
}

/*
typedef struct
{
    UINT8    identifier;    // Source device = 0x16
    UINT8    sID;           // Secondary ID
    UINT16   altitudeLow;   // BCD, meters, format 3.1 (Low order of altitude)
    UINT32   latitude;      // BCD, format 4.4, Degrees * 100 + minutes, less than 100 degrees
    UINT32   longitude;     // BCD, format 4.4 , Degrees * 100 + minutes, flag indicates > 99 degrees
    UINT16   course;        // BCD, 3.1
    UINT8    HDOP;          // BCD, format 1.1
    UINT8    GPSflags;      // see definitions below
} STRU_TELE_GPS_LOC;
*/

// GPS flags definitions
#define GPS_FLAGS_IS_NORTH_BIT              0x01
#define GPS_FLAGS_IS_EAST_BIT               0x02
#define GPS_FLAGS_LONGITUDE_GREATER_99_BIT  0x04
#define GPS_FLAGS_GPS_FIX_VALID_BIT         0x08
#define GPS_FLAGS_GPS_DATA_RECEIVED_BIT     0x10
#define GPS_FLAGS_3D_FIX_BIT                0x20
#define GPS_FLAGS_NEGATIVE_ALT_BIT          0x80

bool srxlFrameGpsLoc(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    gpsCoordinateDDDMMmmmm_t coordinate;
    uint32_t latitudeBcd, longitudeBcd, altitudeLo;
    uint16_t altitudeLoBcd, groundCourseBcd, hdop;
    uint8_t hdopBcd, gpsFlags;

    if (!featureIsEnabled(FEATURE_GPS) || !STATE(GPS_FIX) || gpsSol.numSat < 6) {
        return false;
    }

    // lattitude
    GPStoDDDMM_MMMM(gpsSol.llh.lat, &coordinate);
    latitudeBcd  = (dec2bcd(coordinate.dddmm) << 16) | dec2bcd(coordinate.mmmm);

    // longitude
    GPStoDDDMM_MMMM(gpsSol.llh.lon, &coordinate);
    longitudeBcd = (dec2bcd(coordinate.dddmm) << 16) | dec2bcd(coordinate.mmmm);

    // altitude (low order)
    altitudeLo = ABS(gpsSol.llh.altCm) / 10;
    altitudeLoBcd = dec2bcd(altitudeLo % 100000);

    // Ground course
    groundCourseBcd = dec2bcd(gpsSol.groundCourse);

    // HDOP
    hdop = gpsSol.hdop / 10;
    hdop = (hdop > 99) ? 99 : hdop;
    hdopBcd = dec2bcd(hdop);

    // flags
    gpsFlags = GPS_FLAGS_GPS_DATA_RECEIVED_BIT | GPS_FLAGS_GPS_FIX_VALID_BIT | GPS_FLAGS_3D_FIX_BIT;
    gpsFlags |= (gpsSol.llh.lat > 0) ? GPS_FLAGS_IS_NORTH_BIT : 0;
    gpsFlags |= (gpsSol.llh.lon > 0) ? GPS_FLAGS_IS_EAST_BIT : 0;
    gpsFlags |= (gpsSol.llh.altCm < 0) ? GPS_FLAGS_NEGATIVE_ALT_BIT : 0;
    gpsFlags |= (gpsSol.llh.lon / GPS_DEGREES_DIVIDER > 99) ? GPS_FLAGS_LONGITUDE_GREATER_99_BIT : 0;

    // SRXL frame
    sbufWriteU8(dst, SRXL_FRAMETYPE_GPS_LOC);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU16(dst, altitudeLoBcd);
    sbufWriteU32(dst, latitudeBcd);
    sbufWriteU32(dst, longitudeBcd);
    sbufWriteU16(dst, groundCourseBcd);
    sbufWriteU8(dst, hdopBcd);
    sbufWriteU8(dst, gpsFlags);

    return true;
}

/*
typedef struct
{
   UINT8   identifier;                      // Source device = 0x17
   UINT8   sID;                             // Secondary ID
   UINT16  speed;                           // BCD, knots, format 3.1
   UINT32  UTC;                             // BCD, format HH:MM:SS.S, format 6.1
   UINT8   numSats;                         // BCD, 0-99
   UINT8   altitudeHigh;                    // BCD, meters, format 2.0 (High bits alt)
} STRU_TELE_GPS_STAT;
*/

#define STRU_TELE_GPS_STAT_EMPTY_FIELDS_VALUE 0xff
#define STRU_TELE_GPS_STAT_EMPTY_FIELDS_COUNT 6
#define SPEKTRUM_TIME_UNKNOWN 0xFFFFFFFF

bool srxlFrameGpsStat(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    uint32_t timeBcd;
    uint16_t speedKnotsBcd, speedTmp;
    uint8_t numSatBcd, altitudeHighBcd;
    bool timeProvided = false;

    if (!featureIsEnabled(FEATURE_GPS) || !STATE(GPS_FIX) || gpsSol.numSat < 6) {
        return false;
    }

    // Number of sats and altitude (high bits)
    numSatBcd = (gpsSol.numSat > 99) ? dec2bcd(99) : dec2bcd(gpsSol.numSat);
    altitudeHighBcd = dec2bcd(gpsSol.llh.altCm / 100000);

    // Speed (knots)
    speedTmp = gpsSol.groundSpeed * 1944 / 1000;
    speedKnotsBcd = (speedTmp > 9999) ? dec2bcd(9999) : dec2bcd(speedTmp);

#ifdef USE_RTC_TIME
    dateTime_t dt;
    // RTC
    if (rtcHasTime()) {
        rtcGetDateTime(&dt);
        timeBcd = dec2bcd(dt.hours);
        timeBcd = timeBcd << 8;
        timeBcd = timeBcd | dec2bcd(dt.minutes);
        timeBcd = timeBcd << 8;
        timeBcd = timeBcd | dec2bcd(dt.seconds);
        timeBcd = timeBcd << 4;
        timeBcd = timeBcd | dec2bcd(dt.millis / 100);
        timeProvided = true;
    }
#endif
    timeBcd = (timeProvided) ? timeBcd : SPEKTRUM_TIME_UNKNOWN;

    // SRXL frame
    sbufWriteU8(dst, SRXL_FRAMETYPE_GPS_STAT);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU16(dst, speedKnotsBcd);
    sbufWriteU32(dst, timeBcd);
    sbufWriteU8(dst, numSatBcd);
    sbufWriteU8(dst, altitudeHighBcd);
    sbufFill(dst, STRU_TELE_GPS_STAT_EMPTY_FIELDS_VALUE, STRU_TELE_GPS_STAT_EMPTY_FIELDS_COUNT);

    return true;
}

#endif

/*
typedef struct
{
    UINT8   identifier;     // Source device = 0x34
    UINT8   sID;            // Secondary ID
    INT16   current_A;      // Instantaneous current, 0.1A (0-3276.8A)
    INT16   chargeUsed_A;   // Integrated mAh used, 1mAh (0-32.766Ah)
    UINT16  temp_A;         // Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
    INT16   current_B;      // Instantaneous current, 0.1A (0-3276.8A)
    INT16   chargeUsed_B;   // Integrated mAh used, 1mAh (0-32.766Ah)
    UINT16  temp_B;         // Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
    UINT16  spare;          // Not used
} STRU_TELE_FP_MAH;
*/
#define STRU_TELE_FP_EMPTY_FIELDS_COUNT 2
#define STRU_TELE_FP_EMPTY_FIELDS_VALUE 0xff

#define SPEKTRUM_AMPS_UNUSED 0x7fff
#define SPEKTRUM_AMPH_UNUSED 0x7fff

#define FP_MAH_KEEPALIVE_TIME_OUT 2000000 // 2s

bool srxlFrameFlightPackCurrent(sbuf_t *dst, timeUs_t currentTimeUs)
{
    uint16_t amps = getAmperage() / 10;
    uint16_t mah  = getMAhDrawn();
    static uint16_t sentAmps;
    static uint16_t sentMah;
    static timeUs_t lastTimeSentFPmAh = 0;

    timeUs_t keepAlive = currentTimeUs - lastTimeSentFPmAh;

    if ( amps != sentAmps ||
         mah != sentMah ||
         keepAlive > FP_MAH_KEEPALIVE_TIME_OUT ) {

        sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_FP_MAH);
        sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
        sbufWriteU16(dst, amps);
        sbufWriteU16(dst, mah);
        sbufWriteU16(dst, SPEKTRUM_TEMP_UNUSED);            // temp A
        sbufWriteU16(dst, SPEKTRUM_AMPS_UNUSED);            // Amps B
        sbufWriteU16(dst, SPEKTRUM_AMPH_UNUSED);            // mAH B
        sbufWriteU16(dst, SPEKTRUM_TEMP_UNUSED);            // temp B

        sbufFill(dst, STRU_TELE_FP_EMPTY_FIELDS_VALUE, STRU_TELE_FP_EMPTY_FIELDS_COUNT);

        sentAmps = amps;
        sentMah = mah;
        lastTimeSentFPmAh = currentTimeUs;
        return true;
    }
    return false;
}

#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)

// Betaflight CMS using Spektrum Tx telemetry TEXT_GEN sensor as display.

#define SPEKTRUM_SRXL_DEVICE_TEXTGEN (0x0C)     // Text Generator
#define SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS (9)   // Text Generator ROWS
#define SPEKTRUM_SRXL_DEVICE_TEXTGEN_COLS (13)  // Text Generator COLS

/*
typedef struct
{
    UINT8       identifier;
    UINT8       sID;               // Secondary ID
    UINT8       lineNumber;        // Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
    char        text[13];          // 0-terminated text when < 13 chars
} STRU_SPEKTRUM_SRXL_TEXTGEN;
*/

#if ( SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS > SPEKTRUM_SRXL_DEVICE_TEXTGEN_COLS )
static char srxlTextBuff[SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS][SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS];
static bool lineSent[SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS];
#else
static char srxlTextBuff[SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS][SPEKTRUM_SRXL_DEVICE_TEXTGEN_COLS];
static bool lineSent[SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS];
#endif

//**************************************************************************
// API Running in external client task context. E.g. in the CMS task
int spektrumTmTextGenPutChar(uint8_t col, uint8_t row, char c)
{
    if (row < SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS && col < SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS) {
      // Only update and force a tm transmision if something has actually changed.
        if (srxlTextBuff[row][col] != c) {
          srxlTextBuff[row][col] = c;
          lineSent[row] = false;
        }
    }
    return 0;
}
//**************************************************************************

bool srxlFrameText(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    static uint8_t lineNo = 0;
    int lineCount = 0;

    // Skip already sent lines...
    while (lineSent[lineNo] &&
           lineCount < SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS) {
        lineNo = (lineNo + 1) % SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS;
        lineCount++;
    }

    sbufWriteU8(dst, SPEKTRUM_SRXL_DEVICE_TEXTGEN);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU8(dst, lineNo);
    sbufWriteData(dst, srxlTextBuff[lineNo], SPEKTRUM_SRXL_DEVICE_TEXTGEN_COLS);

    lineSent[lineNo] = true;
    lineNo = (lineNo + 1) % SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS;

    // Always send something, Always one user frame after the two mandatory frames
    // I.e. All of the three frame prep routines QOS, RPM, TEXT should always return true
    // too keep the "Waltz" sequence intact.
    return true;
}
#endif

#if defined(USE_SPEKTRUM_VTX_TELEMETRY) && defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)

static uint8_t vtxDeviceType;

static void collectVtxTmData(spektrumVtx_t * vtx)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    vtxDeviceType = vtxCommonGetDeviceType(vtxDevice);

    // Collect all data from VTX, if VTX is ready
    unsigned vtxStatus;
    if (vtxDevice == NULL || !(vtxCommonGetBandAndChannel(vtxDevice, &vtx->band, &vtx->channel) &&
           vtxCommonGetStatus(vtxDevice, &vtxStatus) &&
           vtxCommonGetPowerIndex(vtxDevice, &vtx->power)) )
        {
            vtx->band    = 0;
            vtx->channel = 0;
            vtx->power   = 0;
            vtx->pitMode = 0;
        } else {
            vtx->pitMode = (vtxStatus & VTX_STATUS_PIT_MODE) ? 1 : 0;
        }

    vtx->powerValue = 0;
#ifdef USE_SPEKTRUM_REGION_CODES
    vtx->region = SpektrumRegion;
#else
    vtx->region = SPEKTRUM_VTX_REGION_NONE;
#endif
}

// Reverse lookup, device power index to Spektrum power range index.
static void convertVtxPower(spektrumVtx_t * vtx)
    {
        uint8_t const * powerIndexTable = NULL;

        vtxCommonLookupPowerValue(vtxCommonDevice(), vtx->power, &vtx->powerValue);
        switch (vtxDeviceType) {

#if defined(USE_VTX_TRAMP)
        case VTXDEV_TRAMP:
            powerIndexTable = vtxTrampPi;
            break;
#endif
#if defined(USE_VTX_SMARTAUDIO)
        case VTXDEV_SMARTAUDIO:
            powerIndexTable = vtxSaPi;
            break;
#endif
#if defined(USE_VTX_RTC6705)
        case VTXDEV_RTC6705:
            powerIndexTable = vtxRTC6705Pi;
            break;
#endif

        case VTXDEV_UNKNOWN:
        case VTXDEV_UNSUPPORTED:
        default:
          break;

        }

        if (powerIndexTable != NULL) {
            for (int i = 0; i < SPEKTRUM_VTX_POWER_COUNT; i++)
                if (powerIndexTable[i] >= vtx->power) {
                    vtx->power = i;                                    // Translate device power index to Spektrum power index.
                    break;
                }
        }
    }

static void convertVtxTmData(spektrumVtx_t * vtx)
{
    // Convert from internal band indexes to Spektrum indexes
    for (int i = 0; i < SPEKTRUM_VTX_BAND_COUNT; i++) {
        if (spek2commonBand[i] == vtx->band) {
            vtx->band = i;
            break;
        }
    }

    // De-bump channel no 1 based interally, 0-based in Spektrum.
    vtx->channel--;

    // Convert Power index to Spektrum ranges, different per brand.
    convertVtxPower(vtx);
}

/*
typedef struct
{
    UINT8		identifier;
    UINT8		sID;	  // Secondary ID
    UINT8		band;	  // VTX Band (0 = Fatshark, 1 = Raceband, 2 = E, 3 = B, 4 = A, 5-7 = Reserved)
    UINT8		channel;  // VTX Channel (0-7)
    UINT8		pit;	  // Pit/Race mode (0 = Race, 1 = Pit). Race = (normal operating) mode. Pit = (reduced power) mode. When PIT is set, it overrides all other power settings
    UINT8		power;	  // VTX Power (0 = Off, 1 = 1mw to 14mW, 2 = 15mW to 25mW, 3 = 26mW to 99mW, 4 = 100mW to 299mW, 5 = 300mW to 600mW, 6 = 601mW+, 7 = manual control)
    UINT16		powerDec; // VTX Power as a decimal 1mw/unit
    UINT8		region;	  // Region (0 = USA, 1 = EU, 0xFF = N/A)
    UINT8		rfu[7];	  // reserved
} STRU_TELE_VTX;
*/

#define STRU_TELE_VTX_EMPTY_COUNT 7
#define STRU_TELE_VTX_EMPTY_VALUE 0xff

#define VTX_KEEPALIVE_TIME_OUT 2000000 // uS

static bool srxlFrameVTX(sbuf_t *dst, timeUs_t currentTimeUs)
{
    static timeUs_t lastTimeSentVtx = 0;
    static spektrumVtx_t vtxSent;

    spektrumVtx_t vtx;
    collectVtxTmData(&vtx);

    if ((vtxDeviceType != VTXDEV_UNKNOWN) && vtxDeviceType != VTXDEV_UNSUPPORTED) {
        convertVtxTmData(&vtx);

        if ((memcmp(&vtxSent, &vtx, sizeof(spektrumVtx_t)) != 0) ||
            ((currentTimeUs - lastTimeSentVtx) > VTX_KEEPALIVE_TIME_OUT) ) {
            // Fill in the VTX tm structure
            sbufWriteU8(dst, TELE_DEVICE_VTX);
            sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
            sbufWriteU8(dst,  vtx.band);
            sbufWriteU8(dst,  vtx.channel);
            sbufWriteU8(dst,  vtx.pitMode);
            sbufWriteU8(dst,  vtx.power);
            sbufWriteU16(dst, vtx.powerValue);
            sbufWriteU8(dst,  vtx.region);

            sbufFill(dst, STRU_TELE_VTX_EMPTY_VALUE, STRU_TELE_VTX_EMPTY_COUNT);

            memcpy(&vtxSent, &vtx, sizeof(spektrumVtx_t));
            lastTimeSentVtx = currentTimeUs;
            return true;
        }
    }
    return false;
}
#endif // USE_SPEKTRUM_VTX_TELEMETRY && USE_SPEKTRUM_VTX_CONTROL && USE_VTX_COMMON


// Schedule array to decide how often each type of frame is sent
// The frames are scheduled in sets of 3 frames, 2 mandatory and 1 user frame.
// The user frame type is cycled for each set.
// Example. QOS, RPM,.CURRENT, QOS, RPM, TEXT. QOS, RPM, CURRENT, etc etc

#define SRXL_SCHEDULE_MANDATORY_COUNT  2 // Mandatory QOS and RPM sensors

#define SRXL_FP_MAH_COUNT   1

#if defined(USE_GPS)
#define SRXL_GPS_LOC_COUNT  1
#define SRXL_GPS_STAT_COUNT 1
#else
#define SRXL_GPS_LOC_COUNT  0
#define SRXL_GPS_STAT_COUNT 0
#endif

#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)
#define SRXL_SCHEDULE_CMS_COUNT  1
#else
#define SRXL_SCHEDULE_CMS_COUNT  0
#endif

#if defined(USE_SPEKTRUM_VTX_TELEMETRY) && defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
#define SRXL_VTX_TM_COUNT        1
#else
#define SRXL_VTX_TM_COUNT        0
#endif

#define SRXL_SCHEDULE_USER_COUNT (SRXL_FP_MAH_COUNT + SRXL_SCHEDULE_CMS_COUNT + SRXL_VTX_TM_COUNT + SRXL_GPS_LOC_COUNT + SRXL_GPS_STAT_COUNT)
#define SRXL_SCHEDULE_COUNT_MAX  (SRXL_SCHEDULE_MANDATORY_COUNT + 1)
#define SRXL_TOTAL_COUNT         (SRXL_SCHEDULE_MANDATORY_COUNT + SRXL_SCHEDULE_USER_COUNT)

typedef bool (*srxlScheduleFnPtr)(sbuf_t *dst, timeUs_t currentTimeUs);

const srxlScheduleFnPtr srxlScheduleFuncs[SRXL_TOTAL_COUNT] = {
    /* must send srxlFrameQos, Rpm and then alternating items of our own */
    srxlFrameQos,
    srxlFrameRpm,
    srxlFrameFlightPackCurrent,
#if defined(USE_GPS)
    srxlFrameGpsStat,
    srxlFrameGpsLoc,
#endif
#if defined(USE_SPEKTRUM_VTX_TELEMETRY) && defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
    srxlFrameVTX,
#endif
#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)
    srxlFrameText,
#endif
};


static void processSrxl(timeUs_t currentTimeUs)
{
    static uint8_t srxlScheduleIndex = 0;
    static uint8_t srxlScheduleUserIndex = 0;

    sbuf_t srxlPayloadBuf;
    sbuf_t *dst = &srxlPayloadBuf;
    srxlScheduleFnPtr srxlFnPtr;

    if (srxlScheduleIndex < SRXL_SCHEDULE_MANDATORY_COUNT) {
        srxlFnPtr = srxlScheduleFuncs[srxlScheduleIndex];
    } else {
        srxlFnPtr = srxlScheduleFuncs[srxlScheduleIndex + srxlScheduleUserIndex];
        srxlScheduleUserIndex = (srxlScheduleUserIndex + 1) % SRXL_SCHEDULE_USER_COUNT;

#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)
        // Boost CMS performance by sending nothing else but CMS Text frames when in a CMS menu.
        // Sideeffect, all other reports are still not sent if user leaves CMS without a proper EXIT.
        if (cmsInMenu &&
            (pCurrentDisplay == &srxlDisplayPort)) {
            srxlFnPtr = srxlFrameText;
        }
#endif

    }

    if (srxlFnPtr) {
        srxlInitializeFrame(dst);
        if (srxlFnPtr(dst, currentTimeUs)) {
            srxlFinalize(dst);
        }
    }
    srxlScheduleIndex = (srxlScheduleIndex + 1) % SRXL_SCHEDULE_COUNT_MAX;
}

void initSrxlTelemetry(void)
{
    // check if there is a serial port open for SRXL telemetry (ie opened by the SRXL RX)
    // and feature is enabled, if so, set SRXL telemetry enabled
    if (srxlRxIsActive()) {
        srxlTelemetryEnabled = true;
        srxl2 = false;
#if defined(USE_SERIALRX_SRXL2)
    } else if (srxl2RxIsActive()) {
        srxlTelemetryEnabled = true;
        srxl2 = true;
#endif
    } else {
        srxlTelemetryEnabled = false;
        srxl2 = false;
    }

#if defined(USE_SPEKTRUM_CMS_TELEMETRY)
    if (srxlTelemetryEnabled) {
        srxlDisplayportRegister();
    }
#endif
 }

bool checkSrxlTelemetryState(void)
{
    return srxlTelemetryEnabled;
}

/*
 * Called periodically by the scheduler
 */
void handleSrxlTelemetry(timeUs_t currentTimeUs)
{
  if (srxl2) {
#if defined(USE_SERIALRX_SRXL2)
      if (srxl2TelemetryRequested()) {
          processSrxl(currentTimeUs);
      }
#endif
  } else {
      if (srxlTelemetryBufferEmpty()) {
          processSrxl(currentTimeUs);
      }
  }
}
#endif
