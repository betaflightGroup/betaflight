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

#include "common/time.h"

#include "pg/pg.h"
#include "pg/rx.h"

#include "drivers/io_types.h"
#include "drivers/serial.h"

#include "rx/ghst_protocol.h"
#include "rx/sbus.h"

#define STICK_CHANNEL_COUNT 4

#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE (PWM_RANGE_MAX - PWM_RANGE_MIN)
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + (PWM_RANGE / 2))

#define PWM_PULSE_MIN   750       // minimum PWM pulse width which is considered valid
#define PWM_PULSE_MAX   2250      // maximum PWM pulse width which is considered valid

#define RXFAIL_STEP_TO_CHANNEL_VALUE(step) (PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) ((constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)
#define MAX_RXFAIL_RANGE_STEP ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 25)

#define DEFAULT_SERVO_MIN 1000
#define DEFAULT_SERVO_MIDDLE 1500
#define DEFAULT_SERVO_MAX 2000

typedef enum {
    RX_FRAME_PENDING = 0,
    RX_FRAME_COMPLETE = (1 << 0),
    RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
    RX_FRAME_DROPPED = (1 << 3)
} rxFrameState_e;

typedef enum {
    SERIALRX_SPEKTRUM1024 = 0,
    SERIALRX_SPEKTRUM2048 = 1,
    SERIALRX_SBUS = 2,
    SERIALRX_SUMD = 3,
    SERIALRX_SUMH = 4,
    SERIALRX_XBUS_MODE_B = 5,
    SERIALRX_XBUS_MODE_B_RJ01 = 6,
    SERIALRX_IBUS = 7,
    SERIALRX_JETIEXBUS = 8,
    SERIALRX_CRSF = 9,
    SERIALRX_SRXL = 10,
    SERIALRX_TARGET_CUSTOM = 11,
    SERIALRX_FPORT = 12,
    SERIALRX_SRXL2 = 13,
    SERIALRX_GHST = 14
} SerialRXType;

#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define MAX_AUX_CHANNEL_COUNT (MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT)

#if MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT > MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT
#else
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#endif

extern const char rcChannelLetters[];

extern float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];       // interval [1000;2000]

#define RSSI_SCALE_MIN 1
#define RSSI_SCALE_MAX 255

#define RSSI_SCALE_DEFAULT 100

typedef enum {
    RX_FAILSAFE_MODE_AUTO = 0,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_FAILSAFE_MODE_INVALID
} rxFailsafeChannelMode_e;

#define RX_FAILSAFE_MODE_COUNT 3

typedef enum {
    RX_FAILSAFE_TYPE_FLIGHT = 0,
    RX_FAILSAFE_TYPE_AUX
} rxFailsafeChannelType_e;

#define RX_FAILSAFE_TYPE_COUNT 2

typedef struct rxFailsafeChannelConfig_s {
    uint8_t mode; // See rxFailsafeChannelMode_e
    uint8_t step;
} rxFailsafeChannelConfig_t;

PG_DECLARE_ARRAY(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs);

typedef struct rxChannelRangeConfig_s {
    uint16_t min;
    uint16_t max;
} rxChannelRangeConfig_t;

PG_DECLARE_ARRAY(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs);

struct rxRuntimeState_s;
typedef float (*rcReadRawDataFnPtr)(const struct rxRuntimeState_s *rxRuntimeState, uint8_t chan); // used by receiver driver to return channel data
typedef uint8_t (*rcFrameStatusFnPtr)(struct rxRuntimeState_s *rxRuntimeState);
typedef bool (*rcProcessFrameFnPtr)(const struct rxRuntimeState_s *rxRuntimeState);
typedef timeUs_t rcGetFrameTimeUsFn(void);  // used to retrieve the timestamp in microseconds for the last channel data frame

typedef enum {
    RX_PROVIDER_NONE = 0,
    RX_PROVIDER_PARALLEL_PWM,
    RX_PROVIDER_PPM,
    RX_PROVIDER_SERIAL,
    RX_PROVIDER_MSP,
    RX_PROVIDER_SPI,
} rxProvider_t;

typedef union rxFrameBuffer_u {
    ghstFrame_t ghst;
    sbusFrameData_t sbus;
} rxFrameBuffer_t;

typedef struct rxRuntimeState_s {
    rxProvider_t         rxProvider;
    SerialRXType         serialrxProvider;
    serialPort_t        *rxSerialPort;
    uint8_t              channelCount; // number of RC channels as reported by current input driver
    uint16_t             rxRefreshRate;
    rcReadRawDataFnPtr   rcReadRawFn;
    rcFrameStatusFnPtr   rcFrameStatusFn;
    rcProcessFrameFnPtr  rcProcessFrameFn;
    rcGetFrameTimeUsFn  *rcFrameTimeUsFn;
    timeUs_t             lastRcFrameTimeUs;
    uint16_t            *channelXData;
    rxFrameBuffer_t     *incomingFrame;
    rxFrameBuffer_t     *validatedFrame;
} rxRuntimeState_t;

typedef enum {
    RSSI_SOURCE_NONE = 0,
    RSSI_SOURCE_ADC,
    RSSI_SOURCE_RX_CHANNEL,
    RSSI_SOURCE_RX_PROTOCOL,
    RSSI_SOURCE_MSP,
    RSSI_SOURCE_FRAME_ERRORS,
    RSSI_SOURCE_RX_PROTOCOL_CRSF,
} rssiSource_e;

extern rssiSource_e rssiSource;

typedef enum {
    LQ_SOURCE_NONE = 0,
    LQ_SOURCE_RX_PROTOCOL_CRSF,
    LQ_SOURCE_RX_PROTOCOL_GHST,
} linkQualitySource_e;

extern linkQualitySource_e linkQualitySource;

extern rxRuntimeState_t rxRuntimeState; //!!TODO remove this extern, only needed once for channelCount

void rxInit(void);
bool rxSerialPortIsActive(SerialRXType serialrxProvider);
timeUs_t rxFrameTimeUs(void);
bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
bool rxIsReceivingSignal(void);
bool rxAreFlightChannelsValid(void);
bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs);

struct rxConfig_s;

void parseRcChannels(const char *input, struct rxConfig_s *rxConfig);

#define RSSI_MAX_VALUE 1023

void setRssiDirect(uint16_t newRssi, rssiSource_e source);
void setRssi(uint16_t rssiValue, rssiSource_e source);
void setRssiMsp(uint8_t newMspRssi);
void updateRSSI(timeUs_t currentTimeUs);
uint16_t getRssi(void);
uint8_t getRssiPercent(void);
bool isRssiConfigured(void);

#define LINK_QUALITY_MAX_VALUE 1023

uint16_t rxGetLinkQuality(void);
void setLinkQualityDirect(uint16_t linkqualityValue);
uint16_t rxGetLinkQualityPercent(void);

int16_t getRssiDbm(void);
void setRssiDbm(int16_t newRssiDbm, rssiSource_e source);
void setRssiDbmDirect(int16_t newRssiDbm, rssiSource_e source);

void rxSetRfMode(uint8_t rfModeValue);
uint8_t rxGetRfMode(void);

void rxSetUplinkTxPwrMw(uint16_t uplinkTxPwrMwValue);
uint16_t rxGetUplinkTxPwrMw(void);

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig);

void suspendRxPwmPpmSignal(void);
void resumeRxPwmPpmSignal(void);

uint16_t rxGetRefreshRate(void);

timeDelta_t rxGetFrameDelta(timeDelta_t *frameAgeUs);

void rxSwapFrameBuffers(rxRuntimeState_t *rxRuntimeState);
