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

#include "platform.h"

#ifdef USE_SERIALRX_SBUS

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus.h"
#include "rx/sbus_channels.h"

/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
*
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

#define SBUS_BAUDRATE                 100000
#define SBUS_RX_REFRESH_RATE          11000
#define SBUS_TIME_NEEDED_PER_FRAME    3000

#define SBUS_FAST_BAUDRATE              200000
#define SBUS_FAST_RX_REFRESH_RATE       6000

#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#if !defined(SBUS_PORT_OPTIONS)
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

enum {
    DEBUG_SBUS_FRAME_FLAGS = 0,
    DEBUG_SBUS_STATE_FLAGS,
    DEBUG_SBUS_FRAME_TIME,
};


// Receive ISR callback
static void sbusDataReceive(uint16_t c, void *data)
{
    rxRuntimeState_t *rxRuntimeState = (rxRuntimeState_t *)data;
    sbusFrameData_t *sbusFrameData = &rxRuntimeState->incomingFrame->sbus;

    const timeUs_t nowUs = microsISR();

    const timeDelta_t sbusFrameTime = cmpTimeUs(nowUs, sbusFrameData->startAtUs);

    if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
        sbusFrameData->position = 0;
    }

    if (sbusFrameData->position == 0) {
        if (c != SBUS_FRAME_BEGIN_BYTE) {
            return;
        }
        sbusFrameData->startAtUs = nowUs;
    }

    if (sbusFrameData->position < SBUS_FRAME_SIZE) {
        sbusFrameData->frame.bytes[sbusFrameData->position++] = (uint8_t)c;
        if (sbusFrameData->position < SBUS_FRAME_SIZE) {
            sbusFrameData->done = false;
        } else {
            sbusFrameData->done = true;
            DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_TIME, sbusFrameTime);
        }
    }
}

static uint8_t sbusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    sbusFrameData_t *sbusFrameData = &rxRuntimeState->incomingFrame->sbus;
    if (!sbusFrameData->done) {
        return RX_FRAME_PENDING;
    }
    sbusFrameData->done = false;

    DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_FLAGS, sbusFrameData->frame.frame.channels.flags);

    const uint8_t frameStatus = sbusChannelsDecode(rxRuntimeState, &sbusFrameData->frame.frame.channels);

    if (!(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
        rxRuntimeState->lastRcFrameTimeUs = sbusFrameData->startAtUs;
    }

    return frameStatus;
}

bool sbusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    static uint32_t sbusBaudRate;

    sbusChannelsInit(rxConfig, rxRuntimeState);

    rxRuntimeState->channelCount = SBUS_MAX_CHANNEL;

    if (rxConfig->sbus_baud_fast) {
        rxRuntimeState->rxRefreshRate = SBUS_FAST_RX_REFRESH_RATE;
        sbusBaudRate  = SBUS_FAST_BAUDRATE;
    } else {
        rxRuntimeState->rxRefreshRate = SBUS_RX_REFRESH_RATE;
        sbusBaudRate  = SBUS_BAUDRATE;
    }

    rxRuntimeState->rcFrameStatusFn = sbusFrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig, rxRuntimeState->serialrxProvider);
#else
    bool portShared = false;
#endif

    rxRuntimeState->rxSerialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        sbusDataReceive,
        rxRuntimeState,
        sbusBaudRate,
        portShared ? MODE_RXTX : MODE_RX,
        SBUS_PORT_OPTIONS | (rxConfig->serialrx_inverted ? 0 : SERIAL_INVERTED) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

    if (rxConfig->rssi_src_frame_errors) {
        rssiSource = RSSI_SOURCE_FRAME_ERRORS;
    }

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = rxRuntimeState->rxSerialPort;
    }
#endif

    return rxRuntimeState->rxSerialPort != NULL;
}
#endif
