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
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_GHST

#include "build/build_config.h"
#include "build/debug.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "pg/rx.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/ghst.h"

#include "telemetry/ghst.h"

// NOTE: SERIAL_BIDIR appears to use OD drive on telemetry data from FC to Rx (slow rise times)
// SERIAL_BIDIR_PP would appear to be the correct way to do this, but this mode kills incoming control data
#define GHST_PORT_OPTIONS               (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_BIDIR)		
#define GHST_PORT_MODE                  MODE_RXTX   // bidirectional on single pin

#define GHST_MAX_FRAME_TIME_US          500         // 14 bytes @ 420k = ~450us
#define GHST_TIME_BETWEEN_FRAMES_US     4500        // fastest frame rate = 222.22Hz, or 4500us

#define GHST_PAYLOAD_OFFSET offsetof(ghstFrameDef_t, type)

STATIC_UNIT_TESTED bool ghstFrameDone = false;
STATIC_UNIT_TESTED ghstFrame_t ghstFrame;
STATIC_UNIT_TESTED ghstFrame_t ghstChannelDataFrame;
STATIC_UNIT_TESTED uint32_t ghstChannelData[GHST_MAX_NUM_CHANNELS];

static serialPort_t *serialPort;
static timeUs_t ghstFrameStartAtUs = 0;
static uint8_t telemetryBuf[GHST_FRAME_SIZE_MAX];
static uint8_t telemetryBufLen = 0;

static timeUs_t lastRcFrameTimeUs = 0;

/* GHST Protocol
 * Ghost uses 420k baud single-wire, half duplex connection, connected to a FC UART 'Tx' pin
 * Each control packet is interleaved with one or more corresponding downlink packets
 * 
 * Uplink packet format (Control packets)
 * <Addr><Len><Type><Payload><CRC>
 *
 * Addr:        u8      Destination address
 * Len          u8      Length includes the packet ID, but not the CRC
 * CRC          u8
 * 
 * Ghost packets are designed to be as short as possible, for minimum latency.
 * 
 * Note that the GHST protocol does not handle, itself, failsafe conditions. Packets are passed from 
 * the Ghost receiver to Betaflight as and when they arrive. Betaflight itself is responsible for
 * determining when a failsafe is necessary based on dropped packets. 
 * 
  */

#define GHST_FRAME_LENGTH_ADDRESS       1
#define GHST_FRAME_LENGTH_FRAMELENGTH   1
#define GHST_FRAME_LENGTH_TYPE_CRC      1

STATIC_UNIT_TESTED uint8_t ghstFrameCRC(void)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, ghstFrame.frame.type);
    for (int i = 0; i < ghstFrame.frame.len - GHST_FRAME_LENGTH_TYPE_CRC - 1; ++i) {
        crc = crc8_dvb_s2(crc, ghstFrame.frame.payload[i]);
    }
    return crc;
}

// Receive ISR callback, called back from serial port
STATIC_UNIT_TESTED void ghstDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    static uint8_t ghstFrameIdx = 0;
    const timeUs_t currentTimeUs = microsISR();

    if (cmpTimeUs(currentTimeUs, ghstFrameStartAtUs) > GHST_MAX_FRAME_TIME_US) {
        // Character received after the max. frame time, assume that this is a new frame
        ghstFrameIdx = 0;
    }

    if (ghstFrameIdx == 0) {
        // timestamp the start of the frame, to allow us to detect frame sync issues
        ghstFrameStartAtUs = currentTimeUs;
    }

    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    const int fullFrameLength = ghstFrameIdx < 3 ? 5 : ghstFrame.frame.len + GHST_FRAME_LENGTH_ADDRESS + GHST_FRAME_LENGTH_FRAMELENGTH;

    if (ghstFrameIdx < fullFrameLength) {
        ghstFrame.bytes[ghstFrameIdx++] = (uint8_t)c;
        if (ghstFrameIdx >= fullFrameLength) {                  // buffer is full?
            ghstFrameIdx = 0;
            const uint8_t crc = ghstFrameCRC();
            if (crc == ghstFrame.bytes[fullFrameLength - 1]) {
                switch (ghstFrame.frame.type)
                {
                    case GHST_UL_RC_CHANS_HS4_5TO8:
                    case GHST_UL_RC_CHANS_HS4_9TO12:
                    case GHST_UL_RC_CHANS_HS4_13TO16:
                        if (ghstFrame.frame.addr == GHST_ADDR_FC) {
                            lastRcFrameTimeUs = currentTimeUs;
                            ghstFrameDone = true;
                            memcpy(&ghstChannelDataFrame, &ghstFrame, sizeof(ghstFrame));
                        }
                        break;

                    default:
                        break;
                }
            }
        }
    }
}
STATIC_UNIT_TESTED uint8_t ghstFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    int iStartIdx = 4;

    if (ghstFrameDone) {
        ghstFrameDone = false;

        const ghstPayloadPulses_t* const rcChannels = (ghstPayloadPulses_t*)&ghstChannelDataFrame.frame.payload;

        // all uplink frames contain CH1..4 data (12 bit)
        ghstChannelData[0] = rcChannels->ch1 >> 1;
        ghstChannelData[1] = rcChannels->ch2 >> 1;
        ghstChannelData[2] = rcChannels->ch3 >> 1;
        ghstChannelData[3] = rcChannels->ch4 >> 1;

        // remainder of uplink frame contains 4 more channels (8 bit), sent in a round-robin fashion
        switch(ghstChannelDataFrame.frame.type)
        {
            case GHST_UL_RC_CHANS_HS4_5TO8:		iStartIdx = 4;	break;
            case GHST_UL_RC_CHANS_HS4_9TO12:	iStartIdx = 8;	break;
            case GHST_UL_RC_CHANS_HS4_13TO16:   iStartIdx = 12;	break;
        } 

        ghstChannelData[iStartIdx++] = rcChannels->cha << 3;
        ghstChannelData[iStartIdx++] = rcChannels->chb << 3;
        ghstChannelData[iStartIdx++] = rcChannels->chc << 3;
        ghstChannelData[iStartIdx++] = rcChannels->chd << 3;

        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

STATIC_UNIT_TESTED uint16_t ghstReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState); 

    // derived from original SBus scaling, with slight correction for offset (now symmetrical 
    // around OpenTx 0 value)
    // scaling is: 
    //      OpenTx   RC     PWM
    // min  -1024    172    988us
    // ctr  0        992    1500us
    // max  1024     1811   2012us   
    //

    return (5 * (rxRuntimeState->channelData[chan]+1) / 8) + 880;
}

// called from telemetry/ghst.c
void ghstRxWriteTelemetryData(const void *data, int len)
{
    len = MIN(len, (int)sizeof(telemetryBuf));
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}

void ghstRxSendTelemetryData(void)
{
    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
        telemetryBufLen = 0; // reset telemetry buffer
    }
}

static timeUs_t ghstFrameTimeUs(void)
{
    return lastRcFrameTimeUs;
}

bool ghstRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    for (int iChan = 0; iChan < GHST_MAX_NUM_CHANNELS; ++iChan) {
        ghstChannelData[iChan] = (16 * rxConfig->midrc) / 10 - 1408;
    }

    rxRuntimeState->channelCount = GHST_MAX_NUM_CHANNELS;
    rxRuntimeState->rxRefreshRate = GHST_TIME_BETWEEN_FRAMES_US;            // TODO: This needs to be dynamic    

    rxRuntimeState->rcReadRawFn = ghstReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ghstFrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = ghstFrameTimeUs;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        ghstDataReceive,
        NULL,
        GHST_RX_BAUDRATE,
        GHST_PORT_MODE,
        GHST_PORT_OPTIONS | (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0)
        );

    return serialPort != NULL;
}

bool ghstRxIsActive(void)
{
    return serialPort != NULL;
}
#endif
