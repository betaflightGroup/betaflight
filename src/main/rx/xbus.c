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

#ifdef USE_SERIALRX_XBUS

#include "common/crc.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/xbus.h"

//
// Serial driver for JR's XBus (MODE B) receiver
//

#define XBUS_CHANNEL_COUNT 12
#define XBUS_RJ01_CHANNEL_COUNT 12

// Frame is: ID(1 byte) + 12*channel(2 bytes) + CRC(2 bytes) = 27
#define XBUS_FRAME_SIZE_A1 27
#define XBUS_FRAME_SIZE_A2 35


#define XBUS_RJ01_FRAME_SIZE 33
#define XBUS_RJ01_MESSAGE_LENGTH 30
#define XBUS_RJ01_OFFSET_BYTES 3

#define XBUS_BAUDRATE 115200
#define XBUS_RJ01_BAUDRATE 250000
#define XBUS_MAX_FRAME_TIME 8000

// NOTE!
// This is actually based on ID+LENGTH (nibble each)
// 0xA - Multiplex ID (also used by JR, no idea why)
// 0x1 - 12 channels
// 0x2 - 16 channels
// However, the JR XG14 that is used for test at the moment
// does only use 0xA1 as its output. This is why the implementation
// is based on these numbers only. Maybe update this in the future?
#define XBUS_START_OF_FRAME_BYTE_A1 (0xA1)      //12 channels
#define XBUS_START_OF_FRAME_BYTE_A2 (0xA2)      //16 channels transfare, but only 12 channels use for

// Pulse length convertion from [0...4095] to µs:
//      800µs  -> 0x000
//      1500µs -> 0x800
//      2200µs -> 0xFFF
// Total range is: 2200 - 800 = 1400 <==> 4095
// Use formula: 800 + value * 1400 / 4096 (i.e. a shift by 12)
#define XBUS_CONVERT_TO_USEC(V) (800 + ((V * 1400) >> 12))

static bool xBusFrameReceived = false;
static bool xBusDataIncoming = false;
static uint8_t xBusFramePosition;
static uint8_t xBusFrameLength;
static uint8_t xBusChannelCount;
static uint8_t xBusProvider;


// Use max values for ram areas
static volatile uint8_t xBusFrame[XBUS_FRAME_SIZE_A2];  //size 35 for 16 channels in xbus_Mode_B
static uint16_t xBusChannelData[XBUS_RJ01_CHANNEL_COUNT];

// Full RJ01 message CRC calculations
static uint8_t xBusRj01CRC8(uint8_t inData, uint8_t seed)
{
    for (uint8_t bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
        const uint8_t temp = ((seed ^ inData) & 0x01);

        if (temp == 0) {
            seed >>= 1;
        } else {
            seed ^= 0x18;
            seed >>= 1;
            seed |= 0x80;
        }

        inData >>= 1;
    }

    return seed;
}


static void xBusUnpackModeBFrame(uint8_t offsetBytes)
{
    // Calculate the CRC of the incoming frame
    // Calculate on all bytes except the final two CRC bytes
    const uint16_t inCrc = crc16_ccitt_update(0, (uint8_t*)&xBusFrame[offsetBytes], xBusFrameLength - 2);

    // Get the received CRC
    const uint16_t crc = (((uint16_t)xBusFrame[offsetBytes + xBusFrameLength - 2]) << 8) + ((uint16_t)xBusFrame[offsetBytes + xBusFrameLength - 1]);

    if (crc == inCrc) {
        // Unpack the data, we have a valid frame, only 12 channel unpack also when receive 16 channel
        for (int i = 0; i < xBusChannelCount; i++) {

            const uint8_t frameAddr = offsetBytes + 1 + i * 2;
            uint16_t value = ((uint16_t)xBusFrame[frameAddr]) << 8;
            value = value + ((uint16_t)xBusFrame[frameAddr + 1]);

            // Convert to internal format
            xBusChannelData[i] = XBUS_CONVERT_TO_USEC(value);
        }

        xBusFrameReceived = true;
    }
}

static void xBusUnpackRJ01Frame(void)
{
    // Calculate the CRC of the incoming frame
    uint8_t outerCrc = 0;
    uint8_t i = 0;

    // When using the Align RJ01 receiver with
    // a MODE B setting in the radio (XG14 tested)
    // the MODE_B -frame is packed within some
    // at the moment unknown bytes before and after:
    // 0xA1 LEN __ 0xA1 12*(High + Low) CRC1 CRC2 + __ __ CRC_OUTER
    // Compared to a standard MODE B frame that only
    // contains the "middle" package.
    // Hence, at the moment, the unknown header and footer
    // of the RJ01 MODEB packages are discarded.
    // However, the LAST byte (CRC_OUTER) is infact an 8-bit
    // CRC for the whole package, using the Dallas-One-Wire CRC
    // method.
    // So, we check both these values as well as the provided length
    // of the outer/full message (LEN)

    //
    // Check we have correct length of message
    //
    if (xBusFrame[1] != XBUS_RJ01_MESSAGE_LENGTH)
    {
        // Unknown package as length is not ok
        return;
    }

    //
    // CRC calculation & check for full message
    //
    for (i = 0; i < xBusFrameLength - 1; i++) {
        outerCrc = xBusRj01CRC8(outerCrc, xBusFrame[i]);
    }

    if (outerCrc != xBusFrame[xBusFrameLength - 1])
    {
        // CRC does not match, skip this frame
        return;
    }

    // Now unpack the "embedded MODE B frame"
    xBusUnpackModeBFrame(XBUS_RJ01_OFFSET_BYTES);
}

// Receive ISR callback
static void xBusDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    uint32_t now;
    static uint32_t xBusTimeLast, xBusTimeInterval;

    // Check if we shall reset frame position due to time
    now = micros();
    xBusTimeInterval = now - xBusTimeLast;
    xBusTimeLast = now;
    if (xBusTimeInterval > XBUS_MAX_FRAME_TIME) {
        xBusFramePosition = 0;
        xBusDataIncoming = false;
    }

    // Check if we shall start a frame?
    if (xBusFramePosition == 0) {
        if (c == XBUS_START_OF_FRAME_BYTE_A1) {
            xBusDataIncoming = true;
            xBusFrameLength = XBUS_FRAME_SIZE_A1;   //decrease framesize (when receiver change, otherwise board must reboot)
        } else if (c == XBUS_START_OF_FRAME_BYTE_A2) {//16channel packet
            xBusDataIncoming = true;
            xBusFrameLength = XBUS_FRAME_SIZE_A2;   //increase framesize
        }
    }

    // Only do this if we are receiving to a frame
    if (xBusDataIncoming == true) {
        // Store in frame copy
        xBusFrame[xBusFramePosition] = (uint8_t)c;
        xBusFramePosition++;
    }

    // Done?
    if (xBusFramePosition == xBusFrameLength) {
        switch (xBusProvider) {
        case SERIALRX_XBUS_MODE_B:
            xBusUnpackModeBFrame(0);
            FALLTHROUGH; //!!TODO - check this fall through is correct
        case SERIALRX_XBUS_MODE_B_RJ01:
            xBusUnpackRJ01Frame();
        }
        xBusDataIncoming = false;
        xBusFramePosition = 0;
    }
}

// Indicate time to read a frame from the data...
static uint8_t xBusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    if (!xBusFrameReceived) {
        return RX_FRAME_PENDING;
    }

    xBusFrameReceived = false;

    return RX_FRAME_COMPLETE;
}

static float xBusReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    uint16_t data;

    // Deliver the data wanted
    if (chan >= rxRuntimeState->channelCount) {
        return 0;
    }

    data = xBusChannelData[chan];

    return data;
}

bool xBusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    uint32_t baudRate;

    switch (rxRuntimeState->serialrxProvider) {
    case SERIALRX_XBUS_MODE_B:
        rxRuntimeState->channelCount = XBUS_CHANNEL_COUNT;
        xBusFrameReceived = false;
        xBusDataIncoming = false;
        xBusFramePosition = 0;
        baudRate = XBUS_BAUDRATE;
        xBusFrameLength = XBUS_FRAME_SIZE_A1;
        xBusChannelCount = XBUS_CHANNEL_COUNT;
        xBusProvider = SERIALRX_XBUS_MODE_B;
        break;
    case SERIALRX_XBUS_MODE_B_RJ01:
        rxRuntimeState->channelCount = XBUS_RJ01_CHANNEL_COUNT;
        xBusFrameReceived = false;
        xBusDataIncoming = false;
        xBusFramePosition = 0;
        baudRate = XBUS_RJ01_BAUDRATE;
        xBusFrameLength = XBUS_RJ01_FRAME_SIZE;
        xBusChannelCount = XBUS_RJ01_CHANNEL_COUNT;
        xBusProvider = SERIALRX_XBUS_MODE_B_RJ01;
        break;
    default:
        return false;
        break;
    }

    rxRuntimeState->rxRefreshRate = 11000;

    rxRuntimeState->rcReadRawFn = xBusReadRawRC;
    rxRuntimeState->rcFrameStatusFn = xBusFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig, rxRuntimeState->serialrxProvider);
#else
    bool portShared = false;
#endif

    serialPort_t *xBusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        xBusDataReceive,
        NULL,
        baudRate,
        portShared ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = xBusPort;
    }
#endif

    return xBusPort != NULL;
}
#endif
