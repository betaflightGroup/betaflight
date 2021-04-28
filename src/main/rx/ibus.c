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
 * Driver for IBUS (Flysky) receiver
 *   - initial implementation for MultiWii by Cesco/Pl¸schi
 *   - implementation for BaseFlight by Andreas (fiendie) Tacke
 *   - ported to CleanFlight by Konstantin (digitalentity) Sharlaimov
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_IBUS

#include "pg/rx.h"

#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "rx/rx.h"
#include "rx/ibus.h"
#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"

#define IBUS_MAX_CHANNEL 18
//In AFHDS there is 18 channels encoded in 14 slots (each slot is 2 byte long)
#define IBUS_MAX_SLOTS 14
#define IBUS_BUFFSIZE 32
#define IBUS_MODEL_IA6B 0
#define IBUS_MODEL_IA6 1
#define IBUS_FRAME_GAP 500

#define IBUS_BAUDRATE 115200
#define IBUS_TELEMETRY_PACKET_LENGTH (4)
#define IBUS_SERIAL_RX_PACKET_LENGTH (32)

static uint8_t ibusModel;
static uint8_t ibusSyncByte;
static uint8_t ibusFrameSize;
static uint8_t ibusChannelOffset;
static uint8_t rxBytesToIgnore;
static uint16_t ibusChecksum;

static bool ibusFrameDone = false;
static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];

static uint8_t ibus[IBUS_BUFFSIZE] = { 0, };
static timeUs_t lastFrameTimeUs = 0;
static timeUs_t lastRcFrameTimeUs = 0;

static bool isValidIa6bIbusPacketLength(uint8_t length)
{
    return (length == IBUS_TELEMETRY_PACKET_LENGTH) || (length == IBUS_SERIAL_RX_PACKET_LENGTH);
}


// Receive ISR callback
static void ibusDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    static timeUs_t ibusTimeLast;
    static uint8_t ibusFramePosition;

    const timeUs_t now = microsISR();

    if (cmpTimeUs(now, ibusTimeLast) > IBUS_FRAME_GAP) {
        ibusFramePosition = 0;
        rxBytesToIgnore = 0;
    } else if (rxBytesToIgnore) {
        rxBytesToIgnore--;
        return;
    }

    ibusTimeLast = now;

    if (ibusFramePosition == 0) {
        if (isValidIa6bIbusPacketLength(c)) {
            ibusModel = IBUS_MODEL_IA6B;
            ibusSyncByte = c;
            ibusFrameSize = c;
            ibusChannelOffset = 2;
            ibusChecksum = 0xFFFF;
        } else if ((ibusSyncByte == 0) && (c == 0x55)) {
            ibusModel = IBUS_MODEL_IA6;
            ibusSyncByte = 0x55;
            ibusFrameSize = 31;
            ibusChecksum = 0x0000;
            ibusChannelOffset = 1;
        } else if (ibusSyncByte != c) {
            return;
        }
    }

    ibus[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == ibusFrameSize - 1) {
        lastFrameTimeUs = now;
        ibusFrameDone = true;
    } else {
        ibusFramePosition++;
    }
}


static bool isChecksumOkIa6(void)
{
    uint8_t offset;
    uint8_t i;
    uint16_t chksum, rxsum;
    chksum = ibusChecksum;
    rxsum = ibus[ibusFrameSize - 2] + (ibus[ibusFrameSize - 1] << 8);
    for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_SLOTS; i++, offset += 2) {
        chksum += ibus[offset] + (ibus[offset + 1] << 8);
    }
    return chksum == rxsum;
}


static bool checksumIsOk(void) {
    if (ibusModel == IBUS_MODEL_IA6 ) {
        return isChecksumOkIa6();
    } else {
        return isChecksumOkIa6b(ibus, ibusFrameSize);
    }
}


static void updateChannelData(void) {
    uint8_t i;
    uint8_t offset;
    for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_SLOTS; i++, offset += 2) {
        ibusChannelData[i] = ibus[offset] + ((ibus[offset + 1] & 0x0F) << 8);
    }
    //latest IBUS recievers are using prviously not used 4 bits on every channel to incresse total channel count
    for (i = IBUS_MAX_SLOTS, offset = ibusChannelOffset + 1; i < IBUS_MAX_CHANNEL; i++, offset += 6) {
        ibusChannelData[i] = ((ibus[offset] & 0xF0) >> 4) | (ibus[offset + 2] & 0xF0) | ((ibus[offset + 4] & 0xF0) << 4);
    }
}

static uint8_t ibusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!ibusFrameDone) {
        return frameStatus;
    }

    ibusFrameDone = false;

    if (checksumIsOk()) {
        if (ibusModel == IBUS_MODEL_IA6 || ibusSyncByte == IBUS_SERIAL_RX_PACKET_LENGTH) {
            updateChannelData();
            frameStatus = RX_FRAME_COMPLETE;
            lastRcFrameTimeUs = lastFrameTimeUs;
#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
        } else {
            rxBytesToIgnore = respondToIbusRequest(ibus);
#endif
        }
    }

    return frameStatus;
}


static float ibusReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return ibusChannelData[chan];
}

static timeUs_t ibusFrameTimeUsFn(void)
{
    return lastRcFrameTimeUs;
}

bool ibusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxConfig);
    ibusSyncByte = 0;

    rxRuntimeState->channelCount = IBUS_MAX_CHANNEL;
    rxRuntimeState->rxRefreshRate = 20000; // TODO - Verify speed

    rxRuntimeState->rcReadRawFn = ibusReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ibusFrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = ibusFrameTimeUsFn;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = isSerialPortShared(portConfig, FUNCTION_RX_SERIAL, FUNCTION_TELEMETRY_IBUS);
#else
    bool portShared = false;
#endif


    rxBytesToIgnore = 0;
    serialPort_t *ibusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        ibusDataReceive,
        NULL,
        IBUS_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex || portShared ? SERIAL_BIDIR : 0)
        );

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
    if (portShared) {
        initSharedIbusTelemetry(ibusPort);
    }
#endif

    return ibusPort != NULL;
}

#endif //SERIAL_RX
