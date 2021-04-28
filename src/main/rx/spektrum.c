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

#include "platform.h"

#ifdef USE_SERIALRX_SPEKTRUM

#include <string.h>

#include "common/maths.h"

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"

#include "config/config.h"

#include "io/spektrum_rssi.h"
#include "io/spektrum_vtx_control.h"

#include "telemetry/telemetry.h"
#include "telemetry/srxl.h"

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "config/feature.h"

// driver for spektrum satellite receiver / sbus

#define SPEKTRUM_TELEMETRY_FRAME_DELAY_US 1000 // Gap between received Rc frame and transmited TM frame

bool srxlEnabled = false;
int32_t resolution;
uint8_t rssi_channel;

static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekHiRes = false;

static volatile uint8_t spekFrame[SPEK_FRAME_SIZE];

static rxRuntimeState_t *rxRuntimeStatePtr;
static serialPort_t *serialPort;

#if defined(USE_TELEMETRY_SRXL)
static uint8_t telemetryBuf[SRXL_FRAME_SIZE_MAX];
static uint8_t telemetryBufLen = 0;
#endif

static timeUs_t lastRcFrameTimeUs = 0;

// Receive ISR callback
static void spektrumDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    static timeUs_t spekTimeLast = 0;
    static uint8_t spekFramePosition = 0;

    const timeUs_t now = microsISR();
    const timeUs_t spekTimeInterval = cmpTimeUs(now, spekTimeLast);
    spekTimeLast = now;

    if (spekTimeInterval > SPEKTRUM_NEEDED_FRAME_INTERVAL) {
        spekFramePosition = 0;
    }

    if (spekFramePosition < SPEK_FRAME_SIZE) {
        spekFrame[spekFramePosition++] = (uint8_t)c;
        if (spekFramePosition < SPEK_FRAME_SIZE) {
            rcFrameComplete = false;
        } else {
            lastRcFrameTimeUs = now;
            rcFrameComplete = true;
        }
    }
}


uint32_t spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];

static uint8_t spektrumFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

#if defined(USE_TELEMETRY_SRXL)
    static timeUs_t telemetryFrameRequestedUs = 0;

    timeUs_t currentTimeUs = micros();
#endif

    uint8_t result = RX_FRAME_PENDING;

    if (rcFrameComplete) {
        rcFrameComplete = false;

#if defined(USE_SPEKTRUM_REAL_RSSI) || defined(USE_SPEKTRUM_FAKE_RSSI)
        spektrumHandleRSSI(spekFrame);
#endif

        // Get the VTX control bytes in a frame
        uint32_t vtxControl = ((spekFrame[SPEKTRUM_VTX_CONTROL_1] << 24) |
                            (spekFrame[SPEKTRUM_VTX_CONTROL_2] << 16) |
                            (spekFrame[SPEKTRUM_VTX_CONTROL_3] <<  8) |
                            (spekFrame[SPEKTRUM_VTX_CONTROL_4] <<  0) );

        int8_t spektrumRcDataSize;
        // Handle VTX control frame.
        if ((vtxControl & SPEKTRUM_VTX_CONTROL_FRAME_MASK) == SPEKTRUM_VTX_CONTROL_FRAME &&
            (spekFrame[2] & 0x80) == 0 )  {
#if defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
            spektrumHandleVtxControl(vtxControl);
#endif
            spektrumRcDataSize = SPEK_FRAME_SIZE - SPEKTRUM_VTX_CONTROL_SIZE;
        } else {
            spektrumRcDataSize = SPEK_FRAME_SIZE;
        }

        // Get the RC control channel inputs
        for (int b = 3; b < spektrumRcDataSize; b += 2) {
            const uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
            if (spekChannel < rxRuntimeStatePtr->channelCount && spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT) {
                if (rssi_channel == 0 || spekChannel != rssi_channel) {
                    spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
                }
            }
        }

#if defined(USE_TELEMETRY_SRXL)
        if (srxlEnabled && (spekFrame[2] & 0x80) == 0) {
                    telemetryFrameRequestedUs = currentTimeUs;
        }
#endif
        result = RX_FRAME_COMPLETE;
    }

#if defined(USE_TELEMETRY_SRXL)
    if (telemetryBufLen && telemetryFrameRequestedUs && cmpTimeUs(currentTimeUs, telemetryFrameRequestedUs) >= SPEKTRUM_TELEMETRY_FRAME_DELAY_US) {
        telemetryFrameRequestedUs = 0;

        result = (result & ~RX_FRAME_PENDING) | RX_FRAME_PROCESSING_REQUIRED;
    }
#endif

    return result;
}

static float spektrumReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    float data;

    if (chan >= rxRuntimeState->channelCount) {
        return 0;
    }

    if (spekHiRes) {
        data = 0.5f * (float)spekChannelData[chan] + 988; // 2048 mode
    } else {
        data = spekChannelData[chan] + 988;               // 1024 mode
    }

    return data;
}

#ifdef USE_SPEKTRUM_BIND

bool spekShouldBind(uint8_t spektrum_sat_bind)
{
#ifdef USE_SPEKTRUM_BIND_PLUG
    IO_t BindPlug = IOGetByTag(rxConfig()->spektrum_bind_plug_ioTag);

    if (BindPlug) {
        IOInit(BindPlug, OWNER_RX_BIND, 0);
        IOConfigGPIO(BindPlug, IOCFG_IPU);

        // Check status of bind plug and exit if not active
        delayMicroseconds(10);  // allow configuration to settle
        if (IORead(BindPlug)) {
            return false;
        }
    }
#endif // USE_SPEKTRUM_BIND_PLUG

    return !(
        isMPUSoftReset() ||
        spektrum_sat_bind == SPEKTRUM_SAT_BIND_DISABLED ||
        spektrum_sat_bind > SPEKTRUM_SAT_BIND_MAX
    );
}

/* spektrumBind function ported from Baseflight. It's used to bind satellite receiver to TX.
 * Function must be called immediately after startup so that we don't miss satellite bind window.
 * Known parameters. Tested with DSMX satellite and DX8 radio. Framerate (11ms or 22ms) must be selected from TX.
 * 9 = DSMX 11ms / DSMX 22ms
 * 5 = DSM2 11ms 2048 / DSM2 22ms 1024
 */
void spektrumBind(rxConfig_t *rxConfig)
{
    if (!spekShouldBind(rxConfig->spektrum_sat_bind)) {
        return;
    }

    // Determine a pin to use
    ioTag_t bindPin = IO_TAG_NONE;

    if (rxConfig->spektrum_bind_pin_override_ioTag) {
        bindPin = rxConfig->spektrum_bind_pin_override_ioTag;
    } else {
        const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
        if (!portConfig) {
            return;
        }

        int index = SERIAL_PORT_IDENTIFIER_TO_INDEX(portConfig->identifier);
        ioTag_t txPin = serialPinConfig()->ioTagTx[index];
        ioTag_t rxPin = serialPinConfig()->ioTagRx[index];

        // Take care half-duplex case
        switch (rxRuntimeState.serialrxProvider) {
        case SERIALRX_SRXL:
#if defined(USE_TELEMETRY_SRXL)
            if (featureIsEnabled(FEATURE_TELEMETRY) && !telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
                bindPin = txPin;
            }
            break;
#endif // USE_TELEMETRY_SRXL

        default:
            bindPin = rxConfig->halfDuplex ? txPin : rxPin;
        }

        if (!bindPin) {
            return;
        }
    }

    IO_t bindIO = IOGetByTag(bindPin);

    IOInit(bindIO, OWNER_RX_BIND, 0);
    IOConfigGPIO(bindIO, IOCFG_OUT_PP);

    LED1_ON;

    // RX line, set high
    IOWrite(bindIO, true);

    // Bind window is around 20-140ms after powerup
    delay(60);
    LED1_OFF;

    for (int i = 0; i < rxConfig->spektrum_sat_bind; i++) {
        LED0_OFF;
        LED2_OFF;
        // RX line, drive low for 120us
        IOWrite(bindIO, false);
        delayMicroseconds(120);

        LED0_ON;
        LED2_ON;
        // RX line, drive high for 120us
        IOWrite(bindIO, true);
        delayMicroseconds(120);

    }


    // Release the bind pin to avoid interference with an actual rx pin,
    // when rxConfig->spektrum_bind_pin_override_ioTag is used.
    // This happens when the bind pin is connected in parallel to the rx pin.

    if (rxConfig->spektrum_bind_pin_override_ioTag) {
        delay(50); // Keep it high for 50msec
        IOConfigGPIO(bindIO, IOCFG_IN_FLOATING);
    }

    // If we came here as a result of hard  reset (power up, with spektrum_sat_bind set), then reset it back to zero and write config
    // Don't reset if hardware bind plug is present
    // Reset only when autoreset is enabled

    if (!rxConfig->spektrum_bind_plug_ioTag && rxConfig->spektrum_sat_bind_autoreset == 1 && !isMPUSoftReset()) {
        rxConfig->spektrum_sat_bind = 0;
        saveConfigAndNotify();
    }
}
#endif // USE_SPEKTRUM_BIND

#if defined(USE_TELEMETRY_SRXL)
static bool spektrumProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
        telemetryBufLen = 0; // reset telemetry buffer
    }

    return true;
}

bool srxlTelemetryBufferEmpty()
{
  if (telemetryBufLen == 0) {
      return true;
  } else {
      return false;
  }
}

void srxlRxWriteTelemetryData(const void *data, int len)
{
    len = MIN(len, (int)sizeof(telemetryBuf));
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}
#endif

static timeUs_t spektrumFrameTimeUsFn(void)
{
    return lastRcFrameTimeUs;
}

bool spektrumInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeStatePtr = rxRuntimeState;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    srxlEnabled = false;
#if defined(USE_TELEMETRY_SRXL)
    bool portShared = telemetryCheckRxPortShared(portConfig, rxRuntimeState->serialrxProvider);
#else
    bool portShared = false;
#endif

    switch (rxRuntimeState->serialrxProvider) {
    default:

        break;
    case SERIALRX_SRXL:
#if defined(USE_TELEMETRY_SRXL)
        srxlEnabled = (featureIsEnabled(FEATURE_TELEMETRY) && !portShared);
        FALLTHROUGH;
#endif
    case SERIALRX_SPEKTRUM2048:
        // 11 bit frames
        spek_chan_shift = 3;
        spek_chan_mask = 0x07;
        spekHiRes = true;
        resolution = 2048;
        rxRuntimeState->channelCount = SPEKTRUM_2048_CHANNEL_COUNT;
        rxRuntimeState->rxRefreshRate = 11000;
        break;
    case SERIALRX_SPEKTRUM1024:
        // 10 bit frames
        spek_chan_shift = 2;
        spek_chan_mask = 0x03;
        spekHiRes = false;
        resolution = 1024;
        rxRuntimeState->channelCount = SPEKTRUM_1024_CHANNEL_COUNT;
        rxRuntimeState->rxRefreshRate = 22000;
        break;
    }

    rxRuntimeState->rcReadRawFn = spektrumReadRawRC;
    rxRuntimeState->rcFrameStatusFn = spektrumFrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = spektrumFrameTimeUsFn;
#if defined(USE_TELEMETRY_SRXL)
    rxRuntimeState->rcProcessFrameFn = spektrumProcessFrame;
#endif

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        spektrumDataReceive,
        NULL,
        SPEKTRUM_BAUDRATE,
        portShared || srxlEnabled ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) |
        ((srxlEnabled || rxConfig->halfDuplex) ? SERIAL_BIDIR : 0)
        );

#if defined(USE_TELEMETRY_SRXL)
    if (portShared) {
        telemetrySharedPort = serialPort;
    }
#endif

    rssi_channel = rxConfig->rssi_channel - 1; // -1 because rxConfig->rssi_channel is 1-based and rssi_channel is 0-based.
    if (rssi_channel >= rxRuntimeState->channelCount) {
        rssi_channel = 0;
    }

    return serialPort != NULL;
}

bool srxlRxIsActive(void)
{
    return serialPort != NULL;
}

#endif // SERIAL_RX
