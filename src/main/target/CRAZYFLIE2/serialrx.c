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
 * This file implements the custom Crazyflie serial Rx protocol which
 * consists of CRTP packets sent from the onboard NRF51 over UART using
 * the syslink protocol.
 *
 * The implementation supports two types of commander packets:
 * - RPYT on port CRTP_PORT_SETPOINT
 * - CPPM emulation on port CRTP_PORT_SETPOINT_GENERIC using type cppmEmuType
 *
 * The CPPM emulation packet type is recommended for use with this target.
 *
 * The RPYT type is mainly for legacy support (various mobile apps, python PC client,
 * etc) and can be used with the following restrictions to ensure angles are accurately
 * translated into PWM values:
 * - Max angles for pitch and roll must be set to 50 degrees
 * - Max yaw rate must be set to 400 degrees
 *
 * This implementation has been ported from the Crazyflie source code.
 *
 * For more information, see the following Crazyflie wiki pages:
 * CRTP:    https://wiki.bitcraze.io/projects:crazyflie:crtp
 * Syslink: https://wiki.bitcraze.io/doc:crazyflie:syslink:index
 */

#include <stdbool.h>
#include <stdint.h>

#include "io/serial.h"

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/targetcustomserial.h"
#include "syslink.h"
#include "crtp.h"

#define SYSLINK_BAUDRATE 1000000

// Variables for the Syslink Rx state machine
static syslinkRxState_e rxState = waitForFirstStart;
static syslinkPacket_t slp;
static uint8_t dataIndex = 0;
static uint8_t cksum[2] = {0};
static uint8_t counter = 0;

static rxRuntimeState_t *rxRuntimeStatePtr;
static serialPort_t *serialPort;

#define SUPPORTED_CHANNEL_COUNT (4 + CRTP_CPPM_EMU_MAX_AUX_CHANNELS)
static uint32_t channelData[SUPPORTED_CHANNEL_COUNT];
static bool rcFrameComplete = false;

static void routeIncommingPacket(syslinkPacket_t* slp)
{
    // Only support packets of type SYSLINK_RADIO_RAW
    if (slp->type == SYSLINK_RADIO_RAW) {
        crtpPacket_t *crtpPacket = (crtpPacket_t*)(slp->data);

        switch (crtpPacket->header.port) {
            case CRTP_PORT_SETPOINT:
            {
                crtpCommanderRPYT_t *crtpRYPTPacket =
                        (crtpCommanderRPYT_t*)&crtpPacket->data[0];

                // Write RPYT channels in TAER order

                // Translate thrust from 0-MAX_UINT16 into a PWM_style value (1000-2000)
                channelData[0] = (crtpRYPTPacket->thrust * 1000 / UINT16_MAX) + 1000;

                // Translate RPY from an angle setpoint to a PWM-style value (1000-2000)
                // For R and P, assume the client sends a max of -/+50 degrees (full range of 100)
                // For Y, assume a max of -/+400 deg/s (full range of 800)
                channelData[1] = (uint16_t)((crtpRYPTPacket->roll / 100 * 1000) + 1500);
                channelData[2] = (uint16_t)((-crtpRYPTPacket->pitch / 100 * 1000) + 1500); // Pitch is inverted
                channelData[3] = (uint16_t)((crtpRYPTPacket->yaw / 800 * 1000) + 1500);

                rcFrameComplete = true;
                break;
            }
            case CRTP_PORT_SETPOINT_GENERIC:
                // First byte of the packet is the type
                // Only support the CPPM Emulation type
                if (crtpPacket->data[0] == cppmEmuType) {
                    crtpCommanderCPPMEmuPacket_t *crtpCppmPacket =
                            (crtpCommanderCPPMEmuPacket_t*)&crtpPacket->data[1];

                    // Write RPYT channels in TAER order
                    channelData[0] = crtpCppmPacket->channelThrust;
                    channelData[1] = crtpCppmPacket->channelRoll;
                    channelData[2] = crtpCppmPacket->channelPitch;
                    channelData[3] = crtpCppmPacket->channelYaw;

                    // Write the rest of the auxiliary channels
                    uint8_t i;
                    for (i = 0; i < crtpCppmPacket->hdr.numAuxChannels; i++) {
                        channelData[i + 4] = crtpCppmPacket->channelAux[i];
                    }
                }
                rcFrameComplete = true;
                break;
            default:
                // Unsupported port - do nothing
                break;
        }
    }
}

// Receive ISR callback
static void dataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    counter++;
    switch (rxState) {
        case waitForFirstStart:
            rxState = (c == SYSLINK_START_BYTE1) ? waitForSecondStart : waitForFirstStart;
            break;
        case waitForSecondStart:
            rxState = (c == SYSLINK_START_BYTE2) ? waitForType : waitForFirstStart;
            break;
        case waitForType:
            cksum[0] = c;
            cksum[1] = c;
            slp.type = c;
            rxState = waitForLength;
            break;
        case waitForLength:
            if (c <= SYSLINK_MTU) {
                slp.length = c;
                cksum[0] += c;
                cksum[1] += cksum[0];
                dataIndex = 0;
                rxState = (c > 0) ? waitForData : waitForChksum1;
            }
            else {
                rxState = waitForFirstStart;
            }
            break;
        case waitForData:
            slp.data[dataIndex] = c;
            cksum[0] += c;
            cksum[1] += cksum[0];
            dataIndex++;
            if (dataIndex == slp.length) {
                rxState = waitForChksum1;
            }
            break;
        case waitForChksum1:
            if (cksum[0] == c) {
                rxState = waitForChksum2;
            }
            else {
                rxState = waitForFirstStart; //Checksum error
            }
            break;
        case waitForChksum2:
            if (cksum[1] == c) {
                routeIncommingPacket(&slp);
            }
            else {
                rxState = waitForFirstStart; //Checksum error
            }
            rxState = waitForFirstStart;
            break;
        default:
            break;
    }
}

static uint8_t frameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    if (!rcFrameComplete) {
        return RX_FRAME_PENDING;
    }

    // Set rcFrameComplete to false so we don't process this one twice
    rcFrameComplete = false;

    return RX_FRAME_COMPLETE;
}

static float readRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    if (chan >= rxRuntimeState->channelCount) {
        return 0;
    }
    return channelData[chan];
}


bool targetCustomSerialRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeStatePtr = rxRuntimeState;

    if (rxConfig->serialrx_provider != SERIALRX_TARGET_CUSTOM)
    {
        return false;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    rxRuntimeState->channelCount = SUPPORTED_CHANNEL_COUNT;
    rxRuntimeState->rxRefreshRate = 20000; // Value taken from rx_spi.c (NRF24 is being used downstream)
    rxRuntimeState->rcReadRawFn = readRawRC;
    rxRuntimeState->rcFrameStatusFn = frameStatus;

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        dataReceive,
        NULL,
        SYSLINK_BAUDRATE,
        MODE_RX,
        SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO
        );

    return serialPort != NULL;
}

