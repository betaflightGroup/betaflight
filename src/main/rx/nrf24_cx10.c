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

// This file borrows heavily from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_CX10

#include "build/build_config.h"

#include "pg/rx.h"

#include "drivers/io.h"
#include "drivers/rx/rx_nrf24l01.h"
#include "drivers/rx/rx_xn297.h"
#include "drivers/time.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/nrf24_cx10.h"

/*
 * Deviation transmitter
 * Bind phase lasts 6 seconds for CX10, for CX10A it lasts until an acknowledgment is received.
 * Other transmitters may vary but should have similar characteristics.
 * For CX10A protocol: after receiving a bind packet, the receiver must send back a data packet with byte[9] = 1 as acknowledgment
 */

/*
 * CX10 Protocol
 * No auto acknowledgment
 * Payload size is 19 and static for CX10A variant, 15 and static for CX10 variant.
 * Data rate is 1Mbps
 * Bind Phase
 * uses address {0xcc, 0xcc, 0xcc, 0xcc, 0xcc}, converted by XN297
 * uses channel 0x02
 * Data phase
 * uses same address as bind phase
 * hops between 4 channels that are set from the txId sent in the bind packet
 */

#define RC_CHANNEL_COUNT 9

enum {
    RATE_LOW = 0,
    RATE_MID = 1,
    RATE_HIGH= 2
};

#define FLAG_FLIP       0x10 // goes to rudder channel
// flags1
#define FLAG_MODE_MASK  0x03
#define FLAG_HEADLESS   0x04
// flags2
#define FLAG_VIDEO      0x02
#define FLAG_PICTURE    0x04

static rx_spi_protocol_e cx10Protocol;

typedef enum {
    STATE_BIND = 0,
    STATE_ACK,
    STATE_DATA
} protocol_state_t;

STATIC_UNIT_TESTED protocol_state_t protocolState;

#define CX10_PROTOCOL_PAYLOAD_SIZE  15
#define CX10A_PROTOCOL_PAYLOAD_SIZE 19
static uint8_t payloadSize;
#define ACK_TO_SEND_COUNT 8

#define CRC_LEN 2
#define RX_TX_ADDR_LEN     5
STATIC_UNIT_TESTED uint8_t txAddr[RX_TX_ADDR_LEN] = {0x55, 0x0F, 0x71, 0x0C, 0x00}; // converted XN297 address, 0xC710F55 (28 bit)
STATIC_UNIT_TESTED uint8_t rxAddr[RX_TX_ADDR_LEN] = {0x49, 0x26, 0x87, 0x7d, 0x2f}; // converted XN297 address
#define TX_ID_LEN 4
STATIC_UNIT_TESTED uint8_t txId[TX_ID_LEN];

#define CX10_RF_BIND_CHANNEL           0x02
#define RF_CHANNEL_COUNT 4
STATIC_UNIT_TESTED uint8_t cx10RfChannelIndex = 0;
STATIC_UNIT_TESTED uint8_t cx10RfChannels[RF_CHANNEL_COUNT]; // channels are set using txId from bind packet

#define CX10_PROTOCOL_HOP_TIMEOUT  1500 // 1.5ms
#define CX10A_PROTOCOL_HOP_TIMEOUT 6500 // 6.5ms
static uint32_t hopTimeout;
static uint32_t timeOfLastHop;

/*
 * Returns true if it is a bind packet.
 */
STATIC_UNIT_TESTED bool cx10CheckBindPacket(const uint8_t *packet)
{
    const bool bindPacket = (packet[0] == 0xaa); // 10101010
    if (bindPacket) {
        txId[0] = packet[1];
        txId[1] = packet[2];
        txId[2] = packet[3];
        txId[3] = packet[4];
        return true;
    }
    return false;
}

STATIC_UNIT_TESTED uint16_t cx10ConvertToPwmUnsigned(const uint8_t *pVal)
{
    uint16_t ret = (*(pVal + 1)) & 0x7f; // mask out top bit which is used for a flag for the rudder
    ret = (ret << 8) | *pVal;
    return ret;
}

void cx10Nrf24SetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    const uint8_t offset = (cx10Protocol == RX_SPI_NRF24_CX10) ? 0 : 4;
    rcData[RC_SPI_ROLL] = (PWM_RANGE_MAX + PWM_RANGE_MIN) - cx10ConvertToPwmUnsigned(&payload[5 + offset]);  // aileron
    rcData[RC_SPI_PITCH] = (PWM_RANGE_MAX + PWM_RANGE_MIN) - cx10ConvertToPwmUnsigned(&payload[7 + offset]); // elevator
    rcData[RC_SPI_THROTTLE] = cx10ConvertToPwmUnsigned(&payload[9 + offset]); // throttle
    rcData[RC_SPI_YAW] = cx10ConvertToPwmUnsigned(&payload[11 + offset]);  // rudder
    const uint8_t flags1 = payload[13 + offset];
    const uint8_t rate = flags1 & FLAG_MODE_MASK; // takes values 0, 1, 2
    if (rate == RATE_LOW) {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIN;
    } else if (rate == RATE_MID) {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIDDLE;
    } else {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MAX;
    }
    // flip flag is in YAW byte
    rcData[RC_CHANNEL_FLIP] = payload[12 + offset] & FLAG_FLIP ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    const uint8_t flags2 = payload[14 + offset];
    rcData[RC_CHANNEL_PICTURE] = flags2 & FLAG_PICTURE ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_VIDEO] = flags2 & FLAG_VIDEO ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_HEADLESS] = flags1 & FLAG_HEADLESS ? PWM_RANGE_MAX : PWM_RANGE_MIN;
}

static void cx10HopToNextChannel(void)
{
    ++cx10RfChannelIndex;
    if (cx10RfChannelIndex >= RF_CHANNEL_COUNT) {
        cx10RfChannelIndex = 0;
    }
    NRF24L01_SetChannel(cx10RfChannels[cx10RfChannelIndex]);
}

// The hopping channels are determined by the txId
STATIC_UNIT_TESTED void cx10SetHoppingChannels(const uint8_t *txId)
{
    cx10RfChannelIndex = 0;
    cx10RfChannels[0] = 0x03 + (txId[0] & 0x0F);
    cx10RfChannels[1] = 0x16 + (txId[0] >> 4);
    cx10RfChannels[2] = 0x2D + (txId[1] & 0x0F);
    cx10RfChannels[3] = 0x40 + (txId[1] >> 4);
}

static bool cx10CrcOK(uint16_t crc, const uint8_t *payload)
{
    if (payload[payloadSize] != (crc >> 8)) {
        return false;
    }
    if (payload[payloadSize + 1] != (crc & 0xff)) {
        return false;
    }
    return true;
}

static bool cx10ReadPayloadIfAvailable(uint8_t *payload)
{
    if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize + CRC_LEN)) {
        const uint16_t crc = XN297_UnscramblePayload(payload, payloadSize, rxAddr);
        if (cx10CrcOK(crc, payload)) {
            return true;
        }
    }
    return false;
}

/*
 * This is called periodically by the scheduler.
 * Returns RX_SPI_RECEIVED_DATA if a data packet was received.
 */
rx_spi_received_e cx10Nrf24DataReceived(uint8_t *payload)
{
    static uint8_t ackCount;
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
    int totalDelayUs;
    uint32_t timeNowUs;

    switch (protocolState) {
    case STATE_BIND:
        if (cx10ReadPayloadIfAvailable(payload)) {
            const bool bindPacket = cx10CheckBindPacket(payload);
            if (bindPacket) {
                // set the hopping channels as determined by the txId received in the bind packet
                cx10SetHoppingChannels(txId);
                ret = RX_SPI_RECEIVED_BIND;
                protocolState = STATE_ACK;
                ackCount = 0;
            }
        }
        break;
    case STATE_ACK:
        // transmit an ACK packet
        ++ackCount;
        totalDelayUs = 0;
        // send out an ACK on the bind channel, required by deviationTx
        payload[9] = 0x01;
        NRF24L01_SetChannel(CX10_RF_BIND_CHANNEL);
        NRF24L01_FlushTx();
        XN297_WritePayload(payload, payloadSize, rxAddr);
        NRF24L01_SetTxMode();// enter transmit mode to send the packet
        // wait for the ACK packet to send before changing channel
        static const int fifoDelayUs = 100;
        while (!(NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BIT(NRF24L01_17_FIFO_STATUS_TX_EMPTY))) {
            delayMicroseconds(fifoDelayUs);
            totalDelayUs += fifoDelayUs;
        }
        // send out an ACK on each of the hopping channels, required by CX10 transmitter
        for (int ii = 0; ii < RF_CHANNEL_COUNT; ++ii) {
            NRF24L01_SetChannel(cx10RfChannels[ii]);
            XN297_WritePayload(payload, payloadSize, rxAddr);
            NRF24L01_SetTxMode();// enter transmit mode to send the packet
            // wait for the ACK packet to send before changing channel
            while (!(NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BIT(NRF24L01_17_FIFO_STATUS_TX_EMPTY))) {
                delayMicroseconds(fifoDelayUs);
                totalDelayUs += fifoDelayUs;
            }
        }
        static const int delayBetweenPacketsUs = 1000;
        if (totalDelayUs < delayBetweenPacketsUs) {
            delayMicroseconds(delayBetweenPacketsUs - totalDelayUs);
        }
        NRF24L01_SetRxMode();//reenter receive mode after sending ACKs
        if (ackCount > ACK_TO_SEND_COUNT) {
            NRF24L01_SetChannel(cx10RfChannels[0]);
            // and go into data state to wait for first data packet
            protocolState = STATE_DATA;
        }
        break;
    case STATE_DATA:
        timeNowUs = micros();
        // read the payload, processing of payload is deferred
        if (cx10ReadPayloadIfAvailable(payload)) {
            cx10HopToNextChannel();
            timeOfLastHop = timeNowUs;
            ret = RX_SPI_RECEIVED_DATA;
        }
        if (timeNowUs > timeOfLastHop + hopTimeout) {
            cx10HopToNextChannel();
            timeOfLastHop = timeNowUs;
        }
    }
    return ret;
}

static void cx10Nrf24Setup(rx_spi_protocol_e protocol)
{
    cx10Protocol = protocol;
    protocolState = STATE_BIND;
    payloadSize = (protocol == RX_SPI_NRF24_CX10) ? CX10_PROTOCOL_PAYLOAD_SIZE : CX10A_PROTOCOL_PAYLOAD_SIZE;
    hopTimeout = (protocol == RX_SPI_NRF24_CX10) ? CX10_PROTOCOL_HOP_TIMEOUT : CX10A_PROTOCOL_HOP_TIMEOUT;

    NRF24L01_Initialize(0); // sets PWR_UP, no CRC
    NRF24L01_SetupBasic();

    NRF24L01_SetChannel(CX10_RF_BIND_CHANNEL);

    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    // RX_ADDR for pipes P2 to P5 are left at default values
    NRF24L01_FlushRx();
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, txAddr, RX_TX_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxAddr, RX_TX_ADDR_LEN);

    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payloadSize + CRC_LEN); // payload + 2 bytes CRC

    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets
}

bool cx10Nrf24Init(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    UNUSED(extiConfig);

    rxRuntimeState->channelCount = RC_CHANNEL_COUNT;
    cx10Nrf24Setup((rx_spi_protocol_e)rxSpiConfig->rx_spi_protocol);

    return true;
}
#endif
