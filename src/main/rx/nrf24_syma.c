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

#ifdef USE_RX_SYMA

#include "build/build_config.h"

#include "pg/rx.h"

#include "drivers/io.h"
#include "drivers/rx/rx_nrf24l01.h"
#include "drivers/time.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/nrf24_syma.h"

/*
 * Deviation transmitter sends 345 bind packets, then starts sending data packets.
 * Packets are send at rate of at least one every 4 milliseconds, ie at least 250Hz.
 * This means binding phase lasts 1.4 seconds, the transmitter then enters the data phase.
 * Other transmitters may vary but should have similar characteristics.
 */


/*
 * SymaX Protocol
 * No auto acknowledgment
 * Data rate is 250Kbps
 * Payload size is 10, static
 * Bind Phase
 * uses address {0xab,0xac,0xad,0xae,0xaf}
 * hops between 4 channels {0x4b, 0x30, 0x40, 0x20}
 * Data Phase
 * uses address received in bind packets
 * hops between 4 channels generated from address received in bind packets
 *
 * SymaX5C Protocol
 * No auto acknowledgment
 * Payload size is 16, static
 * Data rate is 1Mbps
 * Bind Phase
 * uses address {0x6d,0x6a,0x73,0x73,0x73}
 * hops between 16 channels {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36, 0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};
 * Data phase
 * uses same address as bind phase
 * hops between 15 channels {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24, 0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};
 * (common channels between both phases are: 0x27, 0x39, 0x24, 0x22, 0x2d)
 */

#define RC_CHANNEL_COUNT 9

enum {
    RATE_LOW = 0,
    RATE_MID = 1,
    RATE_HIGH= 2
};

#define FLAG_PICTURE    0x40
#define FLAG_VIDEO      0x80
#define FLAG_FLIP       0x40
#define FLAG_HEADLESS   0x80

#define FLAG_FLIP_X5C   0x01
#define FLAG_PICTURE_X5C 0x08
#define FLAG_VIDEO_X5C  0x10
#define FLAG_RATE_X5C   0x04

STATIC_UNIT_TESTED rx_spi_protocol_e symaProtocol;

typedef enum {
    STATE_BIND = 0,
    STATE_DATA
} protocol_state_t;

STATIC_UNIT_TESTED protocol_state_t protocolState;

// X11, X12, X5C-1 have 10-byte payload, X5C has 16-byte payload
#define SYMA_X_PROTOCOL_PAYLOAD_SIZE      10
#define SYMA_X5C_PROTOCOL_PAYLOAD_SIZE    16
STATIC_UNIT_TESTED uint8_t payloadSize;

#define RX_TX_ADDR_LEN     5
// set rxTxAddr to SymaX bind values
STATIC_UNIT_TESTED uint8_t rxTxAddr[RX_TX_ADDR_LEN] = {0xab, 0xac, 0xad, 0xae, 0xaf};
STATIC_UNIT_TESTED const uint8_t rxTxAddrX5C[RX_TX_ADDR_LEN] = {0x6d, 0x6a, 0x73, 0x73, 0x73};   // X5C uses same address for bind and data

// radio channels for frequency hopping
#define SYMA_X_RF_BIND_CHANNEL             8
#define SYMA_X_RF_CHANNEL_COUNT            4
#define SYMA_X5C_RF_BIND_CHANNEL_COUNT    16
#define SYMA_X5C_RF_CHANNEL_COUNT         15

STATIC_UNIT_TESTED uint8_t symaRfChannelCount = SYMA_X_RF_CHANNEL_COUNT;
STATIC_UNIT_TESTED uint8_t symaRfChannelIndex = 0;
// set rfChannels to SymaX bind channels, reserve enough space for SymaX5C channels
STATIC_UNIT_TESTED uint8_t symaRfChannels[SYMA_X5C_RF_BIND_CHANNEL_COUNT]  = {0x4b, 0x30, 0x40, 0x20};
STATIC_UNIT_TESTED const uint8_t symaRfChannelsX5C[SYMA_X5C_RF_CHANNEL_COUNT] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24, 0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};

static uint32_t packetCount = 0;
static uint32_t timeOfLastHop;
static uint32_t hopTimeout = 10000; // 10ms

STATIC_UNIT_TESTED bool symaCheckBindPacket(const uint8_t *packet)
{
    bool bindPacket = false;
    if (symaProtocol == RX_SPI_NRF24_SYMA_X) {
        if ((packet[5] == 0xaa) && (packet[6] == 0xaa) && (packet[7] == 0xaa)) {
            bindPacket = true;
            rxTxAddr[4] = packet[0];
            rxTxAddr[3] = packet[1];
            rxTxAddr[2] = packet[2];
            rxTxAddr[1] = packet[3];
            rxTxAddr[0] = packet[4];
        }
    } else {
        if ((packet[0] == 0) && (packet[1] == 0) && (packet[14] == 0xc0) && (packet[15] == 0x17)) {
            bindPacket = true;
        }
    }
    return bindPacket;
}

STATIC_UNIT_TESTED uint16_t symaConvertToPwmUnsigned(uint8_t val)
{
    uint32_t ret = val;
    ret = ret * (PWM_RANGE_MAX - PWM_RANGE_MIN) / UINT8_MAX + PWM_RANGE_MIN;
    return (uint16_t)ret;
}

STATIC_UNIT_TESTED uint16_t symaConvertToPwmSigned(uint8_t val)
{
    int32_t ret = val & 0x7f;
    ret = (ret * (PWM_RANGE_MAX - PWM_RANGE_MIN)) / (2 * INT8_MAX);
    if (val & 0x80) {// sign bit set
        ret = -ret;
    }
    return (uint16_t)(PWM_RANGE_MIDDLE + ret);
}

void symaNrf24SetRcDataFromPayload(uint16_t *rcData, const uint8_t *packet)
{
    rcData[RC_SPI_THROTTLE] = symaConvertToPwmUnsigned(packet[0]); // throttle
    rcData[RC_SPI_ROLL] = symaConvertToPwmSigned(packet[3]); // aileron
    if (symaProtocol == RX_SPI_NRF24_SYMA_X) {
        rcData[RC_SPI_PITCH] = symaConvertToPwmSigned(packet[1]); // elevator
        rcData[RC_SPI_YAW] = symaConvertToPwmSigned(packet[2]); // rudder
        const uint8_t rate = (packet[5] & 0xc0) >> 6;
        if (rate == RATE_LOW) {
            rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIN;
        } else if (rate == RATE_MID) {
            rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIDDLE;
        } else {
            rcData[RC_CHANNEL_RATE] = PWM_RANGE_MAX;
        }
        rcData[RC_CHANNEL_FLIP] = packet[6] & FLAG_FLIP ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[RC_CHANNEL_PICTURE] = packet[4] & FLAG_PICTURE ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[RC_CHANNEL_VIDEO] = packet[4] & FLAG_VIDEO ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[RC_CHANNEL_HEADLESS] = packet[14] & FLAG_HEADLESS ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    } else {
        rcData[RC_SPI_PITCH] = symaConvertToPwmSigned(packet[2]); // elevator
        rcData[RC_SPI_YAW] = symaConvertToPwmSigned(packet[1]); // rudder
        const uint8_t flags = packet[14];
        rcData[RC_CHANNEL_RATE] = flags & FLAG_RATE_X5C ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[RC_CHANNEL_FLIP] = flags & FLAG_FLIP_X5C ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[RC_CHANNEL_PICTURE] = flags & FLAG_PICTURE_X5C ? PWM_RANGE_MAX : PWM_RANGE_MIN;
        rcData[RC_CHANNEL_VIDEO] = flags & FLAG_VIDEO_X5C ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    }
}

static void symaHopToNextChannel(void)
{
    // hop channel every second packet
    ++packetCount;
    if ((packetCount & 0x01) == 0) {
        ++symaRfChannelIndex;
        if (symaRfChannelIndex >= symaRfChannelCount) {
            symaRfChannelIndex = 0;
        }
    }
    NRF24L01_SetChannel(symaRfChannels[symaRfChannelIndex]);
}

// The SymaX hopping channels are determined by the low bits of rxTxAddress
static void setSymaXHoppingChannels(uint32_t addr)
{
    addr = addr & 0x1f;
    if (addr == 0x06) {
        addr = 0x07;
    }
    const uint32_t inc = (addr << 24) | (addr << 16) | (addr << 8) | addr;
    uint32_t * const prfChannels = (uint32_t *)symaRfChannels;
    if (addr == 0x16) {
        *prfChannels = 0x28481131;
    } else if (addr == 0x1e) {
        *prfChannels = 0x38184121;
    } else if (addr < 0x10) {
        *prfChannels = 0x3A2A1A0A + inc;
    } else if (addr < 0x18) {
        *prfChannels = 0x1231FA1A + inc;
    } else {
        *prfChannels = 0x19FA2202 + inc;
    }
}

/*
 * This is called periodically by the scheduler.
 * Returns RX_SPI_RECEIVED_DATA if a data packet was received.
 */
rx_spi_received_e symaNrf24DataReceived(uint8_t *payload)
{
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (protocolState) {
    case STATE_BIND:
        if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize)) {
            const bool bindPacket = symaCheckBindPacket(payload);
            if (bindPacket) {
                ret = RX_SPI_RECEIVED_BIND;
                protocolState = STATE_DATA;
                // using protocol NRF24L01_SYMA_X, since NRF24L01_SYMA_X5C went straight into data mode
                // set the hopping channels as determined by the rxTxAddr received in the bind packet
                setSymaXHoppingChannels(rxTxAddr[0]);
                // set the NRF24 to use the rxTxAddr received in the bind packet
                NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);
                packetCount = 0;
                symaRfChannelIndex = 0;
                NRF24L01_SetChannel(symaRfChannels[0]);
            }
        }
        break;
    case STATE_DATA:
        // read the payload, processing of payload is deferred
        if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize)) {
            symaHopToNextChannel();
            timeOfLastHop = micros();
            ret = RX_SPI_RECEIVED_DATA;
        }
        if (micros() > timeOfLastHop + hopTimeout) {
            symaHopToNextChannel();
            timeOfLastHop = micros();
        }
        break;
    }
    return ret;
}

static void symaNrf24Setup(rx_spi_protocol_e protocol)
{
    symaProtocol = protocol;
    NRF24L01_Initialize(BIT(NRF24L01_00_CONFIG_EN_CRC) | BIT( NRF24L01_00_CONFIG_CRCO)); // sets PWR_UP, EN_CRC, CRCO - 2 byte CRC
    NRF24L01_SetupBasic();

    if (symaProtocol == RX_SPI_NRF24_SYMA_X) {
        payloadSize = SYMA_X_PROTOCOL_PAYLOAD_SIZE;
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
        protocolState = STATE_BIND;
        // RX_ADDR for pipes P1-P5 are left at default values
        NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);
    } else {
        payloadSize = SYMA_X5C_PROTOCOL_PAYLOAD_SIZE;
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
        // RX_ADDR for pipes P1-P5 are left at default values
        NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddrX5C, RX_TX_ADDR_LEN);
        // just go straight into data mode, since the SYMA_X5C protocol does not actually require binding
        protocolState = STATE_DATA;
        symaRfChannelCount = SYMA_X5C_RF_CHANNEL_COUNT;
        memcpy(symaRfChannels, symaRfChannelsX5C, SYMA_X5C_RF_CHANNEL_COUNT);
    }
    NRF24L01_SetChannel(symaRfChannels[0]);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payloadSize);

    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets
}

bool symaNrf24Init(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    UNUSED(extiConfig);

    rxRuntimeState->channelCount = RC_CHANNEL_COUNT;
    symaNrf24Setup((rx_spi_protocol_e)rxSpiConfig->rx_spi_protocol);

    return true;
}
#endif
