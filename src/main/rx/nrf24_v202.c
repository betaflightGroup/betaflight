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

// this file is copied with modifications from bradwii for jd385
// see https://github.com/hackocopter/bradwii-jd385

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RX_V202

#include "build/build_config.h"

#include "common/utils.h"

#include "pg/rx.h"

#include "drivers/io.h"
#include "drivers/rx/rx_nrf24l01.h"
#include "drivers/time.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/nrf24_v202.h"

/*
 * V202 Protocol
 * No auto acknowledgment
 * Payload size is 16 and static
 * Data rate is 1Mbps, there is a 256Kbps data rate used by the Deviation transmitter implementation
 * Bind Phase
 * uses address {0x66, 0x88, 0x68, 0x68, 0x68}
 * uses channels from the frequency hopping table
 * Data phase
 * uses same address as bind phase
 * hops between 16 channels that are set using the txId sent in the bind packet and the frequency hopping table
 */

#define V2X2_PAYLOAD_SIZE 16
#define V2X2_NFREQCHANNELS 16
#define TXIDSIZE 3
#define V2X2_RC_CHANNEL_COUNT 11

enum {
    // packet[14] flags
    V2X2_FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
    V2X2_FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
    V2X2_FLAG_FLIP   = 0x04,
    V2X2_FLAG_UNK9   = 0x08,
    V2X2_FLAG_LED    = 0x10,
    V2X2_FLAG_UNK10  = 0x20,
    V2X2_FLAG_BIND   = 0xC0,
    // packet[10] flags
    V2X2_FLAG_HEADLESS  = 0x02,
    V2X2_FLAG_MAG_CAL_X = 0x08,
    V2X2_FLAG_MAG_CAL_Y = 0x20
};

enum {
    PHASE_NOT_BOUND = 0,
    PHASE_BOUND
};

// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
static const uint8_t v2x2_freq_hopping[][V2X2_NFREQCHANNELS] = {
 { 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
   0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
 { 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
   0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
 { 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
   0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
 { 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
   0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};

STATIC_UNIT_TESTED uint8_t rf_channels[V2X2_NFREQCHANNELS];
STATIC_UNIT_TESTED uint8_t rf_ch_num;
STATIC_UNIT_TESTED uint8_t bind_phase;
static uint32_t packet_timer;
STATIC_UNIT_TESTED uint8_t txid[TXIDSIZE];
static uint32_t rx_timeout;
extern uint16_t rxSpiRcData[];

static const unsigned char v2x2_channelindex[] = {RC_SPI_THROTTLE,RC_SPI_YAW,RC_SPI_PITCH,RC_SPI_ROLL,
        RC_SPI_AUX1,RC_SPI_AUX2,RC_SPI_AUX3,RC_SPI_AUX4,RC_SPI_AUX5,RC_SPI_AUX6,RC_SPI_AUX7};

static void prepare_to_bind(void)
{
    packet_timer = micros();
    for (int i = 0; i < V2X2_NFREQCHANNELS; ++i) {
        rf_channels[i] = v2x2_freq_hopping[0][i];
    }
    rx_timeout = 1000L;
}

static void switch_channel(void)
{
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_channels[rf_ch_num]);
    if (++rf_ch_num >= V2X2_NFREQCHANNELS) rf_ch_num = 0;
}

static void v2x2_set_tx_id(uint8_t *id)
{
    uint8_t sum;
    txid[0] = id[0];
    txid[1] = id[1];
    txid[2] = id[2];
    sum = id[0] + id[1] + id[2];

    // Base row is defined by lowest 2 bits
    const uint8_t *fh_row = v2x2_freq_hopping[sum & 0x03];
    // Higher 3 bits define increment to corresponding row
    uint8_t increment = (sum & 0x1e) >> 2;
    for (int i = 0; i < V2X2_NFREQCHANNELS; ++i) {
        uint8_t val = fh_row[i] + increment;
        // Strange avoidance of channels divisible by 16
        rf_channels[i] = (val & 0x0f) ? val : val - 3;
    }
}

static void decode_bind_packet(uint8_t *packet)
{
    if ((packet[14] & V2X2_FLAG_BIND) == V2X2_FLAG_BIND) {
        // Fill out rf_channels with bound protocol parameters
        v2x2_set_tx_id(&packet[7]);
        bind_phase = PHASE_BOUND;
        rx_timeout = 1000L; // find the channel as fast as possible
    }
}

// Returns whether the data was successfully decoded
static rx_spi_received_e decode_packet(uint8_t *packet)
{
    if (bind_phase != PHASE_BOUND) {
        decode_bind_packet(packet);
        return RX_SPI_RECEIVED_BIND;
    }
    // Decode packet
    if ((packet[14] & V2X2_FLAG_BIND) == V2X2_FLAG_BIND) {
        return RX_SPI_RECEIVED_BIND;
    }
    if (packet[7] != txid[0] ||
        packet[8] != txid[1] ||
        packet[9] != txid[2]) {
        return RX_SPI_RECEIVED_NONE;
    }
    // Restore regular interval
    rx_timeout = 10000L; // 4ms interval, duplicate packets, (8ms unique) + 25%
    // TREA order in packet to MultiWii order is handled by
    // correct assignment to channelindex
    // Throttle 0..255 to 1000..2000
    rxSpiRcData[v2x2_channelindex[0]] = ((uint16_t)packet[0]) * 1000 / 255 + 1000;
    for (int i = 1; i < 4; ++i) {
        uint8_t a = packet[i];
        rxSpiRcData[v2x2_channelindex[i]] = ((uint16_t)(a < 0x80 ? 0x7f - a : a)) * 1000 / 255 + 1000;
    }
    const uint8_t flags[] = {V2X2_FLAG_LED, V2X2_FLAG_FLIP, V2X2_FLAG_CAMERA, V2X2_FLAG_VIDEO}; // two more unknown bits
    for (int i = 4; i < 8; ++i) {
        rxSpiRcData[v2x2_channelindex[i]] = (packet[14] & flags[i-4]) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    }
    const uint8_t flags10[] = {V2X2_FLAG_HEADLESS, V2X2_FLAG_MAG_CAL_X, V2X2_FLAG_MAG_CAL_Y};
    for (int i = 8; i < 11; ++i) {
        rxSpiRcData[v2x2_channelindex[i]] = (packet[10] & flags10[i-8]) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    }
    packet_timer = micros();
    return RX_SPI_RECEIVED_DATA;
}

void v202Nrf24SetRcDataFromPayload(uint16_t *rcData, const uint8_t *packet)
{
    UNUSED(rcData);
    UNUSED(packet);
    // Ideally the decoding of the packet should be moved into here, to reduce the overhead of v202DataReceived function.
}

static rx_spi_received_e readrx(uint8_t *packet)
{
    if (!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & BIT(NRF24L01_07_STATUS_RX_DR))) {
        uint32_t t = micros() - packet_timer;
        if (t > rx_timeout) {
            switch_channel();
            packet_timer = micros();
        }
        return RX_SPI_RECEIVED_NONE;
    }
    packet_timer = micros();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR)); // clear the RX_DR flag
    NRF24L01_ReadPayload(packet, V2X2_PAYLOAD_SIZE);
    NRF24L01_FlushRx();

    switch_channel();
    return decode_packet(packet);
}

/*
 * This is called periodically by the scheduler.
 * Returns RX_SPI_RECEIVED_DATA if a data packet was received.
 */
rx_spi_received_e v202Nrf24DataReceived(uint8_t *packet)
{
    return readrx(packet);
}

static void v202Nrf24Setup(rx_spi_protocol_e protocol)
{
    NRF24L01_Initialize(BIT(NRF24L01_00_CONFIG_EN_CRC) | BIT(NRF24L01_00_CONFIG_CRCO)); // 2-bytes CRC

    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BIT(NRF24L01_02_EN_RXADDR_ERX_P0));  // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xFF); // 4ms retransmit t/o, 15 tries
    if (protocol == RX_SPI_NRF24_V202_250K) {
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    } else {
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    }
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR) | BIT(NRF24L01_07_STATUS_TX_DS) | BIT(NRF24L01_07_STATUS_MAX_RT));     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, V2X2_PAYLOAD_SIZE);  // bytes of data payload for pipe 0
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
#define RX_TX_ADDR_LEN 5
    const uint8_t rx_tx_addr[RX_TX_ADDR_LEN] = {0x66, 0x88, 0x68, 0x68, 0x68};
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, RX_TX_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, RX_TX_ADDR_LEN);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    rf_ch_num = 0;
    bind_phase = PHASE_NOT_BOUND;
    prepare_to_bind();
    switch_channel();
    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets
}

bool v202Nrf24Init(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    UNUSED(extiConfig);

    rxRuntimeState->channelCount = V2X2_RC_CHANNEL_COUNT;
    v202Nrf24Setup((rx_spi_protocol_e)rxSpiConfig->rx_spi_protocol);

    return true;
}
#endif
