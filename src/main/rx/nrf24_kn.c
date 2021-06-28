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

#ifdef USE_RX_KN

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rx/rx_nrf24l01.h"
#include "drivers/time.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/nrf24_kn.h"

/*
 * KN Protocol
 */

#define KN_PAYLOAD_SIZE 16
#define KN_NFREQCHANNELS 4
#define RX_TX_ADDR_LEN 5
#define KN_RC_CHANNEL_COUNT 8

enum {
    // packet[12] flags
    KN_FLAG_DR = 0x01,
    KN_FLAG_TRHOLD  = 0x02,
    KN_FLAG_IDLEUP   = 0x04,
    KN_FLAG_TD   = 0x40
};

enum {
    PHASE_NOT_BOUND = 0,
    PHASE_RECEIVED,
    PHASE_BOUND
};

STATIC_UNIT_TESTED uint8_t rf_ch_num;
STATIC_UNIT_TESTED uint8_t bind_phase;
static uint32_t packet_timer;
STATIC_UNIT_TESTED uint8_t txid[RX_TX_ADDR_LEN];
STATIC_UNIT_TESTED uint8_t kn_freq_hopping[KN_NFREQCHANNELS];
static uint32_t rx_timeout;
extern uint16_t rxSpiRcData[];

static const unsigned char kn_channelindex[] = {RC_SPI_THROTTLE,RC_SPI_ROLL,RC_SPI_PITCH,RC_SPI_YAW,
        RC_SPI_AUX1,RC_SPI_AUX2,RC_SPI_AUX3,RC_SPI_AUX4};

static void prepare_to_bind(void)
{
    packet_timer = micros();
    for (int i = 0; i < KN_NFREQCHANNELS; ++i) {
        kn_freq_hopping[i] = 0;
    }
    rx_timeout = 1000L;
}

static void switch_channel(void)
{
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, kn_freq_hopping[rf_ch_num]);
    if (++rf_ch_num >= KN_NFREQCHANNELS) rf_ch_num = 0;
}

static void decode_bind_packet(uint8_t *packet)
{
	if (packet[0]==0x4b && packet[1]==0x4e && packet[2]==0x44 && packet[3]==0x5a) {
		txid[0] = packet[4];
		txid[1] = packet[5];
		txid[2] = packet[6];
		txid[3] = packet[7];
		txid[4] = 0x4b;
		
		kn_freq_hopping[0] = packet[8];
		kn_freq_hopping[1] = packet[9];
		kn_freq_hopping[2] = packet[10];
		kn_freq_hopping[3] = packet[11];
		
		if (packet[15]==0x01) {
			NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
		} else {
			NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
		}
		
		NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, txid, RX_TX_ADDR_LEN);
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, txid, RX_TX_ADDR_LEN);
		
        bind_phase = PHASE_BOUND;
        rx_timeout = 1000L; // find the channel as fast as possible
    }
}

// Returns whether the data was successfully decoded
static rx_spi_received_e decode_packet(uint8_t *packet)
{
    if (bind_phase == PHASE_NOT_BOUND) {
        decode_bind_packet(packet);
        return RX_SPI_RECEIVED_BIND;
    }
    // Decode packet
    // Restore regular interval
    rx_timeout = 13000L; // 13ms if data received
    bind_phase = PHASE_RECEIVED;
	
    for (int i = 0; i < 4; ++i) {
        uint16_t a = packet[i*2];
        uint16_t b = packet[(i*2)+1];
        rxSpiRcData[kn_channelindex[i]] = ((uint16_t)(a<<8)+b) * 1000 / 1024 + 1000;
    }
    const uint8_t flags[] = {KN_FLAG_DR, KN_FLAG_TRHOLD, KN_FLAG_IDLEUP, KN_FLAG_TD};
    for (int i = 4; i < 8; ++i) {
        rxSpiRcData[kn_channelindex[i]] = (packet[12] & flags[i-4]) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    }
    packet_timer = micros();
    return RX_SPI_RECEIVED_DATA;
}

void knNrf24SetRcDataFromPayload(uint16_t *rcData, const uint8_t *packet)
{
    UNUSED(rcData);
    UNUSED(packet);
}

static rx_spi_received_e readrx(uint8_t *packet)
{
    if (!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & BIT(NRF24L01_07_STATUS_RX_DR))) {
        uint32_t t = micros() - packet_timer;
        if (t > rx_timeout) {
			if (bind_phase == PHASE_RECEIVED) {
				switch_channel();
			}
            packet_timer = micros();
			rx_timeout = 10000L; // 10ms if data not received
        }
        return RX_SPI_RECEIVED_NONE;
    }
    packet_timer = micros();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR)); // clear the RX_DR flag
    NRF24L01_ReadPayload(packet, KN_PAYLOAD_SIZE);
    NRF24L01_FlushRx();

    switch_channel();
    return decode_packet(packet);
}

/*
 * This is called periodically by the scheduler.
 * Returns RX_SPI_RECEIVED_DATA if a data packet was received.
 */
rx_spi_received_e knNrf24DataReceived(uint8_t *packet)
{
    return readrx(packet);
}

static void knNrf24Setup(rx_spi_protocol_e protocol)
{
    NRF24L01_Initialize(BIT(NRF24L01_00_CONFIG_EN_CRC) | BIT(NRF24L01_00_CONFIG_CRCO)); // 2-bytes CRC

    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BIT(NRF24L01_02_EN_RXADDR_ERX_P0));  // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00);
    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR) | BIT(NRF24L01_07_STATUS_TX_DS) | BIT(NRF24L01_07_STATUS_MAX_RT));     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, KN_PAYLOAD_SIZE);  // bytes of data payload for pipe 0
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here

    const uint8_t rx_tx_addr[RX_TX_ADDR_LEN] = {0x4b, 0x4e, 0x44, 0x5a, 0x4b};
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, RX_TX_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, RX_TX_ADDR_LEN);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    rf_ch_num = 0;
    bind_phase = PHASE_NOT_BOUND;
    prepare_to_bind();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x53);// switch to channel 83
    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets
}

bool knNrf24Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    UNUSED(extiConfig);

    rxRuntimeState->channelCount = KN_RC_CHANNEL_COUNT;
    knNrf24Setup((rx_spi_protocol_e)rxConfig->rx_spi_protocol);

    return true;
}
#endif
