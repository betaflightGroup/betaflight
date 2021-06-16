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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define GHST_RX_BAUDRATE                420000

#define GHST_TX_BAUDRATE_FAST           400000
#define GHST_TX_BAUDRATE_SLOW           115200
#define GHST_BYTE_TIME_FAST_US          ((1000000/GHST_TX_BAUDRATE_FAST)*10)      // 10 bit words (8 data, 1 start, 1 stop)
#define GHST_BYTE_TIME_SLOW_US          ((1000000/GHST_TX_BAUDRATE_SLOW)*10)
#define GHST_UART_WORDLENGTH            UART_WORDLENGTH_8B

typedef enum {
    GHST_ADDR_RADIO             = 0x80,
    GHST_ADDR_TX_MODULE_SYM     = 0x81,     // symmetrical, 400k pulses, 400k telemetry
    GHST_ADDR_TX_MODULE_ASYM    = 0x88,     // asymmetrical, 400k pulses, 115k telemetry
    GHST_ADDR_FC                = 0x82,
    GHST_ADDR_GOGGLES           = 0x83,
    GHST_ADDR_QUANTUM_TEE1      = 0x84,     // phase 2
    GHST_ADDR_QUANTUM_TEE2      = 0x85,
    GHST_ADDR_QUANTUM_GW1       = 0x86,
    GHST_ADDR_5G_CLK            = 0x87,     // phase 3
    GHST_ADDR_RX                = 0x89
} ghstAddr_e;

typedef enum {
    // frame types 0x10 - 0x1f always include 4 primary channels, plus either 4 aux channels,
    // or other type-specific data. Expect types 0x14-0x1f to be added in the future, and even though
    // not explicitly supported, the 4 primary channels should always be extracted.
    GHST_UL_RC_CHANS_HS4_FIRST  = 0x10,     // First frame type including 4 primary channels
    GHST_UL_RC_CHANS_HS4_5TO8   = 0x10,     // primary 4 channel, plus CH5-8
    GHST_UL_RC_CHANS_HS4_9TO12  = 0x11,     // primary 4 channel, plus CH9-12
    GHST_UL_RC_CHANS_HS4_13TO16 = 0x12,     // primary 4 channel, plus CH13-16
    GHST_UL_RC_CHANS_HS4_RSSI   = 0x13,     // primary 4 channel, plus RSSI, LQ, RF Mode, and Tx Power
    GHST_UL_RC_CHANS_HS4_LAST   = 0x1f      // Last frame type including 4 primary channels
} ghstUl_e;

#define GHST_UL_RC_CHANS_SIZE       12      // 1 (type) + 10 (data) + 1 (crc)

typedef enum {
    GHST_DL_OPENTX_SYNC         = 0x20,
    GHST_DL_LINK_STAT           = 0x21,
    GHST_DL_VTX_STAT            = 0x22,
    GHST_DL_PACK_STAT           = 0x23,     // Battery (Pack) Status
    GHST_DL_GPS_PRIMARY         = 0x25,     // Primary GPS data (position)
    GHST_DL_GPS_SECONDARY       = 0x26,
    GHST_DL_MAGBARO             = 0x27
} ghstDl_e;

#define GHST_RC_CTR_VAL_12BIT       0x7C0   // servo center for 12 bit values (0x3e0 << 1)
#define GHST_RC_CTR_VAL_8BIT        0x7C    // servo center for 8 bit values

#define GHST_FRAME_SIZE             14      // including addr, type, len, crc, and payload

#define GHST_PAYLOAD_SIZE_MAX       14

#define GHST_FRAME_SIZE_MAX         24

#define GPS_FLAGS_FIX               0x01
#define GPS_FLAGS_FIX_HOME          0x02

#define MISC_FLAGS_MAGHEAD          0x01
#define MISC_FLAGS_BAROALT          0x02
#define MISC_FLAGS_VARIO            0x04

typedef struct ghstFrameDef_s {
    uint8_t addr;
    uint8_t len;
    uint8_t type;
    uint8_t payload[GHST_PAYLOAD_SIZE_MAX + 1];         // CRC adds 1
} ghstFrameDef_t;

typedef union ghstFrame_u {
    uint8_t bytes[GHST_FRAME_SIZE];
    ghstFrameDef_t frame;
} ghstFrame_t;


/* Pulses payload (channel data), for 4x 12-bit channels */
typedef struct ghstPayloadServo4_s {
    // 48 bits, or 6 bytes
    unsigned int ch1: 12;
    unsigned int ch2: 12;
    unsigned int ch3: 12;
    unsigned int ch4: 12;
} __attribute__ ((__packed__)) ghstPayloadServo4_t;

/* Pulses payload (channel data). Includes 4x high speed control channels, plus 4 channels from CH5-CH12 */
typedef struct ghstPayloadPulses_s {
    // 80 bits, or 10 bytes
    ghstPayloadServo4_t ch1to4;

    unsigned int cha: 8;
    unsigned int chb: 8;
    unsigned int chc: 8;
    unsigned int chd: 8;
} __attribute__ ((__packed__)) ghstPayloadPulses_t;

/* Pulses payload (channel data), with RSSI/LQ, and other related data */
typedef struct ghstPayloadPulsesRssi_s {
    // 80 bits, or 10 bytes
   ghstPayloadServo4_t ch1to4;

    unsigned int lq: 8;                 // 0-100
    unsigned int rssi: 8;               // 0 - 128 sign inverted, dBm
    unsigned int rfProtocol: 8;
    signed int txPwrdBm: 8;             // tx power in dBm, use lookup table to map to published mW values
} __attribute__ ((__packed__)) ghstPayloadPulsesRssi_t;
