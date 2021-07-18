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

#include "drivers/exti.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"

#include "rx/rx.h"

// Used in MSP. Append at end.
typedef enum {
    RX_SPI_NRF24_V202_250K = 0,
    RX_SPI_NRF24_V202_1M,
    RX_SPI_NRF24_SYMA_X,
    RX_SPI_NRF24_SYMA_X5C,
    RX_SPI_NRF24_CX10,
    RX_SPI_NRF24_CX10A,
    RX_SPI_NRF24_H8_3D,
    RX_SPI_NRF24_INAV,
    RX_SPI_FRSKY_D,
    RX_SPI_FRSKY_X,
    RX_SPI_A7105_FLYSKY,
    RX_SPI_A7105_FLYSKY_2A,
    RX_SPI_NRF24_KN,
    RX_SPI_SFHSS,
    RX_SPI_CYRF6936_DSM,
    RX_SPI_FRSKY_X_LBT,
    RX_SPI_REDPINE,
    RX_SPI_FRSKY_X_V2,
    RX_SPI_FRSKY_X_LBT_V2,
    RX_SPI_EXPRESSLRS,
    RX_SPI_PROTOCOL_COUNT
} rx_spi_protocol_e;

typedef enum {
    RX_SPI_RECEIVED_NONE = 0,
    RX_SPI_RECEIVED_BIND = (1 << 0),
    RX_SPI_RECEIVED_DATA = (1 << 1),
    RX_SPI_ROCESSING_REQUIRED = (1 << 2),
} rx_spi_received_e;

// RC channels in AETR order
typedef enum {
    RC_SPI_ROLL = 0,
    RC_SPI_PITCH,
    RC_SPI_THROTTLE,
    RC_SPI_YAW,
    RC_SPI_AUX1,
    RC_SPI_AUX2,
    RC_SPI_AUX3,
    RC_SPI_AUX4,
    RC_SPI_AUX5,
    RC_SPI_AUX6,
    RC_SPI_AUX7,
    RC_SPI_AUX8,
    RC_SPI_AUX9,
    RC_SPI_AUX10,
    RC_SPI_AUX11,
    RC_SPI_AUX12,
    RC_SPI_AUX13,
    RC_SPI_AUX14
} rc_spi_aetr_e;

typedef struct {
    ioConfig_t ioConfig;
    extiTrigger_t trigger;
} rxSpiExtiConfig_t;

// RC channels as used by deviation
#define RC_CHANNEL_RATE        RC_SPI_AUX1
#define RC_CHANNEL_FLIP        RC_SPI_AUX2
#define RC_CHANNEL_PICTURE     RC_SPI_AUX3
#define RC_CHANNEL_VIDEO       RC_SPI_AUX4
#define RC_CHANNEL_HEADLESS    RC_SPI_AUX5
#define RC_CHANNEL_RTH         RC_SPI_AUX6 // return to home

bool rxSpiInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState);
