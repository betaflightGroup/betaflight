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
 *
 * Author: Dominic Clifton
 */

#pragma once

#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

/*
 * Quad SPI supports 1/2/4 wire modes
 *
 * 1LINE is like SPI MISO/MOSI using D0 (MOSI)/D1(MISO).
 * 2LINE uses D0, D1 (bidirectional).
 * 4LINE uses D0..D3 (bidirectional)
 *
 * See ST Micros' AN4760 "Quad-SPI (QSPI) interface on STM32 microcontrollers"
 */

#ifdef USE_QUADSPI

#if !defined(STM32H7)
#error Quad SPI unsupported on this MCU
#endif

#define QUADSPI_IO_AF_BK_IO_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define QUADSPI_IO_AF_CLK_CFG             IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define QUADSPI_IO_AF_BK_CS_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define QUADSPI_IO_BK_CS_CFG              IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)

typedef enum {
    QUADSPI_CLOCK_INITIALIZATION = 256,
    /* QSPI freq = CLK /(1 + ClockPrescaler) = 200 MHz/(1+x) */
    QUADSPI_CLOCK_INITIALISATION = 255, //  0.78125 Mhz
    QUADSPI_CLOCK_SLOW           = 19,  // 10.00000 Mhz
    QUADSPI_CLOCK_STANDARD       = 9,   // 20.00000 MHz
    QUADSPI_CLOCK_FAST           = 3,   // 50.00000 MHz
    QUADSPI_CLOCK_ULTRAFAST      = 1    //100.00000 MHz
} QUADSPIClockDivider_e;

typedef enum QUADSPIDevice {
    QUADSPIINVALID = -1,
    QUADSPIDEV_1   = 0,
} QUADSPIDevice;

#define QUADSPIDEV_COUNT 1

// Macros to convert between CLI bus number and SPIDevice.
#define QUADSPI_CFG_TO_DEV(x)   ((x) - 1)
#define QUADSPI_DEV_TO_CFG(x)   ((x) + 1)

typedef enum {
    QUADSPI_MODE_BK1_ONLY = 0,
    QUADSPI_MODE_BK2_ONLY,
    QUADSPI_MODE_DUAL_FLASH,
} quadSpiMode_e;

//
// QUADSPI1_CS_FLAGS
//
#define QUADSPI_BK1_CS_MASK         ((1 << 1) | (1 << 0))

#define QUADSPI_BK1_CS_NONE         ((0 << 1) | (0 << 0))
#define QUADSPI_BK1_CS_HARDWARE     ((0 << 1) | (1 << 0)) // pin must support QSPI Alternate function for BK1_NCS
#define QUADSPI_BK1_CS_SOFTWARE     ((1 << 1) | (0 << 0)) // use any GPIO pin for BK1 CS

#define QUADSPI_BK2_CS_MASK         ((1 << 3) | (1 << 2))

#define QUADSPI_BK2_CS_NONE         ((0 << 3) | (0 << 2))
#define QUADSPI_BK2_CS_HARDWARE     ((0 << 3) | (1 << 2)) // pin must support QSPI Alternate function for BK2_NCS
#define QUADSPI_BK2_CS_SOFTWARE     ((1 << 3) | (0 << 2)) // use any GPIO pin for BK2 CS

#define QUADSPI_CS_MODE_MASK         (1 << 4)

#define QUADSPI_CS_MODE_SEPARATE    (0 << 4)
#define QUADSPI_CS_MODE_LINKED      (1 << 4)

// H7 QSPI notes:
// Hardware supports BK1 and BK2 connected to both flash chips when using DUAL FLASH mode.
// Hardware does NOT support using BK1_NCS for single flash chip on BK2.
// It's possible to use BK1_NCS for single chip on BK2 using software CS via QUADSPI_BK2_CS_SOFTWARE


void quadSpiPreInit(void);

bool quadSpiInit(QUADSPIDevice device);
void quadSpiSetDivisor(QUADSPI_TypeDef *instance, uint16_t divisor);

bool quadSpiTransmit1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length);
bool quadSpiReceive1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length);
bool quadSpiReceive4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length);

bool quadSpiInstructionWithData1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length);

bool quadSpiReceiveWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length);
bool quadSpiReceiveWithAddress4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length);
bool quadSpiTransmitWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length);
bool quadSpiTransmitWithAddress4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length);


bool quadSpiInstructionWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize);

//bool quadSpiIsBusBusy(SPI_TypeDef *instance);

uint16_t quadSpiGetErrorCounter(QUADSPI_TypeDef *instance);
void quadSpiResetErrorCounter(QUADSPI_TypeDef *instance);

QUADSPIDevice quadSpiDeviceByInstance(QUADSPI_TypeDef *instance);
QUADSPI_TypeDef *quadSpiInstanceByDevice(QUADSPIDevice device);

//
// Config
//

struct quadSpiConfig_s;
void quadSpiPinConfigure(const struct quadSpiConfig_s *pConfig);

#endif
