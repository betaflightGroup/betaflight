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

#include "drivers/bus.h"
#include "drivers/io_types.h"
#include "drivers/bus.h"
#include "drivers/rcc_types.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#if defined(STM32F4) || defined(STM32F3)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#elif defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#elif defined(STM32F1)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_50MHz)
#define SPI_IO_AF_MOSI_CFG      IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_50MHz)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_Mode_Out_PP,      GPIO_Speed_50MHz)
#endif

// De facto standard mode
// See https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
//
// Mode CPOL CPHA
//  0    0    0
//  1    0    1
//  2    1    0
//  3    1    1
typedef enum {
    SPI_MODE0_POL_LOW_EDGE_1ST = 0,
    SPI_MODE1_POL_LOW_EDGE_2ND,
    SPI_MODE2_POL_HIGH_EDGE_1ST,
    SPI_MODE3_POL_HIGH_EDGE_2ND
} SPIMode_e;

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4,
    SPIDEV_5,
    SPIDEV_6
} SPIDevice;

#if defined(STM32F1)
#define SPIDEV_COUNT 2
#elif defined(STM32F3) || defined(STM32F4)
#define SPIDEV_COUNT 3
#elif defined(STM32F7)
#define SPIDEV_COUNT 4
#elif defined(STM32H7)
#define SPIDEV_COUNT 6
#else
#define SPIDEV_COUNT 4

#endif

// Macros to convert between CLI bus number and SPIDevice.
#define SPI_CFG_TO_DEV(x)   ((x) - 1)
#define SPI_DEV_TO_CFG(x)   ((x) + 1)

// Work around different check routines in the libraries for different MCU types
#if defined(STM32H7)
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXWNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->RXDR)
#else
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->DR)
#endif

void spiPreinit(void);
void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, uint8_t init);
void spiPreinitByIO(IO_t io);
void spiPreinitByTag(ioTag_t tag);

bool spiInit(SPIDevice device, bool leadingEdge);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data);
bool spiIsBusBusy(SPI_TypeDef *instance);

bool spiTransfer(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len);

uint16_t spiGetErrorCounter(SPI_TypeDef *instance);
void spiResetErrorCounter(SPI_TypeDef *instance);

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance);
SPI_TypeDef *spiInstanceByDevice(SPIDevice device);

//
// BusDevice API
//

bool spiBusIsBusBusy(const busDevice_t *bus);

bool spiBusTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length);

uint8_t spiBusTransferByte(const busDevice_t *bus, uint8_t data);
void spiBusWriteByte(const busDevice_t *bus, uint8_t data);
bool spiBusRawTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int len);

bool spiBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool spiBusRawReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
bool spiBusReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
void spiBusWriteRegisterBuffer(const busDevice_t *bus, uint8_t reg, const uint8_t *data, uint8_t length);
uint8_t spiBusRawReadRegister(const busDevice_t *bus, uint8_t reg);
uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg);
void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance);
uint16_t spiCalculateDivider(uint32_t freq);
void spiBusSetDivisor(busDevice_t *bus, uint16_t divider);

void spiBusTransactionInit(busDevice_t *bus, SPIMode_e mode, uint16_t divider);
void spiBusTransactionSetup(const busDevice_t *bus);
void spiBusTransactionBegin(const busDevice_t *bus);
void spiBusTransactionEnd(const busDevice_t *bus);
bool spiBusTransactionWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
uint8_t spiBusTransactionReadRegister(const busDevice_t *bus, uint8_t reg);
bool spiBusTransactionReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
bool spiBusTransactionTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length);

//
// Config
//

struct spiPinConfig_s;
void spiPinConfigure(const struct spiPinConfig_s *pConfig);
void spiBusDeviceRegister(const busDevice_t *bus);
uint8_t spiGetRegisteredDeviceCount(void);
