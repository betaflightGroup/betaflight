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

void spiPreinit(void);
void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, uint8_t init);
void spiPreinitByIO(IO_t io);
void spiPreinitByTag(ioTag_t tag);

bool spiInit(SPIDevice device);
void spiResetStream(dmaChannelDescriptor_t *descriptor);
void spiResetDescriptors(busDevice_t *bus);
void spiInitBusDMA();


SPIDevice spiDeviceByInstance(SPI_TypeDef *instance);
SPI_TypeDef *spiInstanceByDevice(SPIDevice device);

//
// BusDevice API
//
// Routines with RB suffix return false if bus is busy on entry
//
bool spiSetBusInstance(extDevice_t *dev, uint32_t device, resourceOwner_e owner);
uint16_t spiCalculateDivider(uint32_t freq);
void spiSetClkDivisor(extDevice_t *dev, uint16_t divider);

void spiSequence(extDevice_t *dev, busSegment_t *segments);
void spiWait(extDevice_t *dev);
void spiSetAtomicWait(extDevice_t *dev);
void spiWaitClaim(extDevice_t *dev);
bool spiIsBusy(extDevice_t *dev);

uint8_t spiReadReg(extDevice_t *dev, uint8_t reg);
uint8_t spiReadRegMsk(extDevice_t *dev, uint8_t reg);
void spiReadRegBuf(extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool spiReadRegBufRB(extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool spiReadRegMskBufRB(extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);

void spiWrite(extDevice_t *dev, uint8_t data);
void spiWriteReg(extDevice_t *dev, uint8_t reg, uint8_t data);
bool spiWriteRegRB(extDevice_t *dev, uint8_t reg, uint8_t data);

uint8_t spiReadWrite(extDevice_t *dev, uint8_t data);

void spiWriteRegBuf(extDevice_t *dev, uint8_t reg, uint8_t *data, uint32_t length);
uint8_t spiReadWriteReg(extDevice_t *dev, uint8_t reg, uint8_t data);
void spiReadWriteBuf(extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len);
bool spiReadWriteBufRB(extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int length);

//
// Config
//

struct spiPinConfig_s;
void spiPinConfigure(const struct spiPinConfig_s *pConfig);
void spiNegateCS(extDevice_t *dev);
bool spiUseDMA(extDevice_t *dev);
void spiBusDeviceRegister(extDevice_t *dev);
uint8_t spiGetRegisteredDeviceCount(void);
uint8_t spiGetExtDeviceCount(extDevice_t *dev);

