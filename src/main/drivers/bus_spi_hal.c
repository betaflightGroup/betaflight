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
 * HAL version resurrected from v3.1.7 (by jflyper)
 */

// Note that the HAL driver is polled only

#include <stdbool.h>
#include <stdint.h>
#include <strings.h>

#include "platform.h"

#ifdef USE_SPI

#include "bus_spi.h"
#include "bus_spi_impl.h"
#include "dma.h"
#include "io.h"
#include "io_impl.h"
#include "nvic.h"
#include "rcc.h"

#define SPI_TIMEOUT_SYS_TICKS   (SPI_TIMEOUT_US / 1000)

// Position of Prescaler bits are different from MCU to MCU

static uint32_t baudRatePrescaler[8] = {
    SPI_BAUDRATEPRESCALER_2,
    SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,
    SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,
    SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128,
    SPI_BAUDRATEPRESCALER_256,
};

static void spiDivisorToBRbits(SPI_TypeDef *instance, uint16_t divisor)
{
    SPIDevice device = spiDeviceByInstance(instance);

    spiDevice_t *spi = &(spiDevice[device]);

    int prescalerIndex = ffs(divisor) - 2; // prescaler begins at "/2"

    if (prescalerIndex < 0 || prescalerIndex >= (int)ARRAYLEN(baudRatePrescaler)) {
        return;
    }

    if (spi->hspi.Init.BaudRatePrescaler != baudRatePrescaler[prescalerIndex]) {
        spi->hspi.Init.BaudRatePrescaler = baudRatePrescaler[prescalerIndex];

        MODIFY_REG(spi->hspi.Instance->CR1, SPI_CR1_BR_Msk, spi->hspi.Init.BaudRatePrescaler);
    }
}

static void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
    spiDivisorToBRbits(instance, divisor);
}

void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &(spiDevice[device]);

    if (!spi->dev) {
        return;
    }

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

#if defined(STM32F3)
    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);
#endif

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    IOConfigGPIOAF(IOGetByTag(spi->sck), spi->leadingEdge ? SPI_IO_AF_SCK_CFG_LOW : SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);
#endif

#if defined(STM32F10X)
    IOConfigGPIO(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG);
    IOConfigGPIO(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG);
    IOConfigGPIO(IOGetByTag(spi->mosi), SPI_IO_AF_MOSI_CFG);
#endif

    spi->hspi.Instance = spi->dev;
    // DeInit SPI hardware
    HAL_SPI_DeInit(&spi->hspi);

    spi->hspi.Init.Mode = SPI_MODE_MASTER;
    spi->hspi.Init.Direction = SPI_DIRECTION_2LINES;
    spi->hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi->hspi.Init.NSS = SPI_NSS_SOFT;
    spi->hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi->hspi.Init.CRCPolynomial = 7;
    spi->hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    spi->hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi->hspi.Init.TIMode = SPI_TIMODE_DISABLED;
#if !defined(STM32G4)
    spi->hspi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spi->hspi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  /* Recommanded setting to avoid glitches */
#endif
    spi->hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi->hspi.Init.CLKPhase = SPI_PHASE_1EDGE;

    // Init SPI hardware
    HAL_SPI_Init(&spi->hspi);
}

static bool spiInternalReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    SPIDevice device = spiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    if (!rxData) {
        // Tx only
        status = HAL_SPI_Transmit(&spiDevice[device].hspi, txData, len, SPI_TIMEOUT_SYS_TICKS);
    } else if(!txData) {
        // Rx only
        status = HAL_SPI_Receive(&spiDevice[device].hspi, rxData, len, SPI_TIMEOUT_SYS_TICKS);
    } else {
        // Tx and Rx
        status = HAL_SPI_TransmitReceive(&spiDevice[device].hspi, txData, rxData, len, SPI_TIMEOUT_SYS_TICKS);
    }

    return (status == HAL_OK);
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    UNUSED(dev);
    UNUSED(preInit);
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    UNUSED(dev);
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    UNUSED(dev);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    UNUSED(bus);
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    UNUSED(descriptor);
}

// Transfer setup and start
void spiSequence(const extDevice_t *dev, busSegment_t *segments)
{
    busDevice_t *bus = dev->bus;
    SPIDevice device = spiDeviceByInstance(bus->busType_u.spi.instance);
    SPI_HandleTypeDef *hspi = &spiDevice[device].hspi;

    bus->initSegment = true;
    bus->curSegment = segments;

    // Switch bus speed
    spiSetDivisor(bus->busType_u.spi.instance, dev->busType_u.spi.speed);

    // Switch SPI clock polarity/phase if necessary
    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        if (dev->busType_u.spi.leadingEdge){
            hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
        } else {
            hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
        }
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;

        // Init SPI hardware
        HAL_SPI_Init(hspi);
    }

    // Manually work through the segment list performing a transfer for each
    while (bus->curSegment->len) {
        // Assert Chip Select
        IOLo(dev->busType_u.spi.csnPin);

        spiInternalReadWriteBufPolled(
                bus->busType_u.spi.instance,
                bus->curSegment->txData,
                bus->curSegment->rxData,
                bus->curSegment->len);

        if (bus->curSegment->negateCS) {
            // Negate Chip Select
            IOHi(dev->busType_u.spi.csnPin);
        }

        if (bus->curSegment->callback) {
            switch(bus->curSegment->callback(dev->callbackArg)) {
            case BUS_BUSY:
                // Repeat the last DMA segment
                bus->curSegment--;
                break;

            case BUS_ABORT:
                bus->curSegment = (busSegment_t *)NULL;
                return;

            case BUS_READY:
            default:
                // Advance to the next DMA segment
                break;
            }
        }
        bus->curSegment++;
    }

    bus->curSegment = (busSegment_t *)NULL;
}
#endif
