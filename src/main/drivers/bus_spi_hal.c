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

void spiInitDevice(SPIDevice device, bool leadingEdge)
{
    spiDevice_t *spi = &(spiDevice[device]);

    if (!spi->dev) {
        return;
    }

    spi->leadingEdge = leadingEdge;

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

    if (spi->leadingEdge) {
        spi->hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
        spi->hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    } else {
        spi->hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
        spi->hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    }

    // Init SPI hardware
    HAL_SPI_Init(&spi->hspi);
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t out)
{
    uint8_t in;

    spiTransfer(instance, &out, &in, 1);
    return in;
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (spiDevice[device].hspi.State == HAL_SPI_STATE_BUSY) {
        return true;
    } else {
        return false;
    }
}

bool spiTransfer(SPI_TypeDef *instance, const uint8_t *out, uint8_t *in, int len)
{
    SPIDevice device = spiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    if (!in) {
        // Tx only
        status = HAL_SPI_Transmit(&spiDevice[device].hspi, out, len, SPI_TIMEOUT_SYS_TICKS);
    } else if (!out) {
        // Rx only
        status = HAL_SPI_Receive(&spiDevice[device].hspi, in, len, SPI_TIMEOUT_SYS_TICKS);
    } else {
        // Tx and Rx
        status = HAL_SPI_TransmitReceive(&spiDevice[device].hspi, out, in, len, SPI_TIMEOUT_SYS_TICKS);
    }

    if (status != HAL_OK) {
        spiTimeoutUserCallback(instance);
    }

    return true;
}

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

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
    SPIDevice device = spiDeviceByInstance(instance);

    HAL_SPI_DeInit(&spiDevice[device].hspi);

    spiDevice_t *spi = &(spiDevice[device]);

    int prescalerIndex = ffs(divisor) - 2; // prescaler begins at "/2"

    if (prescalerIndex < 0 || prescalerIndex >= (int)ARRAYLEN(baudRatePrescaler)) {
        return;
    }

    spi->hspi.Init.BaudRatePrescaler = baudRatePrescaler[prescalerIndex];

    HAL_SPI_Init(&spi->hspi);
}

#ifdef USE_DMA
DMA_HandleTypeDef* dmaHandleByInstance(SPI_TypeDef *instance)
{
    return &spiDevice[spiDeviceByInstance(instance)].hdma;
}

void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_1].hspi);
}

void SPI2_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_2].hspi);
}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_3].hspi);
}

void SPI4_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_4].hspi);
}

void SPI5_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_5].hspi);
}

void SPI6_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_6].hspi);
}

void dmaSPIIRQHandler(dmaChannelDescriptor_t* descriptor)
{
    SPIDevice device = descriptor->userParam;
    if (device != SPIINVALID) {
        HAL_DMA_IRQHandler(&spiDevice[device].hdma);
    }
}
#endif // USE_DMA
#endif
