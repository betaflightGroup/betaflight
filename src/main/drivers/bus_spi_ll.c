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
#include <string.h>

#include "platform.h"

#if defined(USE_SPI)

#include "common/utils.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"

#ifndef SPI2_SCK_PIN
#define SPI2_NSS_PIN    PB12
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_NSS_PIN    PA15
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

#ifndef SPI4_SCK_PIN
#define SPI4_NSS_PIN    PA15
#define SPI4_SCK_PIN    PB3
#define SPI4_MISO_PIN   PB4
#define SPI4_MOSI_PIN   PB5
#endif

#ifndef SPI1_NSS_PIN
#define SPI1_NSS_PIN NONE
#endif
#ifndef SPI2_NSS_PIN
#define SPI2_NSS_PIN NONE
#endif
#ifndef SPI3_NSS_PIN
#define SPI3_NSS_PIN NONE
#endif
#ifndef SPI4_NSS_PIN
#define SPI4_NSS_PIN NONE
#endif

static LL_SPI_InitTypeDef defaultInit =
{
    .TransferDirection = SPI_DIRECTION_2LINES,
    .Mode = SPI_MODE_MASTER,
    .DataWidth = SPI_DATASIZE_8BIT,
    .NSS = SPI_NSS_SOFT,
    .BaudRate = SPI_BAUDRATEPRESCALER_8,
    .BitOrder = SPI_FIRSTBIT_MSB,
    .CRCPoly = 7,
    .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
};

void spiInitDevice(SPIDevice device, bool leadingEdge)
{
    spiDevice_t *spi = &(spiDevice[device]);

    if (!spi->dev) {
        return;
    }

#ifndef USE_SPI_TRANSACTION
    spi->leadingEdge = leadingEdge;
#else
    UNUSED(leadingEdge);
#endif

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

    if (spi->leadingEdge == true) {
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->sckAF);
    } else {
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
    }

    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);

    LL_SPI_Disable(spi->dev);
    LL_SPI_DeInit(spi->dev);

#ifndef USE_SPI_TRANSACTION
    if (spi->leadingEdge) {
        defaultInit.ClockPolarity = SPI_POLARITY_LOW;
        defaultInit.ClockPhase = SPI_PHASE_1EDGE;
    } else
#endif
    {
        defaultInit.ClockPolarity = SPI_POLARITY_HIGH;
        defaultInit.ClockPhase = SPI_PHASE_2EDGE;
    }

    LL_SPI_SetRxFIFOThreshold(spi->dev, SPI_RXFIFO_THRESHOLD_QF);

    LL_SPI_Init(spi->dev, &defaultInit);
    LL_SPI_Enable(spi->dev);
}

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t txByte)
{
    timeUs_t timeoutStartUs = microsISR();

    while (!LL_SPI_IsActiveFlag_TXE(instance)) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= SPI_TIMEOUT_US) {
            return spiTimeoutUserCallback(instance);
        }
    }
    LL_SPI_TransmitData8(instance, txByte);

    timeoutStartUs = microsISR();
    while (!CHECK_SPI_RX_DATA_AVAILABLE(instance)) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= SPI_TIMEOUT_US) {
            return spiTimeoutUserCallback(instance);
        }
    }

    return (uint8_t)LL_SPI_ReceiveData8(instance);
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
    return LL_SPI_GetTxFIFOLevel(instance) != LL_SPI_TX_FIFO_EMPTY
        || LL_SPI_IsActiveFlag_BSY(instance);
}

bool spiTransfer(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    timeUs_t timeoutStartUs;
    
    // set 16-bit transfer
    CLEAR_BIT(instance->CR2, SPI_RXFIFO_THRESHOLD);
    while (len > 1) {
        timeoutStartUs = microsISR();
        while (!LL_SPI_IsActiveFlag_TXE(instance)) {
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= SPI_TIMEOUT_US) {
                return spiTimeoutUserCallback(instance);
            }
        }
        uint16_t w;
        if (txData) {
            w = *((uint16_t *)txData);
            txData += 2;
        } else {
            w = 0xFFFF;
        }
        LL_SPI_TransmitData16(instance, w);

        timeoutStartUs = microsISR();
        while (!CHECK_SPI_RX_DATA_AVAILABLE(instance)) {
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= SPI_TIMEOUT_US) {
                return spiTimeoutUserCallback(instance);
            }
        }
        w = LL_SPI_ReceiveData16(instance);
        if (rxData) {
            *((uint16_t *)rxData) = w;
            rxData += 2;
        }
        len -= 2;
    }
    // set 8-bit transfer
    SET_BIT(instance->CR2, SPI_RXFIFO_THRESHOLD);
    if (len) {
        timeoutStartUs = microsISR();
        while (!LL_SPI_IsActiveFlag_TXE(instance)) {
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= SPI_TIMEOUT_US) {
                return spiTimeoutUserCallback(instance);
            }
        }
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);

        timeoutStartUs = microsISR();
        while (!CHECK_SPI_RX_DATA_AVAILABLE(instance)) {
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= SPI_TIMEOUT_US) {
                return spiTimeoutUserCallback(instance);
            }
        }
        b = LL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }

    return true;
}

static uint16_t spiDivisorToBRbits(SPI_TypeDef *instance, uint16_t divisor)
{
#if !(defined(STM32F1) || defined(STM32F3))
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.

    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }
#else
    UNUSED(instance);
#endif

    divisor = constrain(divisor, 2, 256);

    return (ffs(divisor) - 2) << SPI_CR1_BR_Pos;
}

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
    LL_SPI_Disable(instance);
    LL_SPI_SetBaudRatePrescaler(instance, spiDivisorToBRbits(instance, divisor));
    LL_SPI_Enable(instance);
}

#ifdef USE_SPI_TRANSACTION
void spiBusTransactionInit(busDevice_t *bus, SPIMode_e mode, uint16_t divisor)
{
    switch (mode) {
    case SPI_MODE0_POL_LOW_EDGE_1ST:
        defaultInit.ClockPolarity = SPI_POLARITY_LOW;
        defaultInit.ClockPhase = SPI_PHASE_1EDGE;
        break;
    case SPI_MODE1_POL_LOW_EDGE_2ND:
        defaultInit.ClockPolarity = SPI_POLARITY_LOW;
        defaultInit.ClockPhase = SPI_PHASE_2EDGE;
        break;
    case SPI_MODE2_POL_HIGH_EDGE_1ST:
        defaultInit.ClockPolarity = SPI_POLARITY_HIGH;
        defaultInit.ClockPhase = SPI_PHASE_1EDGE;
        break;
    case SPI_MODE3_POL_HIGH_EDGE_2ND:
        defaultInit.ClockPolarity = SPI_POLARITY_HIGH;
        defaultInit.ClockPhase = SPI_PHASE_2EDGE;
        break;
    }

    LL_SPI_Disable(bus->busdev_u.spi.instance);
    LL_SPI_DeInit(bus->busdev_u.spi.instance);

    LL_SPI_Init(bus->busdev_u.spi.instance, &defaultInit);
    LL_SPI_SetBaudRatePrescaler(bus->busdev_u.spi.instance, spiDivisorToBRbits(bus->busdev_u.spi.instance, divisor));

    // Configure for 8-bit reads. XXX Is this STM32F303xC specific?
    LL_SPI_SetRxFIFOThreshold(bus->busdev_u.spi.instance, SPI_RXFIFO_THRESHOLD_QF);

    LL_SPI_Enable(bus->busdev_u.spi.instance);

    bus->busdev_u.spi.device = &spiDevice[spiDeviceByInstance(bus->busdev_u.spi.instance)];
    bus->busdev_u.spi.modeCache = bus->busdev_u.spi.instance->CR1;
}

void spiBusTransactionSetup(const busDevice_t *bus)
{
    // We rely on MSTR bit to detect valid modeCache

    if (bus->busdev_u.spi.modeCache && bus->busdev_u.spi.modeCache != bus->busdev_u.spi.device->cr1SoftCopy) {
        bus->busdev_u.spi.instance->CR1 = bus->busdev_u.spi.modeCache;
        bus->busdev_u.spi.device->cr1SoftCopy = bus->busdev_u.spi.modeCache;

        // SCK seems to require some time to switch to a new initial level after CR1 is written.
        // Here we buy some time in addition to the software copy save above.
        __asm__("nop");
    }
}
#endif // USE_SPI_TRANSACTION
#endif
