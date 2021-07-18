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

// This file is copied with modifications from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RX_SPI

#include "build/build_config.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "pg/rx_spi.h"

#include "rx_spi.h"

// 10 MHz max SPI frequency
#define RX_MAX_SPI_CLK_HZ 10000000

#define ENABLE_RX() IOLo(busdev->busdev_u.spi.csnPin)
#define DISABLE_RX() IOHi(busdev->busdev_u.spi.csnPin)

static busDevice_t rxSpiDevice;
static busDevice_t *busdev = &rxSpiDevice;

static IO_t extiPin = IO_NONE;
static extiCallbackRec_t rxSpiExtiCallbackRec;
static bool extiLevel = true;

static volatile bool extiHasOccurred = false;
static volatile timeUs_t lastExtiTimeUs = 0;

void rxSpiDevicePreInit(const rxSpiConfig_t *rxSpiConfig)
{
    spiPreinitRegister(rxSpiConfig->csnTag, IOCFG_IPU, 1);
}

void rxSpiExtiHandler(extiCallbackRec_t* callback)
{
    UNUSED(callback);

    const timeUs_t extiTimeUs = microsISR();

    if (IORead(extiPin) == extiLevel) {
        lastExtiTimeUs = extiTimeUs;
        extiHasOccurred = true;
    }
}

bool rxSpiDeviceInit(const rxSpiConfig_t *rxSpiConfig)
{
    SPI_TypeDef *instance = spiInstanceByDevice(SPI_CFG_TO_DEV(rxSpiConfig->spibus));

    if (!instance) {
        return false;
    }

    spiBusSetInstance(busdev, instance);

    const IO_t rxCsPin = IOGetByTag(rxSpiConfig->csnTag);
    IOInit(rxCsPin, OWNER_RX_SPI_CS, 0);
    IOConfigGPIO(rxCsPin, SPI_IO_CS_CFG);
    busdev->busdev_u.spi.csnPin = rxCsPin;

    IOHi(rxCsPin);
#ifdef USE_SPI_TRANSACTION
    spiBusTransactionInit(busdev, SPI_MODE0_POL_LOW_EDGE_1ST, spiCalculateDivider(RX_MAX_SPI_CLK_HZ));
#else
    spiBusSetDivisor(busdev, spiCalculateDivider(RX_MAX_SPI_CLK_HZ));
#endif

    extiPin = IOGetByTag(rxSpiConfig->extiIoTag);

    if (extiPin) {
        IOInit(extiPin, OWNER_RX_SPI_EXTI, 0);
    }

    return true;
}

void rxSpiExtiInit(ioConfig_t rxSpiExtiPinConfig, extiTrigger_t rxSpiExtiPinTrigger)
{
    if (extiPin) {
        if (rxSpiExtiPinTrigger == BETAFLIGHT_EXTI_TRIGGER_FALLING) {
            extiLevel = false;
        }
        EXTIHandlerInit(&rxSpiExtiCallbackRec, rxSpiExtiHandler);
        EXTIConfig(extiPin, &rxSpiExtiCallbackRec, NVIC_PRIO_MPU_INT_EXTI, rxSpiExtiPinConfig, rxSpiExtiPinTrigger);
        EXTIEnable(extiPin, true);
    }
}


uint8_t rxSpiTransferByte(uint8_t data)
{
    return spiBusTransferByte(busdev, data);
}

void rxSpiWriteByte(uint8_t data)
{
    spiBusWriteByte(busdev, data);
}

void rxSpiWriteCommand(uint8_t command, uint8_t data)
{
    spiBusWriteRegister(busdev, command, data);
}

void rxSpiWriteCommandMulti(uint8_t command, const uint8_t *data, uint8_t length)
{
    spiBusWriteRegisterBuffer(busdev, command, data, length);
}

uint8_t rxSpiReadCommand(uint8_t command, uint8_t data)
{
    UNUSED(data);
    return spiBusRawReadRegister(busdev, command);
}

void rxSpiReadCommandMulti(uint8_t command, uint8_t commandData, uint8_t *retData, uint8_t length)
{
    UNUSED(commandData);
    spiBusRawReadRegisterBuffer(busdev, command, retData, length);
}

bool rxSpiExtiConfigured(void)
{
    return extiPin != IO_NONE;
}

bool rxSpiGetExtiState(void)
{
    return IORead(extiPin);
}

bool rxSpiPollExti(void)
{
    return extiHasOccurred;
}

void rxSpiResetExti(void)
{
    extiHasOccurred = false;
}

timeUs_t rxSpiGetLastExtiTimeUs(void)
{
    return lastExtiTimeUs;
}

void rxSpiTransferCommandMulti(uint8_t *data, uint8_t length)
{
    ENABLE_RX();
    spiBusRawTransfer(busdev, data, data, length);
    DISABLE_RX();
}

#endif
