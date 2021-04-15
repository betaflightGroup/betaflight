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

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

static uint8_t spiRegisteredDeviceCount = 0;

spiDevice_t spiDevice[SPIDEV_COUNT];

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1)
        return SPIDEV_1;
#endif

#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2)
        return SPIDEV_2;
#endif

#ifdef USE_SPI_DEVICE_3
    if (instance == SPI3)
        return SPIDEV_3;
#endif

#ifdef USE_SPI_DEVICE_4
    if (instance == SPI4)
        return SPIDEV_4;
#endif

#ifdef USE_SPI_DEVICE_5
    if (instance == SPI5)
        return SPIDEV_5;
#endif

#ifdef USE_SPI_DEVICE_6
    if (instance == SPI6)
        return SPIDEV_6;
#endif

    return SPIINVALID;
}

SPI_TypeDef *spiInstanceByDevice(SPIDevice device)
{
    if (device == SPIINVALID || device >= SPIDEV_COUNT) {
        return NULL;
    }

    return spiDevice[device].dev;
}

bool spiInit(SPIDevice device, bool leadingEdge)
{
    switch (device) {
    case SPIINVALID:
        return false;

    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3) && !defined(STM32F1)
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_4:
#if defined(USE_SPI_DEVICE_4)
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_5:
#if defined(USE_SPI_DEVICE_5)
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_6:
#if defined(USE_SPI_DEVICE_6)
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif
    }
    return false;
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return -1;
    }
    spiDevice[device].errorCount++;
    return spiDevice[device].errorCount;
}

bool spiBusTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length)
{
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransfer(bus->busdev_u.spi.instance, txData, rxData, length);
    IOHi(bus->busdev_u.spi.csnPin);
    return true;
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return 0;
    }
    return spiDevice[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID) {
        spiDevice[device].errorCount = 0;
    }
}

bool spiBusIsBusBusy(const busDevice_t *bus)
{
    return spiIsBusBusy(bus->busdev_u.spi.instance);
}

uint8_t spiBusTransferByte(const busDevice_t *bus, uint8_t data)
{
    return spiTransferByte(bus->busdev_u.spi.instance, data);
}

void spiBusWriteByte(const busDevice_t *bus, uint8_t data)
{
    IOLo(bus->busdev_u.spi.csnPin);
    spiBusTransferByte(bus, data);
    IOHi(bus->busdev_u.spi.csnPin);
}

bool spiBusRawTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int len)
{
    return spiTransfer(bus->busdev_u.spi.instance, txData, rxData, len);
}

bool spiBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransferByte(bus->busdev_u.spi.instance, reg);
    spiTransferByte(bus->busdev_u.spi.instance, data);
    IOHi(bus->busdev_u.spi.csnPin);

    return true;
}

bool spiBusRawReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransferByte(bus->busdev_u.spi.instance, reg);
    spiTransfer(bus->busdev_u.spi.instance, NULL, data, length);
    IOHi(bus->busdev_u.spi.csnPin);

    return true;
}

bool spiBusReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiBusRawReadRegisterBuffer(bus, reg | 0x80, data, length);
}

void spiBusWriteRegisterBuffer(const busDevice_t *bus, uint8_t reg, const uint8_t *data, uint8_t length)
{
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransferByte(bus->busdev_u.spi.instance, reg);
    spiTransfer(bus->busdev_u.spi.instance, data, NULL, length);
    IOHi(bus->busdev_u.spi.csnPin);
}

uint8_t spiBusRawReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data;
    IOLo(bus->busdev_u.spi.csnPin);
    spiTransferByte(bus->busdev_u.spi.instance, reg);
    spiTransfer(bus->busdev_u.spi.instance, NULL, &data, 1);
    IOHi(bus->busdev_u.spi.csnPin);

    return data;
}

uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg)
{
    return spiBusRawReadRegister(bus, reg | 0x80);
}

void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance)
{
    bus->bustype = BUSTYPE_SPI;
    bus->busdev_u.spi.instance = instance;
}

uint16_t spiCalculateDivider(uint32_t freq)
{
#if defined(STM32F4) || defined(STM32G4) || defined(STM32F7)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
#else
#error "Base SPI clock not defined for this architecture"
#endif

    uint16_t divisor = 2;

    spiClk >>= 1;

    for (; (spiClk > freq) && (divisor < 256); divisor <<= 1, spiClk >>= 1);

    return divisor;
}

void spiBusSetDivisor(busDevice_t *bus, uint16_t divisor)
{
    spiSetDivisor(bus->busdev_u.spi.instance, divisor);
    // bus->busdev_u.spi.modeCache = bus->busdev_u.spi.instance->CR1;
}

#ifdef USE_SPI_TRANSACTION
// Separate set of spiBusTransactionXXX to keep fast path for acc/gyros.

void spiBusTransactionBegin(const busDevice_t *bus)
{
    spiBusTransactionSetup(bus);
    IOLo(bus->busdev_u.spi.csnPin);
}

void spiBusTransactionEnd(const busDevice_t *bus)
{
    IOHi(bus->busdev_u.spi.csnPin);
}

bool spiBusTransactionTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length)
{
    spiBusTransactionSetup(bus);
    return spiBusTransfer(bus, txData, rxData, length);
}

bool spiBusTransactionWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    spiBusTransactionSetup(bus);
    return spiBusWriteRegister(bus, reg, data);
}

uint8_t spiBusTransactionReadRegister(const busDevice_t *bus, uint8_t reg)
{
    spiBusTransactionSetup(bus);
    return spiBusReadRegister(bus, reg);
}

bool spiBusTransactionReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    spiBusTransactionSetup(bus);
    return spiBusReadRegisterBuffer(bus, reg, data, length);
}
#endif // USE_SPI_TRANSACTION

void spiBusDeviceRegister(const busDevice_t *bus)
{
    UNUSED(bus);

    spiRegisteredDeviceCount++;
}

uint8_t spiGetRegisteredDeviceCount(void)
{
    return spiRegisteredDeviceCount;
}
#endif
