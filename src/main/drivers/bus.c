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

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"


// Access routines where the register is accessed directly
bool busRawWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
#ifdef USE_SPI
    if (dev->bus->busType ==  BUS_TYPE_SPI) {
        return spiWriteRegRB(dev, reg, data);
    } else
#endif
    {
        return busWriteRegister(dev, reg, data);
    }
}

bool busRawWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
#ifdef USE_SPI
    if (dev->bus->busType ==  BUS_TYPE_SPI) {
        return spiWriteRegRB(dev, reg, data);
    } else
#endif
    {
        return busWriteRegisterStart(dev, reg, data);
    }
}

bool busRawReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
#ifdef USE_SPI
    if (dev->bus->busType ==  BUS_TYPE_SPI) {
        return spiReadRegBufRB(dev, reg, data, length);
    } else
#endif
    {
        return busReadRegisterBuffer(dev, reg, data, length);
    }
}

bool busRawReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
#ifdef USE_SPI
    if (dev->bus->busType ==  BUS_TYPE_SPI) {
        return spiReadRegBufRB(dev, reg, data, length);
    } else
#endif
    {
        return busReadRegisterBufferStart(dev, reg, data, length);
    }
}


// Write routines where the register is masked with 0x7f
bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif
    switch (dev->bus->busType) {
#ifdef USE_SPI
    case BUS_TYPE_SPI:
        return spiWriteRegRB(dev, reg & 0x7f, data);
#endif
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        return i2cBusWriteRegister(dev, reg, data);
#endif
    default:
        return false;
    }
}

bool busWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif
    switch (dev->bus->busType) {
#ifdef USE_SPI
    case BUS_TYPE_SPI:
        return spiWriteRegRB(dev, reg & 0x7f, data);
#endif
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        return i2cBusWriteRegisterStart(dev, reg, data);
#endif
    default:
        return false;
    }
}

// Read routines where the register is ORed with 0x80
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (dev->bus->busType) {
#ifdef USE_SPI
    case BUS_TYPE_SPI:
        return spiReadRegMskBufRB(dev, reg | 0x80, data, length);
#endif
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        return i2cBusReadRegisterBuffer(dev, reg, data, length);
#endif
    default:
        return false;
    }
}

// Start the I2C read, but do not wait for completion
bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (dev->bus->busType) {
#ifdef USE_SPI
    case BUS_TYPE_SPI:
        // For SPI allow the transaction to complete
        return spiReadRegMskBufRB(dev, reg | 0x80, data, length);
#endif
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        // Initiate the read access
        return i2cBusReadRegisterBufferStart(dev, reg, data, length);
#endif
    default:
        return false;
    }
}

// Returns true if bus is still busy
bool busBusy(const extDevice_t *dev, bool *error)
{
#if !defined(USE_I2C)
    UNUSED(error);
#endif
    switch (dev->bus->busType) {
#ifdef USE_SPI
    case BUS_TYPE_SPI:
        // No waiting on SPI
        return false;
#endif

#ifdef USE_I2C
    case BUS_TYPE_I2C:
        return i2cBusBusy(dev, error);
#endif

    default:
        return false;
    }
}

uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(dev);
    UNUSED(reg);
    return false;
#else
    uint8_t data;
    busReadRegisterBuffer(dev, reg, &data, 1);
    return data;
#endif
}

void busDeviceRegister(const extDevice_t *dev)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(dev);
#endif

    switch (dev->bus->busType) {
#if defined(USE_SPI)
    case BUS_TYPE_SPI:
        spiBusDeviceRegister(dev);

        break;
#endif
#if defined(USE_I2C)
    case BUS_TYPE_I2C:
        i2cBusDeviceRegister(dev);

        break;
#endif
    default:
        break;
    }
}
