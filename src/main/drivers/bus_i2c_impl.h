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

#include "platform.h"

#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

#define I2C_TIMEOUT_US          10000
#define I2C_TIMEOUT_SYS_TICKS   (I2C_TIMEOUT_US / 1000)

#define I2C_PIN_SEL_MAX 4

typedef struct i2cPinDef_s {
    ioTag_t ioTag;
#if defined(STM32F4) || defined(STM32H7) || defined(STM32G4)
    uint8_t af;
#endif
} i2cPinDef_t;

#if defined(STM32F4) || defined(STM32H7) || defined(STM32G4)
#define I2CPINDEF(pin, af) { DEFIO_TAG_E(pin), af }
#elif defined(STM32F1)
#define I2CPINDEF(pin, af) { DEFIO_TAG_E(pin) }
#else
#define I2CPINDEF(pin) { DEFIO_TAG_E(pin) }
#endif

typedef struct i2cHardware_s {
    I2CDevice device;
    I2C_TypeDef *reg;
    i2cPinDef_t sclPins[I2C_PIN_SEL_MAX];
    i2cPinDef_t sdaPins[I2C_PIN_SEL_MAX];
    rccPeriphTag_t rcc;
#if !defined(STM32F303xC)
    uint8_t ev_irq;
    uint8_t er_irq;
#endif
} i2cHardware_t;

extern const i2cHardware_t i2cHardware[];

#if defined(STM32F1) || defined(STM32F4)
typedef struct i2cState_s {
    volatile bool error;
    volatile bool busy;
    volatile uint8_t addr;
    volatile uint8_t reg;
    volatile uint8_t bytes;
    volatile uint8_t writing;
    volatile uint8_t reading;
    volatile uint8_t* write_p;
    volatile uint8_t* read_p;
} i2cState_t;
#endif

typedef struct i2cDevice_s {
    const i2cHardware_t *hardware;
    I2C_TypeDef *reg;
    IO_t scl;
    IO_t sda;
#if defined(STM32F4) || defined(STM32H7) || defined(STM32G4)
    uint8_t sclAF;
    uint8_t sdaAF;
#endif
    bool pullUp;
    uint16_t clockSpeed;

    // MCU/Driver dependent member follows
#if defined(STM32F1) || defined(STM32F4)
    i2cState_t state;
#endif
#ifdef USE_HAL_DRIVER
    I2C_HandleTypeDef handle;
#endif
} i2cDevice_t;

extern i2cDevice_t i2cDevice[];
