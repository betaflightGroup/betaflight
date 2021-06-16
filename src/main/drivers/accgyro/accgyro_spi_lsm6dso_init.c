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

#ifdef USE_ACCGYRO_LSM6DSO

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_lsm6dso.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// 10 MHz max SPI frequency
#define LSM6DSO_MAX_SPI_CLK_HZ 10000000

#define LSM6DSO_CHIP_ID 0x6C

// LSM6DSO register configuration values
typedef enum {
    LSM6DSO_VAL_INT1_CTRL = 0x02,             // enable gyro data ready interrupt pin 1
    LSM6DSO_VAL_INT2_CTRL = 0x02,             // enable gyro data ready interrupt pin 2
    LSM6DSO_VAL_CTRL1_XL_ODR833 = 0x07,       // accelerometer 833hz output data rate (gyro/8)
    LSM6DSO_VAL_CTRL1_XL_ODR1667 = 0x08,      // accelerometer 1666hz output data rate (gyro/4)
    LSM6DSO_VAL_CTRL1_XL_ODR3332 = 0x09,      // accelerometer 3332hz output data rate (gyro/2)
    LSM6DSO_VAL_CTRL1_XL_ODR3333 = 0x0A,      // accelerometer 6664hz output data rate (gyro/1)
    LSM6DSO_VAL_CTRL1_XL_8G = 0x03,           // accelerometer 8G scale
    LSM6DSO_VAL_CTRL1_XL_16G = 0x01,          // accelerometer 16G scale
    LSM6DSO_VAL_CTRL1_XL_LPF1 = 0x00,         // accelerometer output from LPF1
    LSM6DSO_VAL_CTRL1_XL_LPF2 = 0x01,         // accelerometer output from LPF2
    LSM6DSO_VAL_CTRL2_G_ODR6664 = 0x0A,       // gyro 6664hz output data rate
    LSM6DSO_VAL_CTRL2_G_2000DPS = 0x03,       // gyro 2000dps scale
    LSM6DSO_VAL_CTRL3_C_BDU = BIT(6),         // (bit 6) output registers are not updated until MSB and LSB have been read (prevents MSB from being updated while burst reading LSB/MSB)
    LSM6DSO_VAL_CTRL3_C_H_LACTIVE = 0,        // (bit 5) interrupt pins active high
    LSM6DSO_VAL_CTRL3_C_PP_OD = 0,            // (bit 4) interrupt pins push/pull
    LSM6DSO_VAL_CTRL3_C_SIM = 0,              // (bit 3) SPI 4-wire interface mode
    LSM6DSO_VAL_CTRL3_C_IF_INC = BIT(2),      // (bit 2) auto-increment address for burst reads
    LSM6DSO_VAL_CTRL4_C_I2C_DISABLE = BIT(2), // (bit 2) disable I2C interface
    LSM6DSO_VAL_CTRL4_C_LPF1_SEL_G = BIT(1),  // (bit 1) enable gyro LPF1
    LSM6DSO_VAL_CTRL6_C_XL_HM_MODE = 0,       // (bit 4) enable accelerometer high performance mode
    LSM6DSO_VAL_CTRL6_C_FTYPE_335HZ = 0x00,   // (bits 2:0) gyro LPF1 cutoff 335.5hz
    LSM6DSO_VAL_CTRL6_C_FTYPE_232HZ = 0x01,   // (bits 2:0) gyro LPF1 cutoff 232.0hz
    LSM6DSO_VAL_CTRL6_C_FTYPE_171HZ = 0x02,   // (bits 2:0) gyro LPF1 cutoff 171.1hz
    LSM6DSO_VAL_CTRL6_C_FTYPE_609HZ = 0x03,   // (bits 2:0) gyro LPF1 cutoff 609.0hz
    LSM6DSO_VAL_CTRL9_XL_I3C_DISABLE = BIT(1),// (bit 1) disable I3C interface
} lsm6dsoConfigValues_e;

// LSM6DSO register configuration bit masks
typedef enum {
    LSM6DSO_MASK_CTRL3_C = 0x7C,         // 0b01111100
    LSM6DSO_MASK_CTRL3_C_RESET = BIT(0), // 0b00000001
    LSM6DSO_MASK_CTRL4_C = 0x06,         // 0b00000110
    LSM6DSO_MASK_CTRL6_C = 0x17,         // 0b00010111
    LSM6DSO_MASK_CTRL9_XL = 0x02,        // 0b00000010
} lsm6dsoConfigMasks_e;

uint8_t lsm6dsoDetect(const extDevice_t *dev)
{
    uint8_t chipID = 0;
    spiSetClkDivisor(dev, spiCalculateDivider(LSM6DSO_MAX_SPI_CLK_HZ));

    if (busReadRegisterBuffer(dev, LSM6DSO_REG_WHO_AM_I, &chipID, 1)) {
        if (chipID == LSM6DSO_CHIP_ID) {
            return LSM6DSO_SPI;
        }
    }

    return MPU_NONE;
}

static void lsm6dsoWriteRegister(const extDevice_t *dev, lsm6dsoRegister_e registerID, uint8_t value, unsigned delayMs)
{
    busWriteRegister(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void lsm6dsoWriteRegisterBits(const extDevice_t *dev, lsm6dsoRegister_e registerID, lsm6dsoConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busReadRegisterBuffer(dev, registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        lsm6dsoWriteRegister(dev, registerID, newValue, delayMs);
    }
}

static void lsm6dsoConfig(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Reset the device (wait 100ms before continuing config)
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C_RESET, BIT(0), 100);

    // Configure interrupt pin 1 for gyro data ready only
    lsm6dsoWriteRegister(dev, LSM6DSO_REG_INT1_CTRL, LSM6DSO_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    lsm6dsoWriteRegister(dev, LSM6DSO_REG_INT2_CTRL, LSM6DSO_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF1 output
    lsm6dsoWriteRegister(dev, LSM6DSO_REG_CTRL1_XL, (LSM6DSO_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DSO_VAL_CTRL1_XL_16G << 2) | (LSM6DSO_VAL_CTRL1_XL_LPF1 << 1), 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    lsm6dsoWriteRegister(dev, LSM6DSO_REG_CTRL2_G, (LSM6DSO_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DSO_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C, (LSM6DSO_VAL_CTRL3_C_BDU | LSM6DSO_VAL_CTRL3_C_H_LACTIVE | LSM6DSO_VAL_CTRL3_C_PP_OD | LSM6DSO_VAL_CTRL3_C_SIM | LSM6DSO_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // enable accelerometer high performane mode; set gyro LPF1 cutoff to 335.5hz
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL4_C, LSM6DSO_MASK_CTRL4_C, (LSM6DSO_VAL_CTRL4_C_I2C_DISABLE | LSM6DSO_VAL_CTRL4_C_LPF1_SEL_G), 1);

    // Configure control register 6
    // disable I2C interface; enable gyro LPF1
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL6_C, LSM6DSO_MASK_CTRL6_C, (LSM6DSO_VAL_CTRL6_C_XL_HM_MODE | LSM6DSO_VAL_CTRL6_C_FTYPE_335HZ), 1);

    // Configure control register 9
    // disable I3C interface
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL9_XL, LSM6DSO_MASK_CTRL9_XL, LSM6DSO_VAL_CTRL9_XL_I3C_DISABLE, 1);
}

#if defined(USE_GYRO_EXTI) && defined(USE_MPU_DATA_READY_SIGNAL)
static void lsm6dsoIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, lsm6dsoExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO, true);
}
#endif

static void lsm6dsoSpiGyroInit(gyroDev_t *gyro)
{
    lsm6dsoConfig(gyro);

#if defined(USE_GYRO_EXTI) && defined(USE_MPU_DATA_READY_SIGNAL)
    lsm6dsoIntExtiInit(gyro);
#endif
}

static void lsm6dsoSpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool lsm6dsoSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != LSM6DSO_SPI) {
        return false;
    }

    acc->initFn = lsm6dsoSpiAccInit;
    acc->readFn = lsm6dsoAccRead;

    return true;
}

bool lsm6dsoSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != LSM6DSO_SPI) {
        return false;
    }

    gyro->initFn = lsm6dsoSpiGyroInit;
    gyro->readFn = lsm6dsoGyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif // USE_ACCGYRO_LSM6DSO
