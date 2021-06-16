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
#include "drivers/exti.h"

// LSM6DSO registers (not the complete list)
typedef enum {
    LSM6DSO_REG_INT1_CTRL = 0x0D,  // int pin 1 control
    LSM6DSO_REG_INT2_CTRL = 0x0E,  // int pin 2 control
    LSM6DSO_REG_WHO_AM_I = 0x0F,   // chip ID
    LSM6DSO_REG_CTRL1_XL = 0x10,   // accelerometer control
    LSM6DSO_REG_CTRL2_G = 0x11,    // gyro control
    LSM6DSO_REG_CTRL3_C = 0x12,    // control register 3
    LSM6DSO_REG_CTRL4_C = 0x13,    // control register 4
    LSM6DSO_REG_CTRL5_C = 0x14,    // control register 5
    LSM6DSO_REG_CTRL6_C = 0x15,    // control register 6
    LSM6DSO_REG_CTRL7_G = 0x16,    // control register 7
    LSM6DSO_REG_CTRL8_XL = 0x17,   // control register 8
    LSM6DSO_REG_CTRL9_XL = 0x18,   // control register 9
    LSM6DSO_REG_CTRL10_C = 0x19,   // control register 10
    LSM6DSO_REG_STATUS = 0x1E,     // status register
    LSM6DSO_REG_OUT_TEMP_L = 0x20, // temperature LSB
    LSM6DSO_REG_OUT_TEMP_H = 0x21, // temperature MSB
    LSM6DSO_REG_OUTX_L_G = 0x22,   // gyro X axis LSB
    LSM6DSO_REG_OUTX_H_G = 0x23,   // gyro X axis MSB
    LSM6DSO_REG_OUTY_L_G = 0x24,   // gyro Y axis LSB
    LSM6DSO_REG_OUTY_H_G = 0x25,   // gyro Y axis MSB
    LSM6DSO_REG_OUTZ_L_G = 0x26,   // gyro Z axis LSB
    LSM6DSO_REG_OUTZ_H_G = 0x27,   // gyro Z axis MSB
    LSM6DSO_REG_OUTX_L_A = 0x28,   // acc X axis LSB
    LSM6DSO_REG_OUTX_H_A = 0x29,   // acc X axis MSB
    LSM6DSO_REG_OUTY_L_A = 0x2A,   // acc Y axis LSB
    LSM6DSO_REG_OUTY_H_A = 0x2B,   // acc Y axis MSB
    LSM6DSO_REG_OUTZ_L_A = 0x2C,   // acc Z axis LSB
    LSM6DSO_REG_OUTZ_H_A = 0x2D,   // acc Z axis MSB
} lsm6dsoRegister_e;

// Contained in accgyro_spi_lsm6dso_init.c which is size-optimized
uint8_t lsm6dsoDetect(const extDevice_t *dev);
bool lsm6dsoSpiAccDetect(accDev_t *acc);
bool lsm6dsoSpiGyroDetect(gyroDev_t *gyro);

// Contained in accgyro_spi_lsm6dso.c which is speed-optimized
void lsm6dsoExtiHandler(extiCallbackRec_t *cb);
bool lsm6dsoAccRead(accDev_t *acc);
bool lsm6dsoGyroRead(gyroDev_t *gyro);
