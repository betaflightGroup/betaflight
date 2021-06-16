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

#define mpu9250_CONFIG                      0x1A

/* We should probably use these. :)
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
*/

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

#define MPU9250_BIT_RESET                   (0x80)

// RF = Register Flag
#define MPU_RF_DATA_RDY_EN (1 << 0)

void mpu9250SpiResetGyro(void);

uint8_t mpu9250SpiDetect(const extDevice_t *dev);

bool mpu9250SpiAccDetect(accDev_t *acc);
bool mpu9250SpiGyroDetect(gyroDev_t *gyro);

bool mpu9250SpiWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool mpu9250SpiWriteRegisterVerify(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool mpu9250SpiReadRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t length, uint8_t *data);
