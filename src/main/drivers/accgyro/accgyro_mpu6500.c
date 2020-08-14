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
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/exti.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"

void mpu6500AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool mpu6500AccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    acc->initFn = mpu6500AccInit;
    acc->readFn = mpuAccRead;

    return true;
}

void mpu6500GyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    int gyro_range = INV_FSR_2000DPS;
    int accel_range = INV_FSR_16G;

    if (gyro->mpuDetectionResult.sensor == ICM_20601_SPI) {
        gyro_range = gyro->gyro_high_fsr ? ICM_HIGH_RANGE_FSR_4000DPS : ICM_HIGH_RANGE_FSR_2000DPS;
        accel_range = ICM_HIGH_RANGE_FSR_16G;
    }

    busWriteRegister(&gyro->dev, MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    delay(100);
    busWriteRegister(&gyro->dev, MPU_RA_SIGNAL_PATH_RESET, 0x07);
    delay(100);
    busWriteRegister(&gyro->dev, MPU_RA_PWR_MGMT_1, 0);
    delay(100);
    busWriteRegister(&gyro->dev, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    busWriteRegister(&gyro->dev, MPU_RA_GYRO_CONFIG, gyro_range << 3);
    delay(15);
    busWriteRegister(&gyro->dev, MPU_RA_ACCEL_CONFIG, accel_range << 3);
    delay(15);
    busWriteRegister(&gyro->dev, MPU_RA_CONFIG, mpuGyroDLPF(gyro));
    delay(15);
    busWriteRegister(&gyro->dev, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops); // Get Divider Drops
    delay(100);

    // Data ready interrupt configuration
#ifdef USE_MPU9250_MAG
    busWriteRegister(&gyro->dev, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR | MPU6500_BIT_BYPASS_EN);  // INT_ANYRD_2CLEAR, BYPASS_EN
#else
    busWriteRegister(&gyro->dev, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR);  // INT_ANYRD_2CLEAR
#endif
    delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    busWriteRegister(&gyro->dev, MPU_RA_INT_ENABLE, MPU6500_BIT_RAW_RDY_EN); // RAW_RDY_EN interrupt enable
#endif
    delay(15);
}

bool mpu6500GyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    gyro->initFn = mpu6500GyroInit;
    gyro->readFn = mpuGyroRead;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
