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

#if defined(USE_ACC_MPU6050) || defined(USE_GYRO_MPU6050)

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus_i2c.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6050.h"

//#define DEBUG_MPU_DATA_READY_INTERRUPT

// MPU6050, Standard address 0x68
// MPU_INT on PB13 on rev4 Naze32 hardware
#define MPU6050_ADDRESS         0x68

#define DMP_MEM_START_ADDR 0x6E
#define DMP_MEM_R_W 0x6F

#define MPU6050_SMPLRT_DIV      0       // 8000Hz

static void mpu6050AccInit(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.resolution) {
        case MPU_HALF_RESOLUTION:
            acc->acc_1G = 256 * 4;
            break;
        case MPU_FULL_RESOLUTION:
            acc->acc_1G = 512 * 4;
            break;
    }
}

bool mpu6050AccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }

    acc->initFn = mpu6050AccInit;
    acc->readFn = mpuAccRead;
    acc->revisionCode = (acc->mpuDetectionResult.resolution == MPU_HALF_RESOLUTION ? 'o' : 'n'); // es/non-es variance between MPU6050 sensors, half of the naze boards are mpu6000ES.

    return true;
}

static void mpu6050GyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    busWriteRegister(&gyro->dev, MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(100);
    busWriteRegister(&gyro->dev, MPU_RA_PWR_MGMT_1, 0x03); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    busWriteRegister(&gyro->dev, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops); //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    delay(15); //PLL Settling time when changing CLKSEL is max 10ms.  Use 15ms to be sure
    busWriteRegister(&gyro->dev, MPU_RA_CONFIG, mpuGyroDLPF(gyro)); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    busWriteRegister(&gyro->dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);   //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

    // ACC Init stuff.
    // Accel scale 8g (4096 LSB/g)
    busWriteRegister(&gyro->dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);

    busWriteRegister(&gyro->dev, MPU_RA_INT_PIN_CFG,
            0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0); // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS

#ifdef USE_MPU_DATA_READY_SIGNAL
    busWriteRegister(&gyro->dev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
#endif
}

bool mpu6050GyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }
    gyro->initFn = mpu6050GyroInit;
    gyro->readFn = mpuGyroRead;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif
