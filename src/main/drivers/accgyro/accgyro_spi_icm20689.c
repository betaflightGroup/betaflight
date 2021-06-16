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

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"



// 10 MHz max SPI frequency
#define ICM20689_MAX_SPI_CLK_HZ 10000000

// 10 MHz max SPI frequency for intialisation
#define ICM20689_MAX_SPI_INIT_CLK_HZ 1000000

static void icm20689SpiInit(const extDevice_t *dev)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }


    spiSetClkDivisor(dev, spiCalculateDivider(ICM20689_MAX_SPI_CLK_HZ));

    hardwareInitialised = true;
}

uint8_t icm20689SpiDetect(const extDevice_t *dev)
{
    icm20689SpiInit(dev);

    spiSetClkDivisor(dev, spiCalculateDivider(ICM20689_MAX_SPI_INIT_CLK_HZ));

    // reset the device configuration
    spiWriteReg(dev, MPU_RA_PWR_MGMT_1, ICM20689_BIT_RESET);
    delay(100);

    // reset the device signal paths
    spiWriteReg(dev, MPU_RA_SIGNAL_PATH_RESET, 0x03);
    delay(100);

    uint8_t icmDetected;

    const uint8_t whoAmI = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);

    switch (whoAmI) {
    case ICM20601_WHO_AM_I_CONST:
        icmDetected = ICM_20601_SPI;
        break;
    case ICM20602_WHO_AM_I_CONST:
        icmDetected = ICM_20602_SPI;
        break;
    case ICM20608G_WHO_AM_I_CONST:
        icmDetected = ICM_20608_SPI;
        break;
    case ICM20689_WHO_AM_I_CONST:
        icmDetected = ICM_20689_SPI;
        break;
    default:
        icmDetected = MPU_NONE;
        break;
    }

    spiSetClkDivisor(dev, spiCalculateDivider(ICM20689_MAX_SPI_CLK_HZ));

    return icmDetected;
}

void icm20689AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool icm20689SpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_20602_SPI:
    case ICM_20689_SPI:
        break;
    default:
        return false;
    }

    acc->initFn = icm20689AccInit;
    acc->readFn = mpuAccReadSPI;

    return true;
}

void icm20689GyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(ICM20689_MAX_SPI_INIT_CLK_HZ));

    // Device was already reset during detection so proceed with configuration

    spiWriteReg(&gyro->dev, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    spiWriteReg(&gyro->dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delay(15);
    spiWriteReg(&gyro->dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delay(15);
    spiWriteReg(&gyro->dev, MPU_RA_CONFIG, mpuGyroDLPF(gyro));
    delay(15);
    spiWriteReg(&gyro->dev, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops); // Get Divider Drops
    delay(100);

    // Data ready interrupt configuration
//    spiWriteReg(&gyro->dev, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
    spiWriteReg(&gyro->dev, MPU_RA_INT_PIN_CFG, 0x10);  // INT_ANYRD_2CLEAR, BYPASS_EN

    delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    spiWriteReg(&gyro->dev, MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(ICM20689_MAX_SPI_CLK_HZ));
}

bool icm20689SpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_20601_SPI:
    case ICM_20602_SPI:
    case ICM_20608_SPI:
    case ICM_20689_SPI:
        break;
    default:
        return false;
    }

    gyro->initFn = icm20689GyroInit;
    gyro->readFn = mpuGyroReadSPI;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
