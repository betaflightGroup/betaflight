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

/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_SPI_MPU6000)

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/system.h"


static void mpu6000AccAndGyroInit(gyroDev_t *gyro);

// 1 MHz max SPI frequency for initialisation
#define MPU6000_MAX_SPI_INIT_CLK_HZ 1000000
// 20 MHz max SPI frequency
#define MPU6000_MAX_SPI_CLK_HZ 20000000

#define MPU6000_SHORT_THRESHOLD         82  // Any interrupt interval less than this will be recognised as the short interval of ~79us

// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    0x04
#define BIT_ACC                     0x02
#define BIT_TEMP                    0x01

// Product ID Description for MPU6000
// high 4 bits low 4 bits
// Product Name Product Revision
#define MPU6000ES_REV_C4 0x14
#define MPU6000ES_REV_C5 0x15
#define MPU6000ES_REV_D6 0x16
#define MPU6000ES_REV_D7 0x17
#define MPU6000ES_REV_D8 0x18
#define MPU6000_REV_C4 0x54
#define MPU6000_REV_C5 0x55
#define MPU6000_REV_D6 0x56
#define MPU6000_REV_D7 0x57
#define MPU6000_REV_D8 0x58
#define MPU6000_REV_D9 0x59
#define MPU6000_REV_D10 0x5A

void mpu6000SpiGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    mpu6000AccAndGyroInit(gyro);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_INIT_CLK_HZ));

    // Accel and Gyro DLPF Setting
    spiWriteReg(&gyro->dev, MPU6000_CONFIG, mpuGyroDLPF(gyro));
    delayMicroseconds(1);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_CLK_HZ));

    mpuGyroRead(gyro);

    if (((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

void mpu6000SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

uint8_t mpu6000SpiDetect(const extDevice_t *dev)
{

    spiSetClkDivisor(dev, spiCalculateDivider(MPU6000_MAX_SPI_INIT_CLK_HZ));

    // reset the device configuration
    spiWriteReg(dev, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(100);  // datasheet specifies a 100ms delay after reset

    // reset the device signal paths
    spiWriteReg(dev, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(100);  // datasheet specifies a 100ms delay after signal path reset


    const uint8_t whoAmI = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);
    delayMicroseconds(1); // Ensure CS high time is met which is violated on H7 without this delay
    uint8_t detectedSensor = MPU_NONE;

    if (whoAmI == MPU6000_WHO_AM_I_CONST) {
        const uint8_t productID = spiReadRegMsk(dev, MPU_RA_PRODUCT_ID);

        /* look for a product ID we recognise */

        // verify product revision
        switch (productID) {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
            detectedSensor = MPU_60x0_SPI;
        }
    }

    spiSetClkDivisor(dev, spiCalculateDivider(MPU6000_MAX_SPI_CLK_HZ));
    return detectedSensor;
}

static void mpu6000AccAndGyroInit(gyroDev_t *gyro)
{
    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_INIT_CLK_HZ));

    // Device was already reset during detection so proceed with configuration

    // Clock Source PPL with Z axis gyro reference
    spiWriteReg(&gyro->dev, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    spiWriteReg(&gyro->dev, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    spiWriteReg(&gyro->dev, MPU_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    spiWriteReg(&gyro->dev, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);
    delayMicroseconds(15);

    // Gyro +/- 2000 DPS Full Scale
    spiWriteReg(&gyro->dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    // Accel +/- 16 G Full Scale
    spiWriteReg(&gyro->dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delayMicroseconds(15);

    spiWriteReg(&gyro->dev, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    spiWriteReg(&gyro->dev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delayMicroseconds(15);
#endif

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_CLK_HZ));
    delayMicroseconds(1);
}

bool mpu6000SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    acc->initFn = mpu6000SpiAccInit;
    acc->readFn = mpuAccReadSPI;

    return true;
}

bool mpu6000SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    gyro->initFn = mpu6000SpiGyroInit;
    gyro->readFn = mpuGyroReadSPI;
    gyro->scale = GYRO_SCALE_2000DPS;
#ifdef USE_GYRO_EXTI
    gyro->gyroShortPeriod = clockMicrosToCycles(MPU6000_SHORT_THRESHOLD);
#endif
    return true;
}

#endif
