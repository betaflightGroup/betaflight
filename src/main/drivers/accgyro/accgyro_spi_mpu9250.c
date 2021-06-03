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
 * Kalyn Doerr (RS2K) - Robust rewrite
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/system.h"


// 1 MHz max SPI frequency for initialisation
#define MPU9250_MAX_SPI_INIT_CLK_HZ 1000000
// 20 MHz max SPI frequency
#define MPU9250_MAX_SPI_CLK_HZ 20000000

static void mpu9250AccAndGyroInit(gyroDev_t *gyro);


bool mpu9250SpiWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    delayMicroseconds(1);
    spiWriteRegBuf(dev, reg, &data, sizeof (data));
    delayMicroseconds(1);

    return true;
}

static bool mpu9250SpiSlowReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    delayMicroseconds(1);
    spiReadRegBuf(dev, reg | 0x80, data, length);
    delayMicroseconds(1);

    return true;
}

void mpu9250SpiGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    mpu9250AccAndGyroInit(gyro);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU9250_MAX_SPI_CLK_HZ)); //high speed now that we don't need to write to the slow registers

    mpuGyroRead(gyro);

    if ((((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1)) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

void mpu9250SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool mpu9250SpiWriteRegisterVerify(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    mpu9250SpiWriteRegister(dev, reg, data);
    delayMicroseconds(100);

    uint8_t attemptsRemaining = 20;
    do {
        uint8_t in;
        mpu9250SpiSlowReadRegisterBuffer(dev, reg, &in, 1);
        if (in == data) {
            return true;
        } else {
            debug[3]++;
            mpu9250SpiWriteRegister(dev, reg, data);
            delayMicroseconds(100);
        }
    } while (attemptsRemaining--);
    return false;
}

static void mpu9250AccAndGyroInit(gyroDev_t *gyro) {

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU9250_MAX_SPI_INIT_CLK_HZ)); //low speed for writing to slow registers

    mpu9250SpiWriteRegister(&gyro->dev, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delay(50);

    mpu9250SpiWriteRegisterVerify(&gyro->dev, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);

    mpu9250SpiWriteRegisterVerify(&gyro->dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);

    mpu9250SpiWriteRegisterVerify(&gyro->dev, MPU_RA_CONFIG, mpuGyroDLPF(gyro));

    mpu9250SpiWriteRegisterVerify(&gyro->dev, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);

    mpu9250SpiWriteRegisterVerify(&gyro->dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    mpu9250SpiWriteRegisterVerify(&gyro->dev, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN

#if defined(USE_MPU_DATA_READY_SIGNAL)
    mpu9250SpiWriteRegisterVerify(&gyro->dev, MPU_RA_INT_ENABLE, 0x01); //this resets register MPU_RA_PWR_MGMT_1 and won't read back correctly.
#endif

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU9250_MAX_SPI_CLK_HZ));
}

uint8_t mpu9250SpiDetect(const extDevice_t *dev)
{

    spiSetClkDivisor(dev, spiCalculateDivider(MPU9250_MAX_SPI_INIT_CLK_HZ)); //low speed
    mpu9250SpiWriteRegister(dev, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);

    uint8_t attemptsRemaining = 20;
    do {
        delay(150);
        const uint8_t in = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);
        if (in == MPU9250_WHO_AM_I_CONST || in == MPU9255_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

    spiSetClkDivisor(dev, spiCalculateDivider(MPU9250_MAX_SPI_CLK_HZ));

    return MPU_9250_SPI;
}

bool mpu9250SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }

    acc->initFn = mpu9250SpiAccInit;
    acc->readFn = mpuAccReadSPI;

    return true;
}

bool mpu9250SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }

    gyro->initFn = mpu9250SpiGyroInit;
    gyro->readFn = mpuGyroReadSPI;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
