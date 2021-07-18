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
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_GYRO_SPI_ICM42688P

#include "common/axis.h"
#include "common/maths.h"
#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm42688P.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// 24 MHz max SPI frequency
#define ICM42688P_MAX_SPI_CLK_HZ 24000000

// 10 MHz max SPI frequency for intialisation
#define ICM42688P_MAX_SPI_INIT_CLK_HZ 1000000

#define ICM42688P_RA_PWR_MGMT0                       0x4E

#define ICM42688P_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM42688P_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM42688P_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM42688P_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM42688P_RA_GYRO_CONFIG0                    0x4F
#define ICM42688P_RA_ACCEL_CONFIG0                   0x50

#define ICM42688P_RA_GYRO_ACCEL_CONFIG0              0x52

#define ICM42688P_ACCEL_UI_FILT_BW_LOW_LATENCY       (14 << 4)
#define ICM42688P_GYRO_UI_FILT_BW_LOW_LATENCY        (14 << 0)

#define ICM42688P_RA_GYRO_DATA_X1                    0x25
#define ICM42688P_RA_ACCEL_DATA_X1                   0x1F

#define ICM42688P_RA_INT_CONFIG                      0x14
#define ICM42688P_INT1_MODE_PULSED                   (0 << 2)
#define ICM42688P_INT1_MODE_LATCHED                  (1 << 2)
#define ICM42688P_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM42688P_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM42688P_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM42688P_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM42688P_RA_INT_CONFIG0                     0x63
#define ICM42688P_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM42688P_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (1 << 4)) // duplicate settings in datasheet, Rev 1.3.
#define ICM42688P_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM42688P_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM42688P_RA_INT_CONFIG1                     0x64
#define ICM42688P_INT_ASYNC_RESET_BIT                4
#define ICM42688P_INT_TDEASSERT_DISABLE_BIT          5
#define ICM42688P_INT_TDEASSERT_ENABLED              (0 << ICM42688P_INT_TDEASSERT_DISABLE_BIT)
#define ICM42688P_INT_TDEASSERT_DISABLED             (1 << ICM42688P_INT_TDEASSERT_DISABLE_BIT)
#define ICM42688P_INT_TPULSE_DURATION_BIT            6
#define ICM42688P_INT_TPULSE_DURATION_100            (0 << ICM42688P_INT_TPULSE_DURATION_BIT)
#define ICM42688P_INT_TPULSE_DURATION_8              (1 << ICM42688P_INT_TPULSE_DURATION_BIT)


#define ICM42688P_RA_INT_SOURCE0                     0x65
#define ICM42688P_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM42688P_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

static void icm42688PSpiInit(const busDevice_t *bus)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }


    spiSetDivisor(bus->busdev_u.spi.instance, spiCalculateDivider(ICM42688P_MAX_SPI_CLK_HZ));

    hardwareInitialised = true;
}

uint8_t icm42688PSpiDetect(const busDevice_t *bus)
{
    icm42688PSpiInit(bus);

    spiSetDivisor(bus->busdev_u.spi.instance, spiCalculateDivider(ICM42688P_MAX_SPI_INIT_CLK_HZ));

    spiBusWriteRegister(bus, ICM42688P_RA_PWR_MGMT0, 0x00);

    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    do {
        delay(150);
        const uint8_t whoAmI = spiBusReadRegister(bus, MPU_RA_WHO_AM_I);
        switch (whoAmI) {
        case ICM42688P_WHO_AM_I_CONST:
            icmDetected = ICM_42688P_SPI;
            break;
        default:
            icmDetected = MPU_NONE;
            break;
        }
        if (icmDetected != MPU_NONE) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

    spiSetDivisor(bus->busdev_u.spi.instance, spiCalculateDivider(ICM42688P_MAX_SPI_CLK_HZ));

    return icmDetected;
}

void icm42688PAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool icm42688PAccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&acc->bus, ICM42688P_RA_ACCEL_DATA_X1, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}
bool icm42688PSpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_42688P_SPI:
        break;
    default:
        return false;
    }

    acc->initFn = icm42688PAccInit;
    acc->readFn = icm42688PAccRead;

    return true;
}

typedef struct odrEntry_s {
    uint8_t khz;
    uint8_t odr; // See GYRO_ODR in datasheet.
} odrEntry_t;

static odrEntry_t icm42688PkhzToSupportedODRMap[] = {
    { 8, 3 },
    { 4, 4 },
    { 2, 5 },
    { 1, 6 },
};

void icm42688PGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, spiCalculateDivider(ICM42688P_MAX_SPI_INIT_CLK_HZ));

    spiBusWriteRegister(&gyro->bus, ICM42688P_RA_PWR_MGMT0, ICM42688P_PWR_MGMT0_TEMP_DISABLE_OFF | ICM42688P_PWR_MGMT0_ACCEL_MODE_LN | ICM42688P_PWR_MGMT0_GYRO_MODE_LN);
    delay(15);

    uint8_t outputDataRate = 0;
    bool supportedODRFound = false;

    if (gyro->gyroRateKHz) {
        uint8_t gyroSyncDenominator = gyro->mpuDividerDrops + 1; // rebuild it in here, see gyro_sync.c
        uint8_t desiredODRKhz = 8 / gyroSyncDenominator;
        for (uint32_t i = 0; i < ARRAYLEN(icm42688PkhzToSupportedODRMap); i++) {
            if (icm42688PkhzToSupportedODRMap[i].khz == desiredODRKhz) {
                outputDataRate = icm42688PkhzToSupportedODRMap[i].odr;
                supportedODRFound = true;
                break;
            }
        }
    }

    if (!supportedODRFound) {
        outputDataRate = 6;
        gyro->gyroRateKHz = GYRO_RATE_1_kHz;
    }

    STATIC_ASSERT(INV_FSR_2000DPS == 3, "INV_FSR_2000DPS must be 3 to generate correct value");
    spiBusWriteRegister(&gyro->bus, ICM42688P_RA_GYRO_CONFIG0, (3 - INV_FSR_2000DPS) << 5 | (outputDataRate & 0x0F));
    delay(15);

    STATIC_ASSERT(INV_FSR_16G == 3, "INV_FSR_16G must be 3 to generate correct value");
    spiBusWriteRegister(&gyro->bus, ICM42688P_RA_ACCEL_CONFIG0, (3 - INV_FSR_16G) << 5 | (outputDataRate & 0x0F));
    delay(15);

    spiBusWriteRegister(&gyro->bus, ICM42688P_RA_GYRO_ACCEL_CONFIG0, ICM42688P_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM42688P_GYRO_UI_FILT_BW_LOW_LATENCY);

    spiBusWriteRegister(&gyro->bus, ICM42688P_RA_INT_CONFIG, ICM42688P_INT1_MODE_PULSED | ICM42688P_INT1_DRIVE_CIRCUIT_PP | ICM42688P_INT1_POLARITY_ACTIVE_HIGH);
    spiBusWriteRegister(&gyro->bus, ICM42688P_RA_INT_CONFIG0, ICM42688P_UI_DRDY_INT_CLEAR_ON_SBR);

#ifdef USE_MPU_DATA_READY_SIGNAL
    spiBusWriteRegister(&gyro->bus, ICM42688P_RA_INT_SOURCE0, ICM42688P_UI_DRDY_INT1_EN_ENABLED);

    uint8_t intConfig1Value = spiBusReadRegister(&gyro->bus, ICM42688P_RA_INT_CONFIG1);
    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    intConfig1Value &= ~(1 << ICM42688P_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM42688P_INT_TPULSE_DURATION_8 | ICM42688P_INT_TDEASSERT_DISABLED);

    spiBusWriteRegister(&gyro->bus, ICM42688P_RA_INT_CONFIG1, intConfig1Value);
#endif
    //

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, spiCalculateDivider(ICM42688P_MAX_SPI_CLK_HZ));
}

bool icm42688PGyroReadSPI(gyroDev_t *gyro)
{
    static const uint8_t dataToSend[7] = {ICM42688P_RA_GYRO_DATA_X1 | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];

    const bool ack = spiBusTransfer(&gyro->bus, dataToSend, data, 7);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);

    return true;
}

bool icm42688PSpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_42688P_SPI:
        break;
    default:
        return false;
    }

    gyro->initFn = icm42688PGyroInit;
    gyro->readFn = icm42688PGyroReadSPI;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif
