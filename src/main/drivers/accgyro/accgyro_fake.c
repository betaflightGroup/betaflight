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

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>
#endif

#ifdef USE_FAKE_GYRO

#include "build/build_config.h"

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_fake.h"

static int16_t fakeGyroADC[XYZ_AXIS_COUNT];
gyroDev_t *fakeGyroDev;

static void fakeGyroInit(gyroDev_t *gyro)
{
    fakeGyroDev = gyro;
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&gyro->lock, NULL) != 0) {
        printf("Create gyro lock error!\n");
    }
#endif
}

void fakeGyroSet(gyroDev_t *gyro, int16_t x, int16_t y, int16_t z)
{
    gyroDevLock(gyro);

    fakeGyroADC[X] = x;
    fakeGyroADC[Y] = y;
    fakeGyroADC[Z] = z;

    gyro->dataReady = true;

    gyroDevUnLock(gyro);
}

STATIC_UNIT_TESTED bool fakeGyroRead(gyroDev_t *gyro)
{
    gyroDevLock(gyro);
    if (gyro->dataReady == false) {
        gyroDevUnLock(gyro);
        return false;
    }
    gyro->dataReady = false;

    gyro->gyroADCRaw[X] = fakeGyroADC[X];
    gyro->gyroADCRaw[Y] = fakeGyroADC[Y];
    gyro->gyroADCRaw[Z] = fakeGyroADC[Z];

    gyroDevUnLock(gyro);
    return true;
}

static bool fakeGyroReadTemperature(gyroDev_t *gyro, int16_t *temperatureData)
{
    UNUSED(gyro);
    UNUSED(temperatureData);
    return true;
}

bool fakeGyroDetect(gyroDev_t *gyro)
{
    gyro->initFn = fakeGyroInit;
    gyro->readFn = fakeGyroRead;
    gyro->temperatureFn = fakeGyroReadTemperature;
#if defined(SIMULATOR_BUILD)
    gyro->scale = GYRO_SCALE_2000DPS;
#else
    gyro->scale = 1.0f;
#endif
    return true;
}
#endif // USE_FAKE_GYRO


#ifdef USE_FAKE_ACC

static int16_t fakeAccData[XYZ_AXIS_COUNT];
accDev_t *fakeAccDev;

static void fakeAccInit(accDev_t *acc)
{
    fakeAccDev = acc;
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&acc->lock, NULL) != 0) {
        printf("Create acc lock error!\n");
    }
#endif
}

void fakeAccSet(accDev_t *acc, int16_t x, int16_t y, int16_t z)
{
    accDevLock(acc);

    fakeAccData[X] = x;
    fakeAccData[Y] = y;
    fakeAccData[Z] = z;

    acc->dataReady = true;

    accDevUnLock(acc);
}

static bool fakeAccRead(accDev_t *acc)
{
    accDevLock(acc);
    if (acc->dataReady == false) {
        accDevUnLock(acc);
        return false;
    }
    acc->dataReady = false;

    acc->ADCRaw[X] = fakeAccData[X];
    acc->ADCRaw[Y] = fakeAccData[Y];
    acc->ADCRaw[Z] = fakeAccData[Z];

    accDevUnLock(acc);
    return true;
}

bool fakeAccDetect(accDev_t *acc)
{
    acc->initFn = fakeAccInit;
    acc->readFn = fakeAccRead;
    acc->revisionCode = 0;
    return true;
}
#endif // USE_FAKE_ACC
