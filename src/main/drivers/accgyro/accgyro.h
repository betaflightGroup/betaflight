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

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/sensor_alignment.h"
#include "drivers/exti.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro_mpu.h"

#pragma GCC diagnostic push
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <pthread.h>
#elif !defined(UNIT_TEST)
#pragma GCC diagnostic warning "-Wpadded"
#endif

#define GYRO_SCALE_2000DPS (2000.0f / (1 << 15))   // 16.384 dps/lsb scalefactor for 2000dps sensors
#define GYRO_SCALE_4000DPS (4000.0f / (1 << 15))   //  8.192 dps/lsb scalefactor for 4000dps sensors

typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_MPU9250,
    GYRO_ICM20601,
    GYRO_ICM20602,
    GYRO_ICM20608G,
    GYRO_ICM20649,
    GYRO_ICM20689,
    GYRO_ICM42605,
    GYRO_ICM42688P,
    GYRO_BMI160,
    GYRO_BMI270,
    GYRO_LSM6DSO,
    GYRO_FAKE
} gyroHardware_e;

typedef enum {
    GYRO_HARDWARE_LPF_NORMAL,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    GYRO_HARDWARE_LPF_EXPERIMENTAL
#endif
} gyroHardwareLpf_e;

typedef enum {
    GYRO_RATE_1_kHz,
    GYRO_RATE_1100_Hz,
    GYRO_RATE_3200_Hz,
    GYRO_RATE_6400_Hz,
    GYRO_RATE_6664_Hz,
    GYRO_RATE_8_kHz,
    GYRO_RATE_9_kHz,
    GYRO_RATE_32_kHz,
} gyroRateKHz_e;

typedef struct gyroDev_s {
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_t lock;
#endif
    sensorGyroInitFuncPtr initFn;                             // initialize function
    sensorGyroReadFuncPtr readFn;                             // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;                  // read temperature if available
    extiCallbackRec_t exti;
    busDevice_t bus;
    float scale;                                             // scalefactor
    float gyroZero[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];                           // gyro data after calibration and alignment
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
    int16_t temperature;
    mpuDetectionResult_t mpuDetectionResult;
    sensor_align_e gyroAlign;
    gyroRateKHz_e gyroRateKHz;
    bool dataReady;
    bool gyro_high_fsr;
    uint8_t hardware_lpf;
    uint8_t hardware_32khz_lpf;
    uint8_t mpuDividerDrops;
    ioTag_t mpuIntExtiTag;
    uint8_t gyroHasOverflowProtection;
    gyroHardware_e gyroHardware;
    fp_rotationMatrix_t rotationMatrix;
    uint16_t gyroSampleRateHz;
    uint16_t accSampleRateHz;
} gyroDev_t;

typedef struct accDev_s {
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_t lock;
#endif
    float acc_1G_rec;
    sensorAccInitFuncPtr initFn;                              // initialize function
    sensorAccReadFuncPtr readFn;                              // read 3 axis data function
    busDevice_t bus;
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    mpuDetectionResult_t mpuDetectionResult;
    sensor_align_e accAlign;
    bool dataReady;
    bool acc_high_fsr;
    char revisionCode;                                      // a revision code for the sensor, if known
    uint8_t filler[2];
    fp_rotationMatrix_t rotationMatrix;
} accDev_t;

static inline void accDevLock(accDev_t *acc)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_lock(&acc->lock);
#else
    (void)acc;
#endif
}

static inline void accDevUnLock(accDev_t *acc)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_unlock(&acc->lock);
#else
    (void)acc;
#endif
}

static inline void gyroDevLock(gyroDev_t *gyro)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_lock(&gyro->lock);
#else
    (void)gyro;
#endif
}

static inline void gyroDevUnLock(gyroDev_t *gyro)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_unlock(&gyro->lock);
#else
    (void)gyro;
#endif
}
#pragma GCC diagnostic pop
