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

#include "common/time.h"
#include "pg/pg.h"
#include "drivers/accgyro/accgyro.h"
#include "sensors/sensors.h"

// Type of accelerometer used/detected
typedef enum {
    ACC_DEFAULT,
    ACC_NONE,
    ACC_ADXL345,
    ACC_MPU6050,
    ACC_MMA8452,
    ACC_BMA280,
    ACC_LSM303DLHC,
    ACC_MPU6000,
    ACC_MPU6500,
    ACC_MPU9250,
    ACC_ICM20601,
    ACC_ICM20602,
    ACC_ICM20608G,
    ACC_ICM20649,
    ACC_ICM20689,
    ACC_ICM42605,
    ACC_ICM42688P,
    ACC_BMI160,
    ACC_BMI270,
    ACC_LSM6DSO,
    ACC_FAKE
} accelerationSensor_e;

typedef struct acc_s {
    accDev_t dev;
    uint16_t sampleRateHz;
    float accADC[XYZ_AXIS_COUNT];
    bool isAccelUpdatedAtLeastOnce;
} acc_t;

extern acc_t acc;

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union rollAndPitchTrims_u {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

#if defined(USE_ACC)
typedef struct accelerometerConfig_s {
    uint16_t acc_lpf_hz;                    // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    bool acc_high_fsr;
    flightDynamicsTrims_t accZero;
    rollAndPitchTrims_t accelerometerTrims;
} accelerometerConfig_t;

PG_DECLARE(accelerometerConfig_t, accelerometerConfig);
#endif

bool accInit(uint16_t accSampleRateHz);
bool accIsCalibrationComplete(void);
bool accHasBeenCalibrated(void);
void accStartCalibration(void);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void accUpdate(timeUs_t currentTimeUs, rollAndPitchTrims_t *rollAndPitchTrims);
bool accGetAccumulationAverage(float *accumulation);
union flightDynamicsTrims_u;
void setAccelerationTrims(union flightDynamicsTrims_u *accelerationTrimsToUse);
void accInitFilters(void);
void applyAccelerometerTrimsDelta(union rollAndPitchTrims_u *rollAndPitchTrimsDelta);
