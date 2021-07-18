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
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_icm42605.h"
#include "drivers/accgyro/accgyro_spi_icm42688p.h"
#include "drivers/accgyro/accgyro_spi_lsm6dso.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"

#ifdef USE_ACC_ADXL345
#include "drivers/accgyro_legacy/accgyro_adxl345.h"
#endif

#ifdef USE_ACC_BMA280
#include "drivers/accgyro_legacy/accgyro_bma280.h"
#endif

#ifdef USE_ACC_LSM303DLHC
#include "drivers/accgyro_legacy/accgyro_lsm303dlhc.h"
#endif

#ifdef USE_ACC_MMA8452
#include "drivers/accgyro_legacy/accgyro_mma845x.h"
#endif

#include "drivers/bus_spi.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "pg/gyrodev.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/sensors.h"

#include "acceleration_init.h"

#define CALIBRATING_ACC_CYCLES              400

FAST_DATA_ZERO_INIT accelerationRuntime_t accelerationRuntime;

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    RESET_CONFIG_2(rollAndPitchTrims_t, rollAndPitchTrims,
        .values.roll = 0,
        .values.pitch = 0,
    );
}

static void setConfigCalibrationCompleted(void)
{
    accelerometerConfigMutable()->accZero.values.calibrationCompleted = 1;
}

bool accHasBeenCalibrated(void)
{
    return accelerometerConfig()->accZero.values.calibrationCompleted;
}

void accResetRollAndPitchTrims(void)
{
    resetRollAndPitchTrims(&accelerometerConfigMutable()->accelerometerTrims);
}

static void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero)
{
    accZero->values.roll = 0;
    accZero->values.pitch = 0;
    accZero->values.yaw = 0;
    accZero->values.calibrationCompleted = 0;
}

void pgResetFn_accelerometerConfig(accelerometerConfig_t *instance)
{
    RESET_CONFIG_2(accelerometerConfig_t, instance,
        .acc_lpf_hz = 10,
        .acc_hardware = ACC_DEFAULT,
        .acc_high_fsr = false,
    );
    resetRollAndPitchTrims(&instance->accelerometerTrims);
    resetFlightDynamicsTrims(&instance->accZero);
}

PG_REGISTER_WITH_RESET_FN(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 2);

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;


bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
    accelerationSensor_e accHardware = ACC_NONE;

#ifdef USE_ACC_ADXL345
    drv_adxl345_config_t acc_params;
#endif

retry:

    switch (accHardwareToUse) {
    case ACC_DEFAULT:
        FALLTHROUGH;

#ifdef USE_ACC_ADXL345
    case ACC_ADXL345: // ADXL345
        acc_params.useFifo = false;
        acc_params.dataRate = 800; // unused currently
        if (adxl345Detect(&acc_params, dev)) {
            accHardware = ACC_ADXL345;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_LSM303DLHC
    case ACC_LSM303DLHC:
        if (lsm303dlhcAccDetect(dev)) {
            accHardware = ACC_LSM303DLHC;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_MPU6050
    case ACC_MPU6050: // MPU6050
        if (mpu6050AccDetect(dev)) {
            accHardware = ACC_MPU6050;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_MMA8452
    case ACC_MMA8452: // MMA8452
        if (mma8452Detect(dev)) {
            accHardware = ACC_MMA8452;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_BMA280
    case ACC_BMA280: // BMA280
        if (bma280Detect(dev)) {
            accHardware = ACC_BMA280;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_MPU6000
    case ACC_MPU6000:
        if (mpu6000SpiAccDetect(dev)) {
            accHardware = ACC_MPU6000;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_MPU9250
    case ACC_MPU9250:
        if (mpu9250SpiAccDetect(dev)) {
            accHardware = ACC_MPU9250;
            break;
        }
        FALLTHROUGH;
#endif

    case ACC_MPU6500:
    case ACC_ICM20601:
    case ACC_ICM20602:
    case ACC_ICM20608G:
#if defined(USE_ACC_MPU6500) || defined(USE_ACC_SPI_MPU6500)
#ifdef USE_ACC_SPI_MPU6500
        if (mpu6500AccDetect(dev) || mpu6500SpiAccDetect(dev)) {
#else
        if (mpu6500AccDetect(dev)) {
#endif
            switch (dev->mpuDetectionResult.sensor) {
            case MPU_9250_SPI:
                accHardware = ACC_MPU9250;
                break;
            case ICM_20601_SPI:
                accHardware = ACC_ICM20601;
                break;
            case ICM_20602_SPI:
                accHardware = ACC_ICM20602;
                break;
            case ICM_20608_SPI:
                accHardware = ACC_ICM20608G;
                break;
            default:
                accHardware = ACC_MPU6500;
            }
            break;
        }
#endif
        FALLTHROUGH;

#ifdef USE_ACC_SPI_ICM20649
    case ACC_ICM20649:
        if (icm20649SpiAccDetect(dev)) {
            accHardware = ACC_ICM20649;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_ICM20689
    case ACC_ICM20689:
        if (icm20689SpiAccDetect(dev)) {
            accHardware = ACC_ICM20689;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_ICM42605
    case ACC_ICM42605:
        if (icm42605SpiAccDetect(dev)) {
            accHardware = ACC_ICM42605;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_ICM42688P
    case ACC_ICM42688P:
        if (icm42688PSpiAccDetect(dev)) {
            accHardware = ACC_ICM42688P;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI160
    case ACC_BMI160:
        if (bmi160SpiAccDetect(dev)) {
            accHardware = ACC_BMI160;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI270
    case ACC_BMI270:
        if (bmi270SpiAccDetect(dev)) {
            accHardware = ACC_BMI270;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_LSM6DSO
    case ACC_LSM6DSO:
        if (lsm6dsoSpiAccDetect(dev)) {
            accHardware = ACC_LSM6DSO;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_FAKE_ACC
    case ACC_FAKE:
        if (fakeAccDetect(dev)) {
            accHardware = ACC_FAKE;
            break;
        }
        FALLTHROUGH;
#endif

    default:
    case ACC_NONE: // disable ACC
        accHardware = ACC_NONE;
        break;
    }

    // Found anything? Check if error or ACC is really missing.
    if (accHardware == ACC_NONE && accHardwareToUse != ACC_DEFAULT && accHardwareToUse != ACC_NONE) {
        // Nothing was found and we have a forced sensor that isn't present.
        accHardwareToUse = ACC_DEFAULT;
        goto retry;
    }

    if (accHardware == ACC_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_ACC] = accHardware;
    sensorsSet(SENSOR_ACC);
    return true;
}

void accInitFilters(void)
{
    // Only set the lowpass cutoff if the ACC sample rate is detected otherwise
    // the filter initialization is not defined (sample rate = 0)
    accelerationRuntime.accLpfCutHz = (acc.sampleRateHz) ? accelerometerConfig()->acc_lpf_hz : 0;
    if (accelerationRuntime.accLpfCutHz) {
        const uint32_t accSampleTimeUs = 1e6 / acc.sampleRateHz;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accelerationRuntime.accFilter[axis], accelerationRuntime.accLpfCutHz, accSampleTimeUs);
        }
    }
}

bool accInit(uint16_t accSampleRateHz)
{
    memset(&acc, 0, sizeof(acc));
    // copy over the common gyro mpu settings
    acc.dev.bus = *gyroSensorBus();
    acc.dev.mpuDetectionResult = *gyroMpuDetectionResult();
    acc.dev.acc_high_fsr = accelerometerConfig()->acc_high_fsr;

    // Copy alignment from active gyro, as all production boards use acc-gyro-combi chip.
    // Exceptions are STM32F3DISCOVERY and STM32F411DISCOVERY, and (may be) handled in future enhancement.

    sensor_align_e alignment = gyroDeviceConfig(0)->alignment;
    const sensorAlignment_t* customAlignment = &gyroDeviceConfig(0)->customAlignment;

#ifdef USE_MULTI_GYRO
    if (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_2) {
        alignment = gyroDeviceConfig(1)->alignment;

        customAlignment = &gyroDeviceConfig(1)->customAlignment;
    }
#endif
    acc.dev.accAlign = alignment;
    buildRotationMatrixFromAlignment(customAlignment, &acc.dev.rotationMatrix);

    if (!accDetect(&acc.dev, accelerometerConfig()->acc_hardware)) {
        return false;
    }
    acc.dev.acc_1G = 256; // set default
    acc.dev.initFn(&acc.dev); // driver initialisation
    acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;

    acc.sampleRateHz = accSampleRateHz;
    accInitFilters();
    return true;
}

void accStartCalibration(void)
{
    accelerationRuntime.calibratingA = CALIBRATING_ACC_CYCLES;
}

bool accIsCalibrationComplete(void)
{
    return accelerationRuntime.calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return accelerationRuntime.calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return accelerationRuntime.calibratingA == CALIBRATING_ACC_CYCLES;
}

void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle()) {
            a[axis] = 0;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += acc.accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        acc.accADC[axis] = 0;
        accelerationRuntime.accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationRuntime.accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationRuntime.accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationRuntime.accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;

        resetRollAndPitchTrims(rollAndPitchTrims);
        setConfigCalibrationCompleted();

        saveConfigAndNotify();
    }

    accelerationRuntime.calibratingA--;
}

void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t b[3];
    static int16_t accZero_saved[3] = { 0, 0, 0 };
    static rollAndPitchTrims_t angleTrim_saved = { { 0, 0 } };

    // Saving old zeropoints before measurement
    if (InflightcalibratingA == 50) {
        accZero_saved[X] = accelerationRuntime.accelerationTrims->raw[X];
        accZero_saved[Y] = accelerationRuntime.accelerationTrims->raw[Y];
        accZero_saved[Z] = accelerationRuntime.accelerationTrims->raw[Z];
        angleTrim_saved.values.roll = rollAndPitchTrims->values.roll;
        angleTrim_saved.values.pitch = rollAndPitchTrims->values.pitch;
    }
    if (InflightcalibratingA > 0) {
        for (int axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += acc.accADC[axis];
            // Clear global variables for next reading
            acc.accADC[axis] = 0;
            accelerationRuntime.accelerationTrims->raw[axis] = 0;
        }
        // all values are measured
        if (InflightcalibratingA == 1) {
            AccInflightCalibrationActive = false;
            AccInflightCalibrationMeasurementDone = true;
            beeper(BEEPER_ACC_CALIBRATION); // indicate end of calibration
            // recover saved values to maintain current flight behaviour until new values are transferred
            accelerationRuntime.accelerationTrims->raw[X] = accZero_saved[X];
            accelerationRuntime.accelerationTrims->raw[Y] = accZero_saved[Y];
            accelerationRuntime.accelerationTrims->raw[Z] = accZero_saved[Z];
            rollAndPitchTrims->values.roll = angleTrim_saved.values.roll;
            rollAndPitchTrims->values.pitch = angleTrim_saved.values.pitch;
        }
        InflightcalibratingA--;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm) {      // the aircraft is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = false;
        accelerationRuntime.accelerationTrims->raw[X] = b[X] / 50;
        accelerationRuntime.accelerationTrims->raw[Y] = b[Y] / 50;
        accelerationRuntime.accelerationTrims->raw[Z] = b[Z] / 50 - acc.dev.acc_1G;    // for nunchuck 200=1G

        resetRollAndPitchTrims(rollAndPitchTrims);
        setConfigCalibrationCompleted();

        saveConfigAndNotify();
    }
}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationRuntime.accelerationTrims = accelerationTrimsToUse;
}

void applyAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    accelerometerConfigMutable()->accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
    accelerometerConfigMutable()->accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;
}
#endif
