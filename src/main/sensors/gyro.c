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
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"
#include "config/simplified_tuning.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/gyrodev.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#ifdef USE_GYRO_DATA_ANALYSE
#include "flight/gyroanalyse.h"
#endif
#include "flight/rpm_filter.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#if ((TARGET_FLASH_SIZE > 128) && (defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20689) || defined(USE_GYRO_SPI_MPU6500)))
#define USE_GYRO_SLEW_LIMITER
#endif

FAST_DATA_ZERO_INIT gyro_t gyro;

static FAST_DATA_ZERO_INIT bool overflowDetected;
#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_DATA_ZERO_INIT timeUs_t overflowTimeUs;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
static FAST_DATA_ZERO_INIT bool yawSpinRecoveryEnabled;
static FAST_DATA_ZERO_INIT int yawSpinRecoveryThreshold;
static FAST_DATA_ZERO_INIT bool yawSpinDetected;
static FAST_DATA_ZERO_INIT timeUs_t yawSpinTimeUs;
#endif

static FAST_DATA_ZERO_INIT float accumulatedMeasurements[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT float gyroPrevious[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT int accumulatedMeasurementCount;

static FAST_DATA_ZERO_INIT int16_t gyroSensorTemperature;

FAST_DATA uint8_t activePidLoopDenom = 1;

static bool firstArmingCalibrationWasStarted = false;

#ifdef UNIT_TEST
STATIC_UNIT_TESTED gyroSensor_t * const gyroSensorPtr = &gyro.gyroSensor1;
STATIC_UNIT_TESTED gyroDev_t * const gyroDevPtr = &gyro.gyroSensor1.gyroDev;
#endif


#define DEBUG_GYRO_CALIBRATION 3

#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% full scale (1950dps for 2000dps gyro)
#define GYRO_OVERFLOW_RESET_THRESHOLD 30340    // 92.5% full scale (1850dps for 2000dps gyro)

PG_REGISTER_WITH_RESET_FN(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 9);

#ifndef GYRO_CONFIG_USE_GYRO_DEFAULT
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1
#endif

void pgResetFn_gyroConfig(gyroConfig_t *gyroConfig)
{
    gyroConfig->gyroCalibrationDuration = 125;        // 1.25 seconds
    gyroConfig->gyroMovementCalibrationThreshold = 48;
    gyroConfig->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
    gyroConfig->gyro_lowpass_type = FILTER_PT1;
    gyroConfig->gyro_lowpass_hz = 200;  // NOTE: dynamic lpf is enabled by default so this setting is actually
                                        // overridden and the static lowpass 1 is disabled. We can't set this
                                        // value to 0 otherwise Configurator versions 10.4 and earlier will also
                                        // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
    gyroConfig->gyro_lowpass2_type = FILTER_PT1;
    gyroConfig->gyro_lowpass2_hz = GYRO_LOWPASS_2_HZ_DEFAULT;
    gyroConfig->gyro_high_fsr = false;
    gyroConfig->gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT;
    gyroConfig->gyro_soft_notch_hz_1 = 0;
    gyroConfig->gyro_soft_notch_cutoff_1 = 0;
    gyroConfig->gyro_soft_notch_hz_2 = 0;
    gyroConfig->gyro_soft_notch_cutoff_2 = 0;
    gyroConfig->checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES;
    gyroConfig->gyro_offset_yaw = 0;
    gyroConfig->yaw_spin_recovery = YAW_SPIN_RECOVERY_AUTO;
    gyroConfig->yaw_spin_threshold = 1950;
    gyroConfig->dyn_lpf_gyro_min_hz = DYN_LPF_GYRO_MIN_HZ_DEFAULT;
    gyroConfig->dyn_lpf_gyro_max_hz = DYN_LPF_GYRO_MAX_HZ_DEFAULT;
    gyroConfig->dyn_notch_max_hz = 600;
    gyroConfig->dyn_notch_count = 1;
    gyroConfig->dyn_notch_bandwidth_hz = 45;
    gyroConfig->dyn_notch_min_hz = 150;
    gyroConfig->gyro_filter_debug_axis = FD_ROLL;
    gyroConfig->dyn_lpf_curve_expo = 5;
    gyroConfig->simplified_gyro_filter = false;
    gyroConfig->simplified_gyro_filter_multiplier = SIMPLIFIED_TUNING_DEFAULT;
}

#ifdef USE_GYRO_DATA_ANALYSE
bool isDynamicFilterActive(void)
{
    return featureIsEnabled(FEATURE_DYNAMIC_FILTER);
}
#endif

FAST_CODE bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.cyclesRemaining == 0;
}

FAST_CODE bool gyroIsCalibrationComplete(void)
{
    switch (gyro.gyroToUse) {
        default:
        case GYRO_CONFIG_USE_GYRO_1: {
            return isGyroSensorCalibrationComplete(&gyro.gyroSensor1);
        }
#ifdef USE_MULTI_GYRO
        case GYRO_CONFIG_USE_GYRO_2: {
            return isGyroSensorCalibrationComplete(&gyro.gyroSensor2);
        }
        case GYRO_CONFIG_USE_GYRO_BOTH: {
            return isGyroSensorCalibrationComplete(&gyro.gyroSensor1) && isGyroSensorCalibrationComplete(&gyro.gyroSensor2);
        }
#endif
    }
}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}

static int32_t gyroCalculateCalibratingCycles(void)
{
    return (gyroConfig()->gyroCalibrationDuration * 10000) / gyro.sampleLooptime;
}

static bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == gyroCalculateCalibratingCycles();
}

static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor)
{
#if defined(USE_FAKE_GYRO) && !defined(UNIT_TEST)
    if (gyroSensor->gyroDev.gyroHardware == GYRO_FAKE) {
        gyroSensor->calibration.cyclesRemaining = 0;
        return;
    }
#endif
    gyroSensor->calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
}

void gyroStartCalibration(bool isFirstArmingCalibration)
{
    if (isFirstArmingCalibration && firstArmingCalibrationWasStarted) {
        return;
    }

    gyroSetCalibrationCycles(&gyro.gyroSensor1);
#ifdef USE_MULTI_GYRO
    gyroSetCalibrationCycles(&gyro.gyroSensor2);
#endif

    if (isFirstArmingCalibration) {
        firstArmingCalibrationWasStarted = true;
    }
}

bool isFirstArmingGyroCalibrationRunning(void)
{
    return firstArmingCalibrationWasStarted && !gyroIsCalibrationComplete();
}

STATIC_UNIT_TESTED void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            if (axis == X) {
                DEBUG_SET(DEBUG_GYRO_RAW, DEBUG_GYRO_CALIBRATION, lrintf(stddev));
            }

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

            // please take care with exotic boardalignment !!
            gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              gyroSensor->gyroDev.gyroZero[axis] -= ((float)gyroConfig()->gyro_offset_yaw / 100);
            }
        }
    }

    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
        schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
            beeper(BEEPER_GYRO_CALIBRATED);
        }
    }

    --gyroSensor->calibration.cyclesRemaining;
}

#if defined(USE_GYRO_SLEW_LIMITER)
FAST_CODE int32_t gyroSlewLimiter(gyroSensor_t *gyroSensor, int axis)
{
    int32_t ret = (int32_t)gyroSensor->gyroDev.gyroADCRaw[axis];
    if (gyroConfig()->checkOverflow || gyro.gyroHasOverflowProtection) {
        // don't use the slew limiter if overflow checking is on or gyro is not subject to overflow bug
        return ret;
    }
    if (abs(ret - gyroSensor->gyroDev.gyroADCRawPrevious[axis]) > (1<<14)) {
        // there has been a large change in value, so assume overflow has occurred and return the previous value
        ret = gyroSensor->gyroDev.gyroADCRawPrevious[axis];
    } else {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = ret;
    }
    return ret;
}
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_CODE_NOINLINE void handleOverflow(timeUs_t currentTimeUs)
{
    // This will need to be revised if we ever allow different sensor types to be
    // used simultaneously. In that case the scale might be different between sensors.
    // It's complicated by the fact that we're using filtered gyro data here which is
    // after both sensors are scaled and averaged.
    const float gyroOverflowResetRate = GYRO_OVERFLOW_RESET_THRESHOLD * gyro.scale;

    if ((fabsf(gyro.gyroADCf[X]) < gyroOverflowResetRate)
          && (fabsf(gyro.gyroADCf[Y]) < gyroOverflowResetRate)
          && (fabsf(gyro.gyroADCf[Z]) < gyroOverflowResetRate)) {
        // if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
        // reset requires good OK values on all axes
        if (cmpTimeUs(currentTimeUs, overflowTimeUs) > 50000) {
            overflowDetected = false;
        }
    } else {
        // not a consecutive OK value, so reset the overflow time
        overflowTimeUs = currentTimeUs;
    }
}

static FAST_CODE void checkForOverflow(timeUs_t currentTimeUs)
{
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (overflowDetected) {
        handleOverflow(currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for overflow in the axes set in overflowAxisMask
        gyroOverflow_e overflowCheck = GYRO_OVERFLOW_NONE;

        // This will need to be revised if we ever allow different sensor types to be
        // used simultaneously. In that case the scale might be different between sensors.
        // It's complicated by the fact that we're using filtered gyro data here which is
        // after both sensors are scaled and averaged.
        const float gyroOverflowTriggerRate = GYRO_OVERFLOW_TRIGGER_THRESHOLD * gyro.scale;

        if (fabsf(gyro.gyroADCf[X]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_X;
        }
        if (fabsf(gyro.gyroADCf[Y]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Y;
        }
        if (fabsf(gyro.gyroADCf[Z]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Z;
        }
        if (overflowCheck & gyro.overflowAxisMask) {
            overflowDetected = true;
            overflowTimeUs = currentTimeUs;
#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = false;
#endif // USE_YAW_SPIN_RECOVERY
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_GYRO_OVERFLOW_CHECK

#ifdef USE_YAW_SPIN_RECOVERY
static FAST_CODE_NOINLINE void handleYawSpin(timeUs_t currentTimeUs)
{
    const float yawSpinResetRate = yawSpinRecoveryThreshold - 100.0f;
    if (fabsf(gyro.gyroADCf[Z]) < yawSpinResetRate) {
        // testing whether 20ms of consecutive OK gyro yaw values is enough
        if (cmpTimeUs(currentTimeUs, yawSpinTimeUs) > 20000) {
            yawSpinDetected = false;
        }
    } else {
        // reset the yaw spin time
        yawSpinTimeUs = currentTimeUs;
    }
}

static FAST_CODE void checkForYawSpin(timeUs_t currentTimeUs)
{
    // if not in overflow mode, handle yaw spins above threshold
#ifdef USE_GYRO_OVERFLOW_CHECK
    if (overflowDetected) {
        yawSpinDetected = false;
        return;
    }
#endif // USE_GYRO_OVERFLOW_CHECK

    if (yawSpinDetected) {
        handleYawSpin(currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for spin on yaw axis only
         if (abs((int)gyro.gyroADCf[Z]) > yawSpinRecoveryThreshold) {
            yawSpinDetected = true;
            yawSpinTimeUs = currentTimeUs;
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_YAW_SPIN_RECOVERY

static FAST_CODE FAST_CODE_NOINLINE void gyroUpdateSensor(gyroSensor_t *gyroSensor)
{
    if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
    gyroSensor->gyroDev.dataReady = false;

    if (isGyroSensorCalibrationComplete(gyroSensor)) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations

#if defined(USE_GYRO_SLEW_LIMITER)
        gyroSensor->gyroDev.gyroADC[X] = gyroSlewLimiter(gyroSensor, X) - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSlewLimiter(gyroSensor, Y) - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSlewLimiter(gyroSensor, Z) - gyroSensor->gyroDev.gyroZero[Z];
#else
        gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];
#endif

        if (gyroSensor->gyroDev.gyroAlign == ALIGN_CUSTOM) {
            alignSensorViaMatrix(gyroSensor->gyroDev.gyroADC, &gyroSensor->gyroDev.rotationMatrix);
        } else {
            alignSensorViaRotation(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
        }
    } else {
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
    }
}

FAST_CODE void gyroUpdate(void)
{
    switch (gyro.gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroUpdateSensor(&gyro.gyroSensor1);
        if (isGyroSensorCalibrationComplete(&gyro.gyroSensor1)) {
            gyro.gyroADC[X] = gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale;
            gyro.gyroADC[Y] = gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale;
            gyro.gyroADC[Z] = gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale;
        }
        break;
#ifdef USE_MULTI_GYRO
    case GYRO_CONFIG_USE_GYRO_2:
        gyroUpdateSensor(&gyro.gyroSensor2);
        if (isGyroSensorCalibrationComplete(&gyro.gyroSensor2)) {
            gyro.gyroADC[X] = gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale;
            gyro.gyroADC[Y] = gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale;
            gyro.gyroADC[Z] = gyro.gyroSensor2.gyroDev.gyroADC[Z] * gyro.gyroSensor2.gyroDev.scale;
        }
        break;
    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroUpdateSensor(&gyro.gyroSensor1);
        gyroUpdateSensor(&gyro.gyroSensor2);
        if (isGyroSensorCalibrationComplete(&gyro.gyroSensor1) && isGyroSensorCalibrationComplete(&gyro.gyroSensor2)) {
            gyro.gyroADC[X] = ((gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale) + (gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale)) / 2.0f;
            gyro.gyroADC[Y] = ((gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale) + (gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale)) / 2.0f;
            gyro.gyroADC[Z] = ((gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale) + (gyro.gyroSensor2.gyroDev.gyroADC[Z] * gyro.gyroSensor2.gyroDev.scale)) / 2.0f;
        }
        break;
#endif
    }

    if (gyro.downsampleFilterEnabled) {
        // using gyro lowpass 2 filter for downsampling
        gyro.sampleSum[X] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[X], gyro.gyroADC[X]);
        gyro.sampleSum[Y] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[Y], gyro.gyroADC[Y]);
        gyro.sampleSum[Z] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[Z], gyro.gyroADC[Z]);
    } else {
        // using simple averaging for downsampling
        gyro.sampleSum[X] += gyro.gyroADC[X];
        gyro.sampleSum[Y] += gyro.gyroADC[Y];
        gyro.sampleSum[Z] += gyro.gyroADC[Z];
        gyro.sampleCount++;
    }
}

#define GYRO_FILTER_FUNCTION_NAME filterGyro
#define GYRO_FILTER_DEBUG_SET(mode, index, value) do { UNUSED(mode); UNUSED(index); UNUSED(value); } while (0)
#define GYRO_FILTER_AXIS_DEBUG_SET(axis, mode, index, value) do { UNUSED(axis); UNUSED(mode); UNUSED(index); UNUSED(value); } while (0)
#include "gyro_filter_impl.c"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET
#undef GYRO_FILTER_AXIS_DEBUG_SET

#define GYRO_FILTER_FUNCTION_NAME filterGyroDebug
#define GYRO_FILTER_DEBUG_SET DEBUG_SET
#define GYRO_FILTER_AXIS_DEBUG_SET(axis, mode, index, value) if (axis == (int)gyro.gyroDebugAxis) DEBUG_SET(mode, index, value)
#include "gyro_filter_impl.c"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET
#undef GYRO_FILTER_AXIS_DEBUG_SET

FAST_CODE void gyroFiltering(timeUs_t currentTimeUs)
{
    if (gyro.gyroDebugMode == DEBUG_NONE) {
        filterGyro();
    } else {
        filterGyroDebug();
    }

#ifdef USE_GYRO_DATA_ANALYSE
    if (isDynamicFilterActive()) {
        gyroDataAnalyse(&gyro.gyroAnalyseState);
    }
#endif

    if (gyro.useDualGyroDebugging) {
        switch (gyro.gyroToUse) {
        case GYRO_CONFIG_USE_GYRO_1:
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyro.gyroSensor1.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyro.gyroSensor1.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 0, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 1, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale));
            break;

#ifdef USE_MULTI_GYRO
        case GYRO_CONFIG_USE_GYRO_2:
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyro.gyroSensor2.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyro.gyroSensor2.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 2, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 3, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale));
            break;

    case GYRO_CONFIG_USE_GYRO_BOTH:
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyro.gyroSensor1.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyro.gyroSensor1.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyro.gyroSensor2.gyroDev.gyroADCRaw[X]);
            DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyro.gyroSensor2.gyroDev.gyroADCRaw[Y]);
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 0, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 1, lrintf(gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 2, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_SCALED, 3, lrintf(gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 0, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[X] * gyro.gyroSensor2.gyroDev.scale)));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 1, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[Y] * gyro.gyroSensor2.gyroDev.scale)));
            DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 2, lrintf((gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale) - (gyro.gyroSensor2.gyroDev.gyroADC[Z] * gyro.gyroSensor2.gyroDev.scale)));
            break;
#endif
        }
    }

#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow && !gyro.gyroHasOverflowProtection) {
        checkForOverflow(currentTimeUs);
    }
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    if (yawSpinRecoveryEnabled) {
        checkForYawSpin(currentTimeUs);
    }
#endif

    if (!overflowDetected) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            // integrate using trapezium rule to avoid bias
            accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyro.gyroADCf[axis]) * gyro.targetLooptime;
            gyroPrevious[axis] = gyro.gyroADCf[axis];
        }
        accumulatedMeasurementCount++;
    }

#if !defined(USE_GYRO_OVERFLOW_CHECK) && !defined(USE_YAW_SPIN_RECOVERY)
    UNUSED(currentTimeUs);
#endif
}

bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (accumulatedMeasurementCount) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        const timeUs_t accumulatedMeasurementTimeUs = accumulatedMeasurementCount * gyro.targetLooptime;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

int16_t gyroReadSensorTemperature(gyroSensor_t gyroSensor)
{
    if (gyroSensor.gyroDev.temperatureFn) {
        gyroSensor.gyroDev.temperatureFn(&gyroSensor.gyroDev, &gyroSensor.gyroDev.temperature);
    }
    return gyroSensor.gyroDev.temperature;
}

void gyroReadTemperature(void)
{
    switch (gyro.gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroSensorTemperature = gyroReadSensorTemperature(gyro.gyroSensor1);
        break;

#ifdef USE_MULTI_GYRO
    case GYRO_CONFIG_USE_GYRO_2:
        gyroSensorTemperature = gyroReadSensorTemperature(gyro.gyroSensor2);
        break;

    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroSensorTemperature = MAX(gyroReadSensorTemperature(gyro.gyroSensor1), gyroReadSensorTemperature(gyro.gyroSensor2));
        break;
#endif // USE_MULTI_GYRO
    }
}

int16_t gyroGetTemperature(void)
{
    return gyroSensorTemperature;
}

bool gyroOverflowDetected(void)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    return overflowDetected;
#else
    return false;
#endif // USE_GYRO_OVERFLOW_CHECK
}

#ifdef USE_YAW_SPIN_RECOVERY
bool gyroYawSpinDetected(void)
{
    return yawSpinDetected;
}
#endif // USE_YAW_SPIN_RECOVERY

uint16_t gyroAbsRateDps(int axis)
{
    return fabsf(gyro.gyroADCf[axis]);
}

#ifdef USE_DYN_LPF

float dynThrottle(float throttle) {
    return throttle * (1 - (throttle * throttle) / 3.0f) * 1.5f;
}

void dynLpfGyroUpdate(float throttle)
{
    if (gyro.dynLpfFilter != DYN_LPF_NONE) {
        unsigned int cutoffFreq;
        if (gyro.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, gyro.dynLpfMin, gyro.dynLpfMax, gyro.dynLpfCurveExpo);
        } else {
            cutoffFreq = fmax(dynThrottle(throttle) * gyro.dynLpfMax, gyro.dynLpfMin);
        }
        DEBUG_SET(DEBUG_DYN_LPF, 2, cutoffFreq);
        const float gyroDt = gyro.targetLooptime * 1e-6f;
        switch (gyro.dynLpfFilter) {
        case DYN_LPF_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&gyro.lowpassFilter[axis].pt1FilterState, pt1FilterGain(cutoffFreq, gyroDt));
            }
            break;
        case DYN_LPF_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&gyro.lowpassFilter[axis].biquadFilterState, cutoffFreq, gyro.targetLooptime);
            }
            break;
        case  DYN_LPF_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterUpdateCutoff(&gyro.lowpassFilter[axis].pt2FilterState, pt2FilterGain(cutoffFreq, gyroDt));
            }
            break;
        case DYN_LPF_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterUpdateCutoff(&gyro.lowpassFilter[axis].pt3FilterState, pt3FilterGain(cutoffFreq, gyroDt));
            }
            break;
        }
    }
}
#endif

#ifdef USE_YAW_SPIN_RECOVERY
void initYawSpinRecovery(int maxYawRate)
{
    bool enabledFlag;
    int threshold;

    switch (gyroConfig()->yaw_spin_recovery) {
    case YAW_SPIN_RECOVERY_ON:
        enabledFlag = true;
        threshold = gyroConfig()->yaw_spin_threshold;
        break;
    case YAW_SPIN_RECOVERY_AUTO:
        enabledFlag = true;
        const int overshootAllowance = MAX(maxYawRate / 4, 200); // Allow a 25% or minimum 200dps overshoot tolerance
        threshold = constrain(maxYawRate + overshootAllowance, YAW_SPIN_RECOVERY_THRESHOLD_MIN, YAW_SPIN_RECOVERY_THRESHOLD_MAX);
        break;
    case YAW_SPIN_RECOVERY_OFF:
    default:
        enabledFlag = false;
        threshold = YAW_SPIN_RECOVERY_THRESHOLD_MAX;
        break;
    }

    yawSpinRecoveryEnabled = enabledFlag;
    yawSpinRecoveryThreshold = threshold;
}
#endif
