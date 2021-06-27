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

#include <math.h>
#include "platform.h"

#ifdef USE_SIMPLIFIED_TUNING

#include "common/axis.h"
#include "common/maths.h"

#include "config/simplified_tuning.h"

#include "sensors/gyro.h"

static void calculateNewPidValues(pidProfile_t *pidProfile)
{
    const pidf_t pidDefaults[FLIGHT_DYNAMICS_INDEX_COUNT] = {
            [PID_ROLL] = PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] = PID_YAW_DEFAULT,
        };

#ifdef USE_D_MIN
    const int dMinDefaults[FLIGHT_DYNAMICS_INDEX_COUNT] = D_MIN_DEFAULT;
#endif

    const float masterMultiplier = pidProfile->simplified_master_multiplier / 100.0f;
    const float feedforwardGain = pidProfile->simplified_feedforward_gain / 100.0f;
    const float pdGain = pidProfile->simplified_pd_gain / 100.0f;
    const float iGain = pidProfile->simplified_i_gain / 100.0f;
    const float pdRatio = pidProfile->simplified_pd_ratio / 100.0f;

    for (int axis = FD_ROLL; axis <= pidProfile->simplified_pids_mode; ++axis) {
        const float rpRatio = (axis == FD_PITCH) ? pidProfile->simplified_roll_pitch_ratio / 100.0f : 1.0f;
        pidProfile->pid[axis].P = constrain(pidDefaults[axis].P * masterMultiplier * pdGain * pdRatio * rpRatio, 0, PID_GAIN_MAX);
        pidProfile->pid[axis].I = constrain(pidDefaults[axis].I * masterMultiplier * iGain * rpRatio, 0, PID_GAIN_MAX);
        pidProfile->pid[axis].D = constrain(pidDefaults[axis].D * masterMultiplier * pdGain * rpRatio, 0, PID_GAIN_MAX);

#ifdef USE_D_MIN
        const float dminRatio = 1.0f + (((float)pidDefaults[axis].D / dMinDefaults[axis] - 1.0f) * (pidProfile->simplified_dmin_ratio / 100.0f - 1.0f));

        if (pidProfile->simplified_dmin_ratio == SIMPLIFIED_TUNING_MAX) {
            pidProfile->d_min[axis] = 0;
        } else {
            pidProfile->d_min[axis] = constrain(dMinDefaults[axis] * masterMultiplier * pdGain * rpRatio * dminRatio, 0, D_MIN_GAIN_MAX);
        }
#endif
        pidProfile->pid[axis].F = constrain(pidDefaults[axis].F * masterMultiplier * feedforwardGain * rpRatio, 0, F_GAIN_MAX);
    }
}

static void calculateNewDTermFilterValues(pidProfile_t *pidProfile)
{
    pidProfile->dyn_lpf_dterm_min_hz = constrain(DYN_LPF_DTERM_MIN_HZ_DEFAULT * pidProfile->simplified_dterm_filter_multiplier / 100, 0, DYN_LPF_FILTER_FREQUENCY_MAX);
    pidProfile->dyn_lpf_dterm_max_hz = constrain(DYN_LPF_DTERM_MAX_HZ_DEFAULT * pidProfile->simplified_dterm_filter_multiplier / 100, 0, DYN_LPF_FILTER_FREQUENCY_MAX);
    pidProfile->dterm_lowpass2_hz = constrain(DTERM_LOWPASS_2_HZ_DEFAULT * pidProfile->simplified_dterm_filter_multiplier / 100, 0, FILTER_FREQUENCY_MAX);
    pidProfile->dterm_filter_type = FILTER_PT1;
    pidProfile->dterm_filter2_type = FILTER_PT1;
}

static void calculateNewGyroFilterValues()
{
    gyroConfigMutable()->dyn_lpf_gyro_min_hz = constrain(DYN_LPF_GYRO_MIN_HZ_DEFAULT * gyroConfig()->simplified_gyro_filter_multiplier / 100, 0, DYN_LPF_FILTER_FREQUENCY_MAX);
    gyroConfigMutable()->dyn_lpf_gyro_max_hz = constrain(DYN_LPF_GYRO_MAX_HZ_DEFAULT * gyroConfig()->simplified_gyro_filter_multiplier / 100, 0, DYN_LPF_FILTER_FREQUENCY_MAX);
    gyroConfigMutable()->gyro_lowpass2_hz = constrain(GYRO_LOWPASS_2_HZ_DEFAULT * gyroConfig()->simplified_gyro_filter_multiplier / 100, 0, FILTER_FREQUENCY_MAX);
    gyroConfigMutable()->gyro_lowpass_type = FILTER_PT1;
    gyroConfigMutable()->gyro_lowpass2_type = FILTER_PT1;
}

void applySimplifiedTuning(pidProfile_t *pidProfile)
{
    if (pidProfile->simplified_pids_mode != PID_SIMPLIFIED_TUNING_OFF) {
        calculateNewPidValues(pidProfile);
    }

    if (pidProfile->simplified_dterm_filter) {
        calculateNewDTermFilterValues(pidProfile);
    }

    if (gyroConfig()->simplified_gyro_filter) {
        calculateNewGyroFilterValues();
    }
}
#endif // USE_SIMPLIFIED_TUNING
