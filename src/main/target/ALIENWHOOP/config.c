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




         \   |   _ _| __|  \ |\ \      /|  |  _ \  _ \ _ \
        _ \  |     |  _|  .  | \ \ \  / __ | (   |(   |__/
      _/  _\____|___|___|_|\_|  \_/\_/ _| _|\___/\___/_|


              Take me to your leader-board...



*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "blackbox/blackbox.h"
#include "fc/rc_modes.h"
#include "common/axis.h"
#include "common/filter.h"
#include "config/feature.h"
#include "drivers/pwm_esc_detect.h"
#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "io/beeper.h"
#include "io/serial.h"
#include "pg/rx.h"
#include "pg/motor.h"
#include "rx/rx.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 32000


void targetConfiguration(void)
{
    if (getDetectedMotorType() == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1050; // for 6mm and 7mm brushed
    }

    /* Default to Spektrum */
    rxConfigMutable()->serialrx_provider = SERIALRX_SPEKTRUM2048; // all DSM* except DSM2 22ms
    rxConfigMutable()->spektrum_sat_bind = 5; // DSM2 11ms
    rxConfigMutable()->spektrum_sat_bind_autoreset = 1;
    rxConfigMutable()->mincheck = 1025;
    parseRcChannels("TAER1234", rxConfigMutable());

    mixerConfigMutable()->yaw_motors_reversed = true;
    imuConfigMutable()->small_angle = 180;

    blackboxConfigMutable()->sample_rate = 1; // sample_rate is half of PID loop frequency

    /* Breadboard-specific settings for development purposes only
     */
#if defined(BREADBOARD)
    boardAlignmentMutable()->pitchDegrees = 90; // vertical breakout board
    barometerConfigMutable()->baro_hardware = BARO_DEFAULT; // still testing not on V1 or V2 pcb
#endif

    compassConfigMutable()->mag_hardware =  MAG_NONE;

    systemConfigMutable()->cpu_overclock = 2; //216MHZ

    pidConfigMutable()->runaway_takeoff_prevention = false;

    featureConfigSet((FEATURE_DYNAMIC_FILTER | FEATURE_AIRMODE | FEATURE_ANTI_GRAVITY) ^ FEATURE_RX_PARALLEL_PWM);

    /* AlienWhoop PIDs tested with 6mm and 7mm motors on most frames */
    for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

    pidProfile->pidSumLimit = 1000;
    pidProfile->pidSumLimitYaw = 1000;

        /* AlienWhoop PIDs tested with 6mm and 7mm motors on most frames */
        pidProfile->pid[PID_PITCH].P = 115;
        pidProfile->pid[PID_PITCH].I = 75;
        pidProfile->pid[PID_PITCH].D = 95;
        pidProfile->pid[PID_ROLL].P = 110;
        pidProfile->pid[PID_ROLL].I = 75;
        pidProfile->pid[PID_ROLL].D = 85;
        pidProfile->pid[PID_YAW].P = 220;
        pidProfile->pid[PID_YAW].I = 75;
        pidProfile->pid[PID_YAW].D = 20;
        pidProfile->pid[PID_LEVEL].P = 65;
        pidProfile->pid[PID_LEVEL].I = 65;
        pidProfile->pid[PID_LEVEL].D = 55;

        /* Setpoints */
        pidProfile->dterm_filter_type = FILTER_BIQUAD;
        pidProfile->dterm_notch_hz = 0;
        pidProfile->pid[PID_PITCH].F = 100;
        pidProfile->pid[PID_ROLL].F = 100;
        pidProfile->feedforwardTransition = 0;

    /* Anti-Gravity */
    pidProfile->itermThrottleThreshold = 500;
    pidProfile->itermAcceleratorGain = 5000;

    pidProfile->levelAngleLimit = 65;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        /* RC Rates */
        controlRateConfig->rcRates[FD_ROLL] = 218;
        controlRateConfig->rcRates[FD_PITCH] = 218;
        controlRateConfig->rcRates[FD_YAW] = 218;

    /* Classic Expo */
        controlRateConfig->rcExpo[FD_ROLL] = 45;
        controlRateConfig->rcExpo[FD_PITCH] = 45;
        controlRateConfig->rcExpo[FD_YAW] = 45;

        /* Super Expo Rates */
        controlRateConfig->rates[FD_ROLL] = 0;
        controlRateConfig->rates[FD_PITCH] = 0;
        controlRateConfig->rates[FD_YAW] = 0;

        /* Throttle PID Attenuation (TPA) */
        controlRateConfig->dynThrPID = 0; // tpa_rate off
        controlRateConfig->tpa_breakpoint = 1600;

    /* Force the clipping mixer at 100% seems better for brushed than default (off) and scaling)? */
        controlRateConfig->throttle_limit_type = THROTTLE_LIMIT_TYPE_CLIP;
        //controlRateConfig->throttle_limit_percent = 100;

        controlRateConfig->thrExpo8 = 20; // 20% throttle expo
    }
}
#endif
