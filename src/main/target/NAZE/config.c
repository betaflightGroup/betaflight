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

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/io.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/controlrate_profile.h"

#include "flight/failsafe.h"
#include "flight/pid.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "pg/beeper_dev.h"
#include "pg/flash.h"
#include "pg/motor.h"

#include "hardware_revision.h"

void targetConfiguration(void)
{
#ifdef BEEBRAIN
    // alternative defaults settings for Beebrain target
    motorConfigMutable()->dev.motorPwmRate = 4000;
    failsafeConfigMutable()->failsafe_delay = 2;
    failsafeConfigMutable()->failsafe_off_delay = 0;

    motorConfigMutable()->minthrottle = 1049;

    gyroDeviceConfigMutable()->extiTag = selectMPUIntExtiConfigByHardwareRevision();

    gyroConfigMutable()->gyro_soft_lpf_hz = 100;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;

    /*for (int channel = 0; channel < NON_AUX_CHANNEL_COUNT; channel++) {
        rxChannelRangeConfigsMutable(channel)->min = 1180;
        rxChannelRangeConfigsMutable(channel)->max = 1860;
    }*/

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[PID_ROLL].P = 60;
        pidProfile->pid[PID_ROLL].I = 70;
        pidProfile->pid[PID_ROLL].D = 17;
        pidProfile->pid[PID_PITCH].P = 80;
        pidProfile->pid[PID_PITCH].I = 90;
        pidProfile->pid[PID_PITCH].D = 18;
        pidProfile->pid[PID_YAW].P = 200;
        pidProfile->pid[PID_YAW].I = 45;
        pidProfile->pid[PID_LEVEL].P = 30;
        pidProfile->pid[PID_LEVEL].D = 30;

        pidProfile->pid[PID_PITCH].F = 200;
        pidProfile->pid[PID_ROLL].F = 200;
        pidProfile->feedforwardTransition = 50;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->rcRates[FD_ROLL] = 100;
        controlRateConfig->rcRates[FD_PITCH] = 100;
        controlRateConfig->rcRates[FD_YAW] = 110;
        controlRateConfig->rcExpo[FD_ROLL] = 0;
        controlRateConfig->rcExpo[FD_PITCH] = 0;
        controlRateConfig->rates[FD_ROLL] = 77;
        controlRateConfig->rates[FD_PITCH] = 77;
        controlRateConfig->rates[FD_YAW] = 80;
    }
#endif

#if !defined(AFROMINI) && !defined(BEEBRAIN)
    if (hardwareRevision >= NAZE32_REV5) {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        beeperDevConfigMutable()->isOpenDrain = false;
        beeperDevConfigMutable()->isInverted = true;
    } else {
        beeperDevConfigMutable()->isOpenDrain = true;
        beeperDevConfigMutable()->isInverted = false;
        flashConfigMutable()->csTag = IO_TAG_NONE;
    }
#endif

#if defined(USE_MAG) && defined(MAG_INT_EXTI)
    if (hardwareRevision < NAZE32_REV5) {
        compassConfigMutable()->interruptTag = IO_TAG(PB12);
    }
#endif
}

void targetValidateConfiguration(void)
{
    if (hardwareRevision < NAZE32_REV5 && accelerometerConfig()->acc_hardware == ACC_ADXL345) {
        accelerometerConfigMutable()->acc_hardware = ACC_NONE;
    }
}
#endif
