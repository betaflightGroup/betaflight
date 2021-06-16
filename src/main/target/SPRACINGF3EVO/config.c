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

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"

#include "flight/pid.h"

#include "pg/sdcard.h"
#include "pg/motor.h"


#if defined(SPRACINGF3MQ)
#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz
#endif

void targetConfiguration(void)
{
    // Temporary workaround: Disable SDCard DMA by default since it causes errors on this target

#if defined(SPRACINGF3MQ)

    motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[FD_ROLL].P = 90;
        pidProfile->pid[FD_ROLL].I = 44;
        pidProfile->pid[FD_ROLL].D = 60;
        pidProfile->pid[FD_PITCH].P = 90;
        pidProfile->pid[FD_PITCH].I = 44;
        pidProfile->pid[FD_PITCH].D = 60;
    }
#endif
}
#endif
