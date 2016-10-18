/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/compass.h"
#include "drivers/pwm_output.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#include "hardware_revision.h"

// alternative defaults settings for AlienFlight targets
void targetConfiguration(master_t *config)
{
    config->rxConfig.spektrum_sat_bind = 5;
    config->rxConfig.spektrum_sat_bind_autoreset = 1;
    config->gyro_sync_denom = 2;
    config->mag_hardware = MAG_NONE;            // disabled by default
    config->pid_process_denom = 1;

    if (hardwareMotorType == MOTOR_BRUSHED) {
        config->motorConfig.minthrottle = 1000;
        config->motorConfig.motorPwmRate = 32000;
        config->motorConfig.motorPwmProtocol = PWM_TYPE_BRUSHED;
        config->motorConfig.useUnsyncedPwm = true;
    }

    config->profile[0].pidProfile.P8[ROLL] = 90;
    config->profile[0].pidProfile.I8[ROLL] = 44;
    config->profile[0].pidProfile.D8[ROLL] = 60;
    config->profile[0].pidProfile.P8[PITCH] = 90;
    config->profile[0].pidProfile.I8[PITCH] = 44;
    config->profile[0].pidProfile.D8[PITCH] = 60;

    *customMotorMixer(0) = (motorMixer_t){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    *customMotorMixer(1) = (motorMixer_t){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    *customMotorMixer(2) = (motorMixer_t){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    *customMotorMixer(3) = (motorMixer_t){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L
    *customMotorMixer(4) = (motorMixer_t){ 1.0f, -1.0f, -0.414178f, -1.0f };    // MIDFRONT_R
    *customMotorMixer(5) = (motorMixer_t){ 1.0f,  1.0f, -0.414178f,  1.0f };    // MIDFRONT_L
    *customMotorMixer(6) = (motorMixer_t){ 1.0f, -1.0f,  0.414178f,  1.0f };    // MIDREAR_R
    *customMotorMixer(7) = (motorMixer_t){ 1.0f,  1.0f,  0.414178f, -1.0f };    // MIDREAR_L#endif
}
