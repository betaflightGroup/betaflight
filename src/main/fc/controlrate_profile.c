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

#include "platform.h"

#include "common/axis.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"

controlRateConfig_t *currentControlRateProfile;

PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 4);

void pgResetFn_controlRateProfiles(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &controlRateConfig[i],
            .thrMid8 = 50,
            .thrExpo8 = 0,
            .dynThrPID = 65,
            .tpa_breakpoint = 1350,
            .rates_type = RATES_TYPE_ACTUAL,
            .rcRates[FD_ROLL] = 7,
            .rcRates[FD_PITCH] = 7,
            .rcRates[FD_YAW] = 7,
            .rcExpo[FD_ROLL] = 0,
            .rcExpo[FD_PITCH] = 0,
            .rcExpo[FD_YAW] = 0,
            .rates[FD_ROLL] = 67,
            .rates[FD_PITCH] = 67,
            .rates[FD_YAW] = 67,
            .throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF,
            .throttle_limit_percent = 100,
            .rate_limit[FD_ROLL] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
            .rate_limit[FD_PITCH] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
            .rate_limit[FD_YAW] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
            .tpaMode = TPA_MODE_D,
            .profileName = { 0 },
            .quickRatesRcExpo = 0,
            .levelExpo[FD_ROLL] = 0,
            .levelExpo[FD_PITCH] = 0,
        );
    }
}

const ratesSettingsLimits_t ratesSettingLimits[RATES_TYPE_COUNT] = {
    [RATES_TYPE_BETAFLIGHT] = { 255, 100, 100 },
    [RATES_TYPE_RACEFLIGHT] = { 200, 255, 100 },
    [RATES_TYPE_KISS]       = { 255,  99, 100 },
    [RATES_TYPE_ACTUAL]     = { 200, 200, 100 },
    [RATES_TYPE_QUICK]      = { 255, 200, 100 },
};

void loadControlRateProfile(void)
{
    currentControlRateProfile = controlRateProfilesMutable(systemConfig()->activeRateProfile);
}

void changeControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = controlRateProfileIndex;
    }

    loadControlRateProfile();
    initRcProcessing();
}

void copyControlRateProfile(const uint8_t dstControlRateProfileIndex, const uint8_t srcControlRateProfileIndex) {
    if ((dstControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT && srcControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT)
        && dstControlRateProfileIndex != srcControlRateProfileIndex
    ) {
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex), controlRateProfiles(srcControlRateProfileIndex), sizeof(controlRateConfig_t));
    }
}
