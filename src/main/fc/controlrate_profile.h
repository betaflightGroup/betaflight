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

#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    RATES_TYPE_BETAFLIGHT = 0,
    RATES_TYPE_RACEFLIGHT,
    RATES_TYPE_KISS,
    RATES_TYPE_ACTUAL,
    RATES_TYPE_QUICK,
    RATES_TYPE_COUNT    // must be the final entry
} ratesType_e;

typedef struct ratesSettingsTypes_s {
    uint8_t rc_rate;
    uint8_t srate;
    uint8_t expo;
} ratesSettingsTypes_t;

typedef enum {
    THROTTLE_LIMIT_TYPE_OFF = 0,
    THROTTLE_LIMIT_TYPE_SCALE,
    THROTTLE_LIMIT_TYPE_CLIP,
    THROTTLE_LIMIT_TYPE_COUNT   // must be the last entry
} throttleLimitType_e;

typedef enum {
    TPA_MODE_PD,
    TPA_MODE_D
} tpaMode_e;

#define MAX_RATE_PROFILE_NAME_LENGTH 8u

typedef struct controlRateConfig_s {
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates_type;
    uint8_t rcRates[3];
    uint8_t rcExpo[3];
    uint8_t rates[3];
    uint8_t dynThrPID;
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
    uint8_t throttle_limit_type;            // Sets the throttle limiting type - off, scale or clip
    uint8_t throttle_limit_percent;         // Sets the maximum pilot commanded throttle limit
    uint16_t rate_limit[3];                 // Sets the maximum rate for the axes
    uint8_t tpaMode;                        // Controls which PID terms TPA effects
    char profileName[MAX_RATE_PROFILE_NAME_LENGTH + 1]; // Descriptive name for rate profile
    uint8_t quickRatesRcExpo;               // Sets expo on rc command for quick rates
    uint8_t levelExpo[2];                   // roll/pitch level mode expo
} controlRateConfig_t;

PG_DECLARE_ARRAY(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);

extern controlRateConfig_t *currentControlRateProfile;
extern const ratesSettingsTypes_t ratesSettingLimits[RATES_TYPE_COUNT];
extern const ratesSettingsTypes_t ratesSettingDefaults[RATES_TYPE_COUNT];

void loadControlRateProfile(void);
void changeControlRateProfile(uint8_t controlRateProfileIndex);

void copyControlRateProfile(const uint8_t dstControlRateProfileIndex, const uint8_t srcControlRateProfileIndex);
