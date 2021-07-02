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
#include <stdlib.h>

#include "platform.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"


PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);

PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
    .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE | FEATURE_DYNAMIC_FILTER | FEATURE_ANTI_GRAVITY | FEATURE_AIRMODE,
);

static uint32_t runtimeFeatureMask;

void featureInit(void)
{
    runtimeFeatureMask = featureConfig()->enabledFeatures;
}

static void featureSet(const uint32_t mask, uint32_t *features)
{
    *features |= mask;
}

static void featureClear(const uint32_t mask, uint32_t *features)
{
    *features &= ~(mask);
}

// Determines if the feature is enabled (active) in the runtime state.
// This is the primary funciton used by code that wants to know if a
// feature is available.
bool featureIsEnabled(const uint32_t mask)
{
    return runtimeFeatureMask & mask;
}

// Determines if the feature is configured (set in the configuration). Doesn't mean the
// feature is active in the runtime. This function *SHOULD ONLY* be used in the config check
// performed at startup and when writing to EEPROM.
bool featureIsConfigured(const uint32_t mask)
{
    return featureConfig()->enabledFeatures & mask;
}

// Updates the configuration *AND* runtime state of a feature.
// Used *ONLY* by the config check process that runs at startup and EEPROM save.
void featureEnableImmediate(const uint32_t mask)
{
    featureSet(mask, &featureConfigMutable()->enabledFeatures);
    featureSet(mask, &runtimeFeatureMask);
}

// Updates the configuration *AND* runtime state of a feature.
// Used *ONLY* by the config check process that runs at startup and EEPROM save.
void featureDisableImmediate(const uint32_t mask)
{
    featureClear(mask, &featureConfigMutable()->enabledFeatures);
    featureClear(mask, &runtimeFeatureMask);
}

// Updates *ONLY* the runtime state of a feature and not the configuration
// Used *ONLY* by the config check process that runs at startup.
void featureDisableRuntime(const uint32_t mask)
{
    featureClear(mask, &runtimeFeatureMask);
}

// Sets the configuration state of the feature and *DOES NOT* change the runtime state.
// For example, used by the CLI "feature" command. Use this function if you want to
// enable a feature that will be active after save/reboot.
void featureConfigSet(const uint32_t mask)
{
    featureSet(mask, &featureConfigMutable()->enabledFeatures);
}

// Sets the configuration state of the feature and *DOES NOT* change the runtime state.
// For example, used by the CLI "feature" command. Use this function if you want to
// disable a feature after a save/reboot.
void featureConfigClear(const uint32_t mask)
{
    featureClear(mask, &featureConfigMutable()->enabledFeatures);
}

// Sets the configuration state of all features and *DOES NOT* change the runtime state.
// For example, used by MSP to update all the configured features in one go.
void featureConfigReplace(const uint32_t mask)
{
    featureConfigMutable()->enabledFeatures = mask;
}
