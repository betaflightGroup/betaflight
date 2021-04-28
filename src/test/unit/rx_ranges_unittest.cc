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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"
    #include "drivers/io.h"
    #include "common/maths.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "rx/rx.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {

PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);

boxBitmask_t rcModeActivationMask;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = 0;

extern float applyRxChannelRangeConfiguraton(float sample, const rxChannelRangeConfig_t *range);
}

#define RANGE_CONFIGURATION(min, max) new (rxChannelRangeConfig_t) {min, max}

TEST(RxChannelRangeTest, TestRxChannelRanges)
{
    memset(&rcModeActivationMask, 0, sizeof(rcModeActivationMask)); // BOXFAILSAFE must be OFF

    // No signal, special condition
    EXPECT_EQ(0, applyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(1000, 2000)));
    EXPECT_EQ(0, applyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(1300, 1700)));
    EXPECT_EQ(0, applyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(900, 2100)));

    // Exact mapping
    EXPECT_EQ(1000, applyRxChannelRangeConfiguraton(1000, RANGE_CONFIGURATION(1000, 2000)));
    EXPECT_EQ(1500, applyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(1000, 2000)));
    EXPECT_EQ(2000, applyRxChannelRangeConfiguraton(2000, RANGE_CONFIGURATION(1000, 2000)));
    EXPECT_EQ(750, applyRxChannelRangeConfiguraton(700, RANGE_CONFIGURATION(1000, 2000)));
    EXPECT_EQ(2250, applyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(1000, 2000)));

    // Reversed channel
    EXPECT_EQ(2000, applyRxChannelRangeConfiguraton(1000, RANGE_CONFIGURATION(2000, 1000)));
    EXPECT_EQ(1500, applyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(2000, 1000)));
    EXPECT_EQ(1000, applyRxChannelRangeConfiguraton(2000, RANGE_CONFIGURATION(2000, 1000)));

    // Shifted range
    EXPECT_EQ(1000, applyRxChannelRangeConfiguraton(900, RANGE_CONFIGURATION(900, 1900)));
    EXPECT_EQ(1500, applyRxChannelRangeConfiguraton(1400, RANGE_CONFIGURATION(900, 1900)));
    EXPECT_EQ(2000, applyRxChannelRangeConfiguraton(1900, RANGE_CONFIGURATION(900, 1900)));
    EXPECT_EQ(750, applyRxChannelRangeConfiguraton(600, RANGE_CONFIGURATION(900, 1900)));
    EXPECT_EQ(2250, applyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(900, 1900)));

    // Narrower range than expected
    EXPECT_EQ(1000, applyRxChannelRangeConfiguraton(1300, RANGE_CONFIGURATION(1300, 1700)));
    EXPECT_EQ(1500, applyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(1300, 1700)));
    EXPECT_EQ(2000, applyRxChannelRangeConfiguraton(1700, RANGE_CONFIGURATION(1300, 1700)));
    EXPECT_EQ(750, applyRxChannelRangeConfiguraton(700, RANGE_CONFIGURATION(1300, 1700)));
    EXPECT_EQ(2250, applyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(1300, 1700)));

    // Wider range than expected
    EXPECT_EQ(1000, applyRxChannelRangeConfiguraton(900, RANGE_CONFIGURATION(900, 2100)));
    EXPECT_EQ(1500, applyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(900, 2100)));
    EXPECT_EQ(2000, applyRxChannelRangeConfiguraton(2100, RANGE_CONFIGURATION(900, 2100)));
    EXPECT_EQ(750, applyRxChannelRangeConfiguraton(600, RANGE_CONFIGURATION(900, 2100)));
    EXPECT_EQ(2250, applyRxChannelRangeConfiguraton(2700, RANGE_CONFIGURATION(900, 2100)));

    // extreme out of range
    EXPECT_EQ(750, applyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(1000, 2000)));
    EXPECT_EQ(750, applyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(1300, 1700)));
    EXPECT_EQ(750, applyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(900, 2100)));

    EXPECT_EQ(2250, applyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(1000, 2000)));
    EXPECT_EQ(2250, applyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(1300, 1700)));
    EXPECT_EQ(2250, applyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(900, 2100)));
}


// stubs
extern "C" {

void failsafeOnRxSuspend(uint32_t ) {}
void failsafeOnRxResume(void) {}
bool taskUpdateRxMainInProgress() { return true; }

uint32_t micros(void) { return 0; }
uint32_t millis(void) { return 0; }

void rxPwmInit(rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxRuntimeState);
    UNUSED(callback);
}

bool sbusInit(rxConfig_t *initialRxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(initialRxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool spektrumInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool sumdInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool sumhInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool crsfRxInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool jetiExBusInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool ibusInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool xBusInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool rxMspInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeState);
    UNUSED(callback);
    return true;
}

bool featureIsEnabled(uint32_t) {
    return false;
}

void featureDisableImmediate(uint32_t) {
}

bool rxMspFrameComplete(void)
{
    return false;
}

bool isPPMDataBeingReceived(void)
{
    return false;
}

bool isPWMDataBeingReceived(void)
{
    return false;
}

void resetPPMDataReceivedState(void)
{
}

void failsafeOnValidDataReceived(void)
{
}

void failsafeOnValidDataFailed(void)
{
}

float pt1FilterGain(float f_cut, float dT)
{
    UNUSED(f_cut);
    UNUSED(dT);
    return 0.0;
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
    UNUSED(filter);
    UNUSED(k);
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    UNUSED(filter);
    UNUSED(input);
    return 0.0;
}

void pinioBoxTaskControl(void) {}

}
