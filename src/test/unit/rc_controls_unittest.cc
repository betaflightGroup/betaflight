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

#include <limits.h>

//#define DEBUG_RC_CONTROLS

extern "C" {
    #include "platform.h"

    #include "common/maths.h"
    #include "common/axis.h"
    #include "common/bitarray.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "blackbox/blackbox.h"
    #include "blackbox/blackbox_fielddefs.h"

    #include "drivers/sensor.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "io/beeper.h"

    #include "rx/rx.h"

    #include "flight/pid.h"

    #include "config/config.h"
    #include "fc/controlrate_profile.h"
    #include "fc/rc_modes.h"
    #include "fc/rc_adjustments.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"
    #include "fc/core.h"

    #include "scheduler/scheduler.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

void unsetArmingDisabled(armingDisableFlags_e flag) {
  UNUSED(flag);
}

class RcControlsModesTest : public ::testing::Test {
protected:
    virtual void SetUp() {
    }
};

TEST_F(RcControlsModesTest, updateActivatedModesWithAllInputsAtMidde)
{
    // given
    boxBitmask_t mask;
    memset(&mask, 0, sizeof(mask));
    rcModeUpdate(&mask);

    // and
    memset(&rxRuntimeState, 0, sizeof(rxRuntimeState_t));
    rxRuntimeState.channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // when
    analyzeModeActivationConditions();
    updateActivatedModes();

    // then
    for (int index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d\n", index);
#endif
        EXPECT_FALSE(IS_RC_MODE_ACTIVE((boxId_e)index));
    }
}

TEST_F(RcControlsModesTest, updateActivatedModesUsingValidAuxConfigurationAndRXValues)
{
    // given
    modeActivationConditionsMutable(0)->modeId = (boxId_e)0;
    modeActivationConditionsMutable(0)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(1)->modeId = (boxId_e)1;
    modeActivationConditionsMutable(1)->auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(1700);

    modeActivationConditionsMutable(2)->modeId = (boxId_e)2;
    modeActivationConditionsMutable(2)->auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(1200);

    modeActivationConditionsMutable(3)->modeId = (boxId_e)3;
    modeActivationConditionsMutable(3)->auxChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(3)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(3)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(4)->modeId = (boxId_e)4;
    modeActivationConditionsMutable(4)->auxChannelIndex = AUX5 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(4)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(4)->range.endStep = CHANNEL_VALUE_TO_STEP(925);

    EXPECT_EQ(0, modeActivationConditions(4)->range.startStep);
    EXPECT_EQ(1, modeActivationConditions(4)->range.endStep);

    modeActivationConditionsMutable(5)->modeId = (boxId_e)5;
    modeActivationConditionsMutable(5)->auxChannelIndex = AUX6 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(5)->range.startStep = CHANNEL_VALUE_TO_STEP(2075);
    modeActivationConditionsMutable(5)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    EXPECT_EQ(47, modeActivationConditions(5)->range.startStep);
    EXPECT_EQ(48, modeActivationConditions(5)->range.endStep);

    modeActivationConditionsMutable(6)->modeId = (boxId_e)6;
    modeActivationConditionsMutable(6)->auxChannelIndex = AUX7 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(6)->range.startStep = CHANNEL_VALUE_TO_STEP(925);
    modeActivationConditionsMutable(6)->range.endStep = CHANNEL_VALUE_TO_STEP(950);

    EXPECT_EQ(1, modeActivationConditions(6)->range.startStep);
    EXPECT_EQ(2, modeActivationConditions(6)->range.endStep);

    // and
    boxBitmask_t mask;
    memset(&mask, 0, sizeof(mask));
    rcModeUpdate(&mask);

    // and
    memset(&rxRuntimeState, 0, sizeof(rxRuntimeState_t));
    rxRuntimeState.channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    rcData[AUX1] = PWM_RANGE_MAX;
    rcData[AUX2] = PWM_RANGE_MIDDLE;
    rcData[AUX3] = PWM_RANGE_MIN;
    rcData[AUX4] = PWM_RANGE_MAX;
    rcData[AUX5] = 899; // value lower that range minimum should be treated the same as the lowest range value
    rcData[AUX6] = 2101; // value higher than the range maximum should be treated the same as the highest range value
    rcData[AUX7] = 950; // value equal to range step upper boundary should not activate the mode

    // and
    boxBitmask_t activeBoxIds;
    memset(&activeBoxIds, 0, sizeof(boxBitmask_t));
    bitArraySet(&activeBoxIds, 0);
    bitArraySet(&activeBoxIds, 1);
    bitArraySet(&activeBoxIds, 2);
    bitArraySet(&activeBoxIds, 3);
    bitArraySet(&activeBoxIds, 4);
    bitArraySet(&activeBoxIds, 5);

    // when
    analyzeModeActivationConditions();
    updateActivatedModes();

    // then
    for (int index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d, %d\n", index, (bool)(bitArrayGet(&activeBoxIds, index)));
#endif
        EXPECT_EQ((bool)(bitArrayGet(&activeBoxIds, index)), IS_RC_MODE_ACTIVE((boxId_e)index));
    }
}

enum {
    COUNTER_QUEUE_CONFIRMATION_BEEP,
    COUNTER_CHANGE_CONTROL_RATE_PROFILE
};
#define CALL_COUNT_ITEM_COUNT 2

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

extern "C" {
void beeperConfirmationBeeps(uint8_t) {
    callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP]++;
}

void beeper(beeperMode_e mode) {
    UNUSED(mode);
}

void changeControlRateProfile(uint8_t) {
    callCounts[COUNTER_CHANGE_CONTROL_RATE_PROFILE]++;
}

}

void resetCallCounters(void) {
    memset(&callCounts, 0, sizeof(callCounts));
}

uint32_t fixedMillis;

extern "C" {
uint32_t millis(void) {
    return fixedMillis;
}

uint32_t micros(void) {
    return fixedMillis * 1000;
}
}

void resetMillis(void) {
    fixedMillis = 0;
}

#define DEFAULT_MIN_CHECK 1100
#define DEFAULT_MAX_CHECK 1900

extern "C" {
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

    extern int stepwiseAdjustmentCount;
    extern timedAdjustmentState_t stepwiseAdjustments[MAX_ADJUSTMENT_RANGE_COUNT];

    extern int continuosAdjustmentCount;
    extern continuosAdjustmentState_t continuosAdjustments[MAX_ADJUSTMENT_RANGE_COUNT];
}

class RcControlsAdjustmentsTest : public ::testing::Test {
protected:
    controlRateConfig_t controlRateConfig = {
            .rcRates[FD_ROLL] = 90,
            .rcRates[FD_PITCH] = 90,
            .rcExpo[FD_ROLL] = 0,
            .rcExpo[FD_PITCH] = 0,
            .thrMid8 = 0,
            .thrExpo8 = 0,
            .rates = {0, 0, 0},
            .dynThrPID = 0,
            .rcExpo[FD_YAW] = 0,
            .tpa_breakpoint = 0
    };

    channelRange_t fullRange = {
        .startStep = MIN_MODE_RANGE_STEP,
        .endStep = MAX_MODE_RANGE_STEP
    };

    int adjustmentRangesIndex;

    virtual void SetUp() {
        PG_RESET(rxConfig);
        rxConfigMutable()->mincheck = DEFAULT_MIN_CHECK;
        rxConfigMutable()->maxcheck = DEFAULT_MAX_CHECK;
        rxConfigMutable()->midrc = 1500;

        controlRateConfig.rcRates[FD_ROLL] = 90;
        controlRateConfig.rcRates[FD_PITCH] = 90;
        controlRateConfig.rcExpo[FD_ROLL] = 0;
        controlRateConfig.rcExpo[FD_PITCH] = 0;
        controlRateConfig.thrMid8 = 0;
        controlRateConfig.thrExpo8 = 0;
        controlRateConfig.rcExpo[FD_YAW] = 0;
        controlRateConfig.rates[0] = 0;
        controlRateConfig.rates[1] = 0;
        controlRateConfig.rates[2] = 0;
        controlRateConfig.dynThrPID = 0;
        controlRateConfig.tpa_breakpoint = 0;

        PG_RESET(adjustmentRanges);
        adjustmentRangesIndex = 0;

        stepwiseAdjustmentCount = 0;
        continuosAdjustmentCount = 0;
    }

    int configureAdjustmentRange(uint8_t switchChannelIndex, uint8_t adjustmentConfigIndex) {
        adjustmentRange_t *adjustmentRange = adjustmentRangesMutable(adjustmentRangesIndex);
        adjustmentRange->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
        adjustmentRange->range = fullRange;

        adjustmentRange->adjustmentConfig = adjustmentConfigIndex;
        adjustmentRange->auxSwitchChannelIndex = switchChannelIndex;

        return adjustmentRangesIndex++;
    }

    timedAdjustmentState_t *configureStepwiseAdjustment(uint8_t switchChannelIndex, uint8_t adjustmentConfigIndex) {
        int adjustmentRangeIndex = configureAdjustmentRange(switchChannelIndex, adjustmentConfigIndex);

        timedAdjustmentState_t *adjustmentState = &stepwiseAdjustments[stepwiseAdjustmentCount++];
        adjustmentState->adjustmentRangeIndex = adjustmentRangeIndex;
        adjustmentState->timeoutAt = 0;
        adjustmentState->ready = true;

        return adjustmentState;
    }

    void configureContinuosAdjustment(uint8_t switchChannelIndex, uint8_t adjustmentConfigIndex) {
        int adjustmentRangeIndex = configureAdjustmentRange(switchChannelIndex, adjustmentConfigIndex);

        continuosAdjustmentState_t *adjustmentState = &continuosAdjustments[continuosAdjustmentCount++];
        adjustmentState->adjustmentRangeIndex = adjustmentRangeIndex;
        adjustmentState->lastRcData = 0;
    }
};

#define ADJUSTMENT_CONFIG_RATE_INDEX 1

TEST_F(RcControlsAdjustmentsTest, processRcAdjustmentsSticksInMiddle)
{
    // given
    const timedAdjustmentState_t *adjustmentState = configureStepwiseAdjustment(AUX3 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_CONFIG_RATE_INDEX);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(90, controlRateConfig.rcRates[FD_ROLL]);
    EXPECT_EQ(90, controlRateConfig.rcRates[FD_PITCH]);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP));
    EXPECT_TRUE(adjustmentState->ready);
}

TEST_F(RcControlsAdjustmentsTest, processRcAdjustmentsWithRcRateFunctionSwitchUp)
{
    // given
    controlRateConfig_t controlRateConfig = {
            .rcRates[FD_ROLL] = 90,
            .rcRates[FD_PITCH] = 90,
            .rcExpo[FD_ROLL] = 0,
            .rcExpo[FD_PITCH] = 0,
            .thrMid8 = 0,
            .thrExpo8 = 0,
            .rates = {0,0,0},
            .dynThrPID = 0,
            .rcExpo[FD_YAW] = 0,
            .tpa_breakpoint = 0
    };

    // and
    PG_RESET(rxConfig);
    rxConfigMutable()->mincheck = DEFAULT_MIN_CHECK;
    rxConfigMutable()->maxcheck = DEFAULT_MAX_CHECK;
    rxConfigMutable()->midrc = 1500;

    // and
    const timedAdjustmentState_t *adjustmentState = configureStepwiseAdjustment(AUX3 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_CONFIG_RATE_INDEX);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // and
    rcData[AUX3] = PWM_RANGE_MAX;

    // and
    fixedMillis = 496;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(91, controlRateConfig.rcRates[FD_ROLL]);
    EXPECT_EQ(91, controlRateConfig.rcRates[FD_PITCH]);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP));
    EXPECT_FALSE(adjustmentState->ready);

    //
    // now pretend a short amount of time has passed, but not enough time to allow the value to have been increased
    //

    // given
    fixedMillis = 497;

    // when
    processRcAdjustments(&controlRateConfig);

    EXPECT_EQ(91, controlRateConfig.rcRates[FD_ROLL]);
    EXPECT_EQ(91, controlRateConfig.rcRates[FD_PITCH]);
    EXPECT_FALSE(adjustmentState->ready);


    //
    // moving the switch back to the middle should immediately reset the state flag without increasing the value
    //


    // given
    rcData[AUX3] = PWM_RANGE_MIDDLE;

    // and
    fixedMillis = 498;

    // when
    processRcAdjustments(&controlRateConfig);

    EXPECT_EQ(91, controlRateConfig.rcRates[FD_ROLL]);
    EXPECT_EQ(91, controlRateConfig.rcRates[FD_PITCH]);
    EXPECT_TRUE(adjustmentState->ready);


    //
    // flipping the switch again, before the state reset would have occurred, allows the value to be increased again

    // given
    rcData[AUX3] = PWM_RANGE_MAX;

    // and
    fixedMillis = 499;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(92, controlRateConfig.rcRates[FD_ROLL]);
    EXPECT_EQ(92, controlRateConfig.rcRates[FD_PITCH]);
    EXPECT_EQ(2, CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP));
    EXPECT_FALSE(adjustmentState->ready);

    //
    // leaving the switch up, after the original timer would have reset the state should now NOT cause
    // the rate to increase, it should only increase after another 500ms from when the state was reset.
    //

    // given
    fixedMillis = 500;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(92, controlRateConfig.rcRates[FD_ROLL]);
    EXPECT_EQ(92, controlRateConfig.rcRates[FD_PITCH]);
    EXPECT_FALSE(adjustmentState->ready);

    //
    // should still not be able to be increased
    //

    // given
    fixedMillis = 997;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(92, controlRateConfig.rcRates[FD_ROLL]);
    EXPECT_EQ(92, controlRateConfig.rcRates[FD_PITCH]);
    EXPECT_FALSE(adjustmentState->ready);

    //
    // 500ms has now passed since the switch was returned to the middle, now that
    // switch is still in the UP position after the timer has elapses it should
    // be increased again.
    //

    // given
    fixedMillis = 998;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(93, controlRateConfig.rcRates[FD_ROLL]);
    EXPECT_EQ(93, controlRateConfig.rcRates[FD_PITCH]);
    EXPECT_EQ(3, CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP));
    EXPECT_FALSE(adjustmentState->ready);
}

#define ADJUSTMENT_RATE_PROFILE_INDEX 12

TEST_F(RcControlsAdjustmentsTest, processRcRateProfileAdjustments)
{
    // given
    configureContinuosAdjustment(AUX4 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_RATE_PROFILE_INDEX);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // and
    rcData[AUX4] = PWM_RANGE_MAX;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP));
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_CHANGE_CONTROL_RATE_PROFILE));
}

#define ADJUSTMENT_PITCH_ROLL_P_INDEX 6
#define ADJUSTMENT_PITCH_ROLL_I_INDEX 7
#define ADJUSTMENT_PITCH_ROLL_D_INDEX 8
#define ADJUSTMENT_YAW_P_INDEX 9
#define ADJUSTMENT_YAW_I_INDEX 10
#define ADJUSTMENT_YAW_D_INDEX 11

TEST_F(RcControlsAdjustmentsTest, processPIDIncreasePidController0)
{
    // given
    pidProfile_t pidProfile;
    memset(&pidProfile, 0, sizeof (pidProfile));
    pidProfile.pid[PID_PITCH].P = 0;
    pidProfile.pid[PID_PITCH].I = 10;
    pidProfile.pid[PID_PITCH].D = 20;
    pidProfile.pid[PID_ROLL].P = 5;
    pidProfile.pid[PID_ROLL].I = 15;
    pidProfile.pid[PID_ROLL].D = 25;
    pidProfile.pid[PID_YAW].P = 7;
    pidProfile.pid[PID_YAW].I = 17;
    pidProfile.pid[PID_YAW].D = 27;
    // and
    controlRateConfig_t controlRateConfig;
    memset(&controlRateConfig, 0, sizeof (controlRateConfig));

    const timedAdjustmentState_t *adjustmentState1 = configureStepwiseAdjustment(AUX1 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_PITCH_ROLL_P_INDEX);
    const timedAdjustmentState_t *adjustmentState2 = configureStepwiseAdjustment(AUX2 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_PITCH_ROLL_I_INDEX);
    const timedAdjustmentState_t *adjustmentState3 = configureStepwiseAdjustment(AUX3 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_PITCH_ROLL_D_INDEX);
    const timedAdjustmentState_t *adjustmentState4 = configureStepwiseAdjustment(AUX1 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_YAW_P_INDEX);
    const timedAdjustmentState_t *adjustmentState5 = configureStepwiseAdjustment(AUX2 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_YAW_I_INDEX);
    const timedAdjustmentState_t *adjustmentState6 = configureStepwiseAdjustment(AUX3 - NON_AUX_CHANNEL_COUNT, ADJUSTMENT_YAW_D_INDEX);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // and
    rcData[AUX1] = PWM_RANGE_MAX;
    rcData[AUX2] = PWM_RANGE_MAX;
    rcData[AUX3] = PWM_RANGE_MAX;

    // when
    currentPidProfile = &pidProfile;
    rcControlsInit();
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(6, CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP));
    EXPECT_FALSE(adjustmentState1->ready);
    EXPECT_FALSE(adjustmentState2->ready);
    EXPECT_FALSE(adjustmentState3->ready);
    EXPECT_FALSE(adjustmentState4->ready);
    EXPECT_FALSE(adjustmentState5->ready);
    EXPECT_FALSE(adjustmentState6->ready);

    // and
    EXPECT_EQ(1,  pidProfile.pid[PID_PITCH].P);
    EXPECT_EQ(11, pidProfile.pid[PID_PITCH].I);
    EXPECT_EQ(21, pidProfile.pid[PID_PITCH].D);
    EXPECT_EQ(6,  pidProfile.pid[PID_ROLL].P);
    EXPECT_EQ(16, pidProfile.pid[PID_ROLL].I);
    EXPECT_EQ(26, pidProfile.pid[PID_ROLL].D);
    EXPECT_EQ(8,  pidProfile.pid[PID_YAW].P);
    EXPECT_EQ(18, pidProfile.pid[PID_YAW].I);
    EXPECT_EQ(28, pidProfile.pid[PID_YAW].D);
}

extern "C" {
void setConfigDirty(void) {}
void saveConfigAndNotify(void) {}
void initRcProcessing(void) {}
void changePidProfile(uint8_t) {}
void pidInitConfig(const pidProfile_t *) {}
void accStartCalibration(void) {}
void gyroStartCalibration(bool isFirstArmingCalibration)
{
    UNUSED(isFirstArmingCalibration);
}
void applyAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
void handleInflightCalibrationStickPosition(void) {}
bool featureIsEnabled(uint32_t) { return false;}
bool sensors(uint32_t) { return false;}
void tryArm(void) {}
void disarm(flightLogDisarmReason_e) {}
void dashboardDisablePageCycling() {}
void dashboardEnablePageCycling() {}

bool failsafeIsActive() { return false; }
bool rxIsReceivingSignal() { return true; }

uint8_t getCurrentControlRateProfileIndex(void) {
    return 0;
}
void GPS_reset_home_position(void) {}
void baroSetGroundLevel(void) {}

void blackboxLogEvent(FlightLogEvent, flightLogEventData_t *) {}

bool cmsInMenu = false;
uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;
int16_t heading;
uint8_t stateFlags = 0;
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
pidProfile_t *currentPidProfile;
rxRuntimeState_t rxRuntimeState;
PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
void resetArmingDisabled(void) {}
timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 20000; }
armingDisableFlags_e getArmingDisableFlags(void) {
    return (armingDisableFlags_e) 0;
}
bool isTryingToArm(void) { return false; }
void resetTryingToArm(void) {}
void setLedProfile(uint8_t profile) { UNUSED(profile); }
uint8_t getLedProfile(void) { return 0; }
void compassStartCalibration(void) {}
void pinioBoxTaskControl(void) {}
}
