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

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/bitarray.h"

    #include "fc/runtime_config.h"
    #include "fc/rc_modes.h"
    #include "fc/rc_controls.h"
    #include "fc/core.h"

    #include "flight/failsafe.h"

    #include "io/beeper.h"

    #include "drivers/io.h"
    #include "rx/rx.h"

    extern boxBitmask_t rcModeActivationMask;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint32_t testFeatureMask = 0;
uint16_t testMinThrottle = 0;
throttleStatus_e throttleStatus = THROTTLE_HIGH;

enum {
    COUNTER_MW_DISARM = 0,
};
#define CALL_COUNT_ITEM_COUNT 1

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

void resetCallCounters(void) {
    memset(&callCounts, 0, sizeof(callCounts));
}

#define TEST_MID_RC 1495            // something other than the default 1500 will suffice.
#define TEST_MIN_CHECK 1100;
#define PERIOD_OF_10_SCONDS 10000
#define DE_ACTIVATE_ALL_BOXES 0

uint32_t sysTickUptime;

void configureFailsafe(void)
{
    rxConfigMutable()->midrc = TEST_MID_RC;
    rxConfigMutable()->mincheck = TEST_MIN_CHECK;

    failsafeConfigMutable()->failsafe_delay = 10; // 1 second
    failsafeConfigMutable()->failsafe_off_delay = 50; // 5 seconds
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE1;
    failsafeConfigMutable()->failsafe_throttle = 1200;
    failsafeConfigMutable()->failsafe_throttle_low_delay = 50; // 5 seconds
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;
    sysTickUptime = 0;
}

void activateBoxFailsafe()
{
    boxBitmask_t newMask;
    bitArraySet(&newMask, BOXFAILSAFE);
    rcModeUpdate(&newMask);
}

void deactivateBoxFailsafe()
{
    boxBitmask_t newMask;
    memset(&newMask, 0, sizeof(newMask));
    rcModeUpdate(&newMask);
}

//
// Stepwise tests
//

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeInitialState)
{
    // given
    configureFailsafe();
    // and
    DISABLE_ARMING_FLAG(ARMED);

    // when
    failsafeInit();
    failsafeReset();

    // then
    EXPECT_FALSE(failsafeIsMonitoring());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeStartMonitoring)
{
    // when
    failsafeStartMonitoring();

    // then
    EXPECT_TRUE(failsafeIsMonitoring());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeFirstArmedCycle)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);

    // when
    failsafeOnValidDataFailed();                    // set last invalid sample at current time
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    // and
    failsafeUpdateState();

    // then
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeNotActivatedWhenReceivingData)
{
    // when
    for (sysTickUptime = 0; sysTickUptime < PERIOD_OF_10_SCONDS; sysTickUptime++) {
        failsafeOnValidDataReceived();

        failsafeUpdateState();

        // then
        EXPECT_FALSE(failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeDetectsRxLossAndStartsLanding)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);

    // and
    failsafeStartMonitoring();
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // set last valid sample at current time

    // when
    for (sysTickUptime = 0; sysTickUptime < (uint32_t)(PERIOD_RXDATA_FAILURE + failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND); sysTickUptime++) {
        failsafeOnValidDataFailed();

        failsafeUpdateState();

        // then
        EXPECT_FALSE(failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }

    // given
    sysTickUptime++;                                // adjust time to point just past the failure time to
    failsafeOnValidDataFailed();                    // cause a lost link

    // when
    failsafeUpdateState();

    // then
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_TRUE(failsafeIsActive());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeCausesLanding)
{
    // given
    sysTickUptime += failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;
    sysTickUptime++;

    // when
    // no call to failsafeOnValidDataReceived();
    failsafeUpdateState();

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // given
    failsafeOnValidDataFailed();                    // set last invalid sample at current time
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    // when
    failsafeUpdateState();

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // given
    sysTickUptime += PERIOD_OF_30_SECONDS + 1;      // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();

    // when
    failsafeUpdateState();

    // then
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM)); // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeDetectsRxLossAndJustDisarms)
{
    // given
    DISABLE_ARMING_FLAG(ARMED);
    resetCallCounters();

    // and
    failsafeStartMonitoring();
    throttleStatus = THROTTLE_LOW;                  // throttle LOW to go for a failsafe just-disarm procedure
    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // set last valid sample at current time

    // when
    for (sysTickUptime = 0; sysTickUptime < (uint32_t)(failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND + PERIOD_RXDATA_FAILURE); sysTickUptime++) {
        failsafeOnValidDataFailed();

        failsafeUpdateState();

        // then
        EXPECT_FALSE(failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }

    // given
    sysTickUptime++;                                // adjust time to point just past the failure time to
    failsafeOnValidDataFailed();                    // cause a lost link
    ENABLE_ARMING_FLAG(ARMED);                      // armed from here (disarmed state has cleared throttleLowPeriod).

    // when
    failsafeUpdateState();

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // given
    failsafeOnValidDataFailed();                    // set last invalid sample at current time
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    // when
    failsafeUpdateState();

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // given
    sysTickUptime += PERIOD_OF_3_SECONDS + 1;       // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();

    // when
    failsafeUpdateState();

    // then
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeSwitchModeKill)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();
    failsafeStartMonitoring();

    // and
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_KILL;

    activateBoxFailsafe();

    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // set last valid sample at current time
    sysTickUptime = PERIOD_RXDATA_FAILURE + 1;      // adjust time to point just past the failure time to
    failsafeOnValidDataFailed();                    // cause a lost link

    // when
    failsafeUpdateState();                          // kill switch handling should come first

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // given
    failsafeOnValidDataFailed();                    // set last invalid sample at current time
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    deactivateBoxFailsafe();

    // when
    failsafeUpdateState();

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // given
    sysTickUptime += PERIOD_OF_1_SECONDS + 1;       // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();

    // when
    failsafeUpdateState();

    // then
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}

TEST(FlightFailsafeTest, TestFailsafeSwitchModeStage2Drop)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();

    // and
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE2;
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT;



    sysTickUptime = 0;                              // restart time from 0
    activateBoxFailsafe();
    failsafeOnValidDataFailed();                    // box failsafe causes data to be invalid

    // when
    failsafeUpdateState();                          // should activate stage2 immediately

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // given
    sysTickUptime += PERIOD_OF_3_SECONDS + 1;       // adjust time to point just past the required additional recovery time
    deactivateBoxFailsafe();
    failsafeOnValidDataReceived();                  // inactive box failsafe gives valid data

    // when
    failsafeUpdateState();

    // then
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}

TEST(FlightFailsafeTest, TestFailsafeSwitchModeStage2Land)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();

    // and
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE2;
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;


    sysTickUptime = 0;                              // restart time from 0
    activateBoxFailsafe();
    failsafeOnValidDataFailed();                    // box failsafe causes data to be invalid

    // when
    failsafeUpdateState();                          // should activate stage2 immediately

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());


    sysTickUptime += failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND + 1;

    // given
    failsafeOnValidDataFailed();                    // set last invalid sample at current time

    // when
    failsafeUpdateState();

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // given
    sysTickUptime += PERIOD_OF_30_SECONDS + 1;       // adjust time to point just past the required additional recovery time

    // and
    deactivateBoxFailsafe();
    failsafeOnValidDataReceived();                   // inactive box failsafe gives valid data

    // when
    failsafeUpdateState();

    // then
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}


/****************************************************************************************/
//
// Additional non-stepwise tests
//
/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeNotActivatedWhenDisarmedAndRXLossIsDetected)
{
    // given
    resetCallCounters();
    configureFailsafe();

    // and
    failsafeInit();

    // and
    DISABLE_ARMING_FLAG(ARMED);

    // when
    failsafeStartMonitoring();

    // and
    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // set last valid sample at current time

    // when
    for (sysTickUptime = 0; sysTickUptime < PERIOD_RXDATA_FAILURE; sysTickUptime++) {
        failsafeOnValidDataFailed();

        failsafeUpdateState();

        // then
        EXPECT_FALSE(failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }

    // given
    sysTickUptime++;                                // adjust time to point just past the failure time to
    failsafeOnValidDataFailed();                    // cause a lost link

    // when
    failsafeUpdateState();

    // then
    EXPECT_TRUE(failsafeIsMonitoring());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // given
    // enough valid data is received
    uint32_t sysTickTarget = sysTickUptime + PERIOD_RXDATA_RECOVERY;
    for (; sysTickUptime < sysTickTarget; sysTickUptime++) {
        failsafeOnValidDataReceived();
        failsafeUpdateState();

        EXPECT_TRUE(isArmingDisabled());
    }

    // and
    sysTickUptime++;                                // adjust time to point just past the failure time to
    failsafeOnValidDataReceived();                  // cause link recovery

    // then
    EXPECT_FALSE(isArmingDisabled());
}

// STUBS

extern "C" {
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
float rcCommand[4];
int16_t debug[DEBUG16_VALUE_COUNT];
bool isUsingSticksToArm = true;

PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

uint32_t micros(void)
{
    return millis() * 1000;
}

throttleStatus_e calculateThrottleStatus()
{
    return throttleStatus;
}

void delay(uint32_t) {}

bool featureIsEnabled(uint32_t mask) {
    return (mask & testFeatureMask);
}

void disarm(flightLogDisarmReason_e) {
    callCounts[COUNTER_MW_DISARM]++;
}

void beeper(beeperMode_e mode) {
    UNUSED(mode);
}

uint16_t getCurrentMinthrottle(void)
{
    return testMinThrottle;
}

bool isUsingSticksForArming(void)
{
    return isUsingSticksToArm;
}

bool areSticksActive(uint8_t stickPercentLimit) {
    UNUSED(stickPercentLimit);
    return false;
}

void beeperConfirmationBeeps(uint8_t beepCount) { UNUSED(beepCount); }

bool crashRecoveryModeActive(void) { return false; }
void pinioBoxTaskControl(void) {}
}
