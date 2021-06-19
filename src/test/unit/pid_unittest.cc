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
#include <cmath>

#include "unittest_macros.h"
#include "gtest/gtest.h"
#include "build/debug.h"

bool simulatedAirmodeEnabled = true;
float simulatedSetpointRate[3] = { 0,0,0 };
float simulatedPrevSetpointRate[3] = { 0,0,0 };
float simulatedRcDeflection[3] = { 0,0,0 };
float simulatedThrottlePIDAttenuation = 1.0f;
float simulatedMotorMixRange = 0.0f;

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

    #include "config/config.h"
    #include "config/config_reset.h"

    #include "drivers/sound_beeper.h"
    #include "drivers/time.h"

    #include "fc/controlrate_profile.h"
    #include "fc/core.h"
    #include "fc/rc.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/pid_init.h"

    #include "io/gps.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"

    gyro_t gyro;
    attitudeEulerAngles_t attitude;

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);

    bool unitLaunchControlActive = false;
    launchControlMode_e unitLaunchControlMode = LAUNCH_CONTROL_MODE_NORMAL;

    float getThrottlePIDAttenuation(void) { return simulatedThrottlePIDAttenuation; }
    float getMotorMixRange(void) { return simulatedMotorMixRange; }
    float getSetpointRate(int axis) { return simulatedSetpointRate[axis]; }
    bool isAirmodeActivated() { return simulatedAirmodeEnabled; }
    float getRcDeflectionAbs(int axis) { return fabsf(simulatedRcDeflection[axis]); }
    void systemBeep(bool) { }
    bool gyroOverflowDetected(void) { return false; }
    float getRcDeflection(int axis) { return simulatedRcDeflection[axis]; }
    void beeperConfirmationBeeps(uint8_t) { }
    bool isLaunchControlActive(void) {return unitLaunchControlActive; }
    void disarm(flightLogDisarmReason_e) { }
    float applyFeedforwardLimit(int axis, float value, float Kp, float currentPidSetpoint)
    {
        UNUSED(axis);
        UNUSED(Kp);
        UNUSED(currentPidSetpoint);
        return value;
    }
    void feedforwardInit(const pidProfile_t) { }
    float feedforwardApply(int axis, bool newRcFrame, feedforwardAveraging_t feedforwardAveraging)
    {
        UNUSED(newRcFrame);
        UNUSED(feedforwardAveraging);
        return simulatedSetpointRate[axis] - simulatedPrevSetpointRate[axis];
    }
    bool shouldApplyFeedforwardLimits(int axis)
    {
        UNUSED(axis);
        return true;
    }
    bool getShouldUpdateFeedforward() { return true; }
    void initRcProcessing(void) { }
}

pidProfile_t *pidProfile;

int loopIter = 0;

// Always use same defaults for testing in future releases even when defaults change
void setDefaultTestSettings(void) {
    pgResetAll();
    pidProfile = pidProfilesMutable(1);
    pidProfile->pid[PID_ROLL]  =  { 40, 40, 30, 65 };
    pidProfile->pid[PID_PITCH] =  { 58, 50, 35, 60 };
    pidProfile->pid[PID_YAW]   =  { 70, 45, 20, 60 };
    pidProfile->pid[PID_LEVEL] =  { 50, 50, 75, 0 };

    // Compensate for the upscaling done without 'use_integrated_yaw'
    pidProfile->pid[PID_YAW].I = pidProfile->pid[PID_YAW].I / 2.5f;

    pidProfile->pidSumLimit = PIDSUM_LIMIT;
    pidProfile->pidSumLimitYaw = PIDSUM_LIMIT_YAW;
    pidProfile->yaw_lowpass_hz = 0;
    pidProfile->dterm_lowpass_hz = 100;
    pidProfile->dterm_lowpass2_hz = 0;
    pidProfile->dterm_notch_hz = 260;
    pidProfile->dterm_notch_cutoff = 160;
    pidProfile->dterm_filter_type = FILTER_BIQUAD;
    pidProfile->itermWindupPointPercent = 50;
    pidProfile->pidAtMinThrottle = PID_STABILISATION_ON;
    pidProfile->levelAngleLimit = 55;
    pidProfile->feedforwardTransition = 100;
    pidProfile->yawRateAccelLimit = 100;
    pidProfile->rateAccelLimit = 0;
    pidProfile->antiGravityMode = ANTI_GRAVITY_SMOOTH;
    pidProfile->itermThrottleThreshold = 250;
    pidProfile->itermAcceleratorGain = 1000;
    pidProfile->crash_time = 500;
    pidProfile->crash_delay = 0;
    pidProfile->crash_recovery_angle = 10;
    pidProfile->crash_recovery_rate = 100;
    pidProfile->crash_dthreshold = 50;
    pidProfile->crash_gthreshold = 400;
    pidProfile->crash_setpoint_threshold = 350;
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_OFF;
    pidProfile->horizon_tilt_effect = 75;
    pidProfile->horizon_tilt_expert_mode = false;
    pidProfile->crash_limit_yaw = 200;
    pidProfile->itermLimit = 150;
    pidProfile->throttle_boost = 0;
    pidProfile->throttle_boost_cutoff = 15;
    pidProfile->iterm_rotation = false;
    pidProfile->iterm_relax = ITERM_RELAX_OFF,
    pidProfile->iterm_relax_cutoff = 11,
    pidProfile->iterm_relax_type = ITERM_RELAX_SETPOINT,
    pidProfile->abs_control_gain = 0,
    pidProfile->launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
    pidProfile->launchControlGain = 40,
    pidProfile->level_race_mode = false,

    gyro.targetLooptime = 8000;
}

timeUs_t currentTestTime(void) {
    return targetPidLooptime * loopIter++;
}

void resetTest(void) {
    loopIter = 0;
    simulatedThrottlePIDAttenuation = 1.0f;
    simulatedMotorMixRange = 0.0f;

    pidStabilisationState(PID_STABILISATION_OFF);
    DISABLE_ARMING_FLAG(ARMED);

    setDefaultTestSettings();
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidData[axis].P = 0;
        pidData[axis].I = 0;
        pidData[axis].D = 0;
        pidData[axis].F = 0;
        pidData[axis].Sum = 0;
        simulatedSetpointRate[axis] = 0;
        simulatedRcDeflection[axis] = 0;
        gyro.gyroADCf[axis] = 0;
    }
    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    attitude.values.yaw = 0;

    flightModeFlags = 0;
    unitLaunchControlActive = false;
    pidProfile->launchControlMode = unitLaunchControlMode;
    pidInit(pidProfile);
    loadControlRateProfile();

    currentControlRateProfile->levelExpo[FD_ROLL] = 0;
    currentControlRateProfile->levelExpo[FD_PITCH] = 0;

    // Run pidloop for a while after reset
    for (int loop = 0; loop < 20; loop++) {
        pidController(pidProfile, currentTestTime());
    }
}

void setStickPosition(int axis, float stickRatio) {
    simulatedPrevSetpointRate[axis] = simulatedSetpointRate[axis];
    simulatedSetpointRate[axis] = 1998.0f * stickRatio;
    simulatedRcDeflection[axis] = stickRatio;
}

// All calculations will have 10% tolerance
float calculateTolerance(float input) {
    return fabs(input * 0.1f);
}

TEST(pidControllerTest, testInitialisation)
{
    resetTest();

    // In initial state PIDsums should be 0
    for (int axis = 0; axis <= FD_YAW; axis++) {
        EXPECT_FLOAT_EQ(0, pidData[axis].P);
        EXPECT_FLOAT_EQ(0, pidData[axis].I);
        EXPECT_FLOAT_EQ(0, pidData[axis].D);
    }
}

TEST(pidControllerTest, testStabilisationDisabled) {
    ENABLE_ARMING_FLAG(ARMED);
    // Run few loops to make sure there is no error building up when stabilisation disabled

    for (int loop = 0; loop < 10; loop++) {
        pidController(pidProfile, currentTestTime());

        // PID controller should not do anything, while stabilisation disabled
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);
    }
}

TEST(pidControllerTest, testPidLoop) {
    // Make sure to start with fresh values
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());

    // Loop 1 - Expecting zero since there is no error
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Add some rotation on ROLL to generate error
    gyro.gyroADCf[FD_ROLL] = 100;
    pidController(pidProfile, currentTestTime());

    // Loop 2 - Expect PID loop reaction to ROLL error
    EXPECT_NEAR(-128.1, pidData[FD_ROLL].P, calculateTolerance(-128.1));
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_NEAR(-7.8, pidData[FD_ROLL].I, calculateTolerance(-7.8));
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_NEAR(-198.4, pidData[FD_ROLL].D, calculateTolerance(-198.4));
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Add some rotation on PITCH to generate error
    gyro.gyroADCf[FD_PITCH] = -100;
    pidController(pidProfile, currentTestTime());

    // Loop 3 - Expect PID loop reaction to PITCH error, ROLL is still in error
    EXPECT_NEAR(-128.1, pidData[FD_ROLL].P, calculateTolerance(-128.1));
    EXPECT_NEAR(185.8, pidData[FD_PITCH].P, calculateTolerance(185.8));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_NEAR(-15.6, pidData[FD_ROLL].I, calculateTolerance(-15.6));
    EXPECT_NEAR(9.8, pidData[FD_PITCH].I, calculateTolerance(9.8));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_NEAR(231.4, pidData[FD_PITCH].D, calculateTolerance(231.4));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Add some rotation on YAW to generate error
    gyro.gyroADCf[FD_YAW] = 100;
    pidController(pidProfile, currentTestTime());

    // Loop 4 - Expect PID loop reaction to PITCH error, ROLL and PITCH are still in error
    EXPECT_NEAR(-128.1, pidData[FD_ROLL].P, calculateTolerance(-128.1));
    EXPECT_NEAR(185.8, pidData[FD_PITCH].P, calculateTolerance(185.8));
    EXPECT_NEAR(-224.2, pidData[FD_YAW].P, calculateTolerance(-224.2));
    EXPECT_NEAR(-23.5, pidData[FD_ROLL].I, calculateTolerance(-23.5));
    EXPECT_NEAR(19.6, pidData[FD_PITCH].I, calculateTolerance(19.6));
    EXPECT_NEAR(-8.7, pidData[FD_YAW].I, calculateTolerance(-8.7));
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_NEAR(-132.25, pidData[FD_YAW].D, calculateTolerance(-132.25));

    // Simulate Iterm behaviour during mixer saturation
    simulatedMotorMixRange = 1.2f;
    pidController(pidProfile, currentTestTime());
    EXPECT_NEAR(-31.3, pidData[FD_ROLL].I, calculateTolerance(-31.3));
    EXPECT_NEAR(29.3, pidData[FD_PITCH].I, calculateTolerance(29.3));
    EXPECT_NEAR(-8.8, pidData[FD_YAW].I, calculateTolerance(-8.8));
    simulatedMotorMixRange = 0;

    // Match the stick to gyro to stop error
    simulatedSetpointRate[FD_ROLL] = 100;
    simulatedSetpointRate[FD_PITCH] = -100;
    simulatedSetpointRate[FD_YAW] = 100;

    for(int loop = 0; loop < 5; loop++) {
        pidController(pidProfile, currentTestTime());
    }
    // Iterm is stalled as it is not accumulating anymore
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_NEAR(-31.3, pidData[FD_ROLL].I, calculateTolerance(-31.3));
    EXPECT_NEAR(29.3, pidData[FD_PITCH].I, calculateTolerance(29.3));
    EXPECT_NEAR(-10.6, pidData[FD_YAW].I, calculateTolerance(-10.6));
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Now disable Stabilisation
    pidStabilisationState(PID_STABILISATION_OFF);
    pidController(pidProfile, currentTestTime());

    // Should all be zero again
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);
}

TEST(pidControllerTest, testPidLevel) {
    // Make sure to start with fresh values
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // Test Angle mode response
    enableFlightMode(ANGLE_MODE);
    float currentPidSetpoint = 30;
    rollAndPitchTrims_t angleTrim = { { 0, 0 } };

    currentPidSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(0, currentPidSetpoint);
    currentPidSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(0, currentPidSetpoint);

    // Test attitude response
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    currentPidSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(275, currentPidSetpoint);
    currentPidSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(-275, currentPidSetpoint);

    setStickPosition(FD_ROLL, -0.5f);
    setStickPosition(FD_PITCH, 0.5f);
    currentPidSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(-137.5, currentPidSetpoint);
    currentPidSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(137.5, currentPidSetpoint);

    attitude.values.roll = -275;
    attitude.values.pitch = 275;
    currentPidSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(0, currentPidSetpoint);
    currentPidSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(0, currentPidSetpoint);

    // Disable ANGLE_MODE
    disableFlightMode(ANGLE_MODE);
    currentPidSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(0, currentPidSetpoint);
    currentPidSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(0, currentPidSetpoint);

    // Test level mode expo
    enableFlightMode(ANGLE_MODE);
    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    currentControlRateProfile->levelExpo[FD_ROLL] = 50;
    currentControlRateProfile->levelExpo[FD_PITCH] = 26;
    currentPidSetpoint = pidLevel(FD_ROLL, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(85.9375, currentPidSetpoint);
    currentPidSetpoint = pidLevel(FD_PITCH, pidProfile, &angleTrim, currentPidSetpoint);
    EXPECT_FLOAT_EQ(-110.6875, currentPidSetpoint);
}


TEST(pidControllerTest, testPidHorizon) {
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    enableFlightMode(HORIZON_MODE);

    // Test full stick response
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    EXPECT_FLOAT_EQ(0, calcHorizonLevelStrength());

    // Expect full rate output on full stick
    // Test zero stick response
    setStickPosition(FD_ROLL, 0);
    setStickPosition(FD_PITCH, 0);
    EXPECT_FLOAT_EQ(1, calcHorizonLevelStrength());

    // Test small stick response
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    EXPECT_NEAR(0.82, calcHorizonLevelStrength(), calculateTolerance(0.82));
}

TEST(pidControllerTest, testMixerSaturation) {
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // Test full stick response
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    setStickPosition(FD_YAW, 1.0f);
    simulatedMotorMixRange = 2.0f;
    pidController(pidProfile, currentTestTime());

    // Expect no iterm accumulation for yaw
    EXPECT_FLOAT_EQ(150, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(-150, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);

    // Test itermWindup limit (note: windup limit now only affects yaw)
    // First store values without exceeding iterm windup limit
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    setStickPosition(FD_YAW, 0.1f);
    simulatedMotorMixRange = 0.0f;
    pidController(pidProfile, currentTestTime());
    float rollTestIterm = pidData[FD_ROLL].I;
    float pitchTestIterm = pidData[FD_PITCH].I;
    float yawTestIterm = pidData[FD_YAW].I;

    // Now compare values when exceeding the limit
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    setStickPosition(FD_YAW, 0.1f);
    simulatedMotorMixRange = (pidProfile->itermWindupPointPercent + 1) / 100.0f;
    pidController(pidProfile, currentTestTime());
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, rollTestIterm);
    EXPECT_FLOAT_EQ(pidData[FD_PITCH].I, pitchTestIterm);
    EXPECT_LT(pidData[FD_YAW].I, yawTestIterm);
}

// TODO - Add more scenarios
TEST(pidControllerTest, testCrashRecoveryMode) {
    resetTest();
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_ON;
    pidInit(pidProfile);
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    sensorsSet(SENSOR_ACC);

    EXPECT_FALSE(crashRecoveryModeActive());

    int loopsToCrashTime = (int)((pidProfile->crash_time * 1000) / targetPidLooptime) + 1;

    // generate crash detection for roll axis
    gyro.gyroADCf[FD_ROLL]  = 800;
    simulatedMotorMixRange = 1.2f;
    for (int loop =0; loop <= loopsToCrashTime; loop++) {
        gyro.gyroADCf[FD_ROLL] += gyro.gyroADCf[FD_ROLL];
        // advance the time to avoid initialized state prevention of crash recovery
        pidController(pidProfile, currentTestTime() + 2000000);
    }

    EXPECT_TRUE(crashRecoveryModeActive());
    // Add additional verifications
}

TEST(pidControllerTest, testFeedForward) {
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].F);

    // Match the stick to gyro to stop error
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    setStickPosition(FD_YAW, -1.0f);

    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(2232.78, pidData[FD_ROLL].F, calculateTolerance(2232.78));
    EXPECT_NEAR(-2061.03, pidData[FD_PITCH].F, calculateTolerance(-2061.03));
    EXPECT_NEAR(-2061.03, pidData[FD_YAW].F, calculateTolerance(-2061.03));

    // Match the stick to gyro to stop error
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    setStickPosition(FD_YAW, -0.5f);

    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(-558.20, pidData[FD_ROLL].F, calculateTolerance(-558.20));
    EXPECT_NEAR(515.26, pidData[FD_PITCH].F, calculateTolerance(515.26));
    EXPECT_NEAR(515.26, pidData[FD_YAW].F, calculateTolerance(515.26));

    setStickPosition(FD_ROLL, 0.0f);
    setStickPosition(FD_PITCH, 0.0f);
    setStickPosition(FD_YAW, 0.0f);

    for (int loop = 0; loop <= 15; loop++) {
        gyro.gyroADCf[FD_ROLL] += gyro.gyroADCf[FD_ROLL];
        pidController(pidProfile, currentTestTime());
    }

    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].F);
}

TEST(pidControllerTest, testItermRelax) {
    resetTest();
    pidProfile->iterm_relax = ITERM_RELAX_RP;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidProfile->iterm_relax_type = ITERM_RELAX_SETPOINT;
    pidInit(pidProfile);

    float itermErrorRate = 0;
    float currentPidSetpoint = 0;
    float gyroRate = 0;

    applyItermRelax(FD_PITCH, 0, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, 0);
    itermErrorRate = -10;
    currentPidSetpoint = 10;
    pidData[FD_PITCH].I = 10;

    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);

    EXPECT_NEAR(-8.16, itermErrorRate, calculateTolerance(-8.16));
    currentPidSetpoint += ITERM_RELAX_SETPOINT_THRESHOLD;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(0, itermErrorRate, calculateTolerance(0));
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(0, itermErrorRate, calculateTolerance(0));

    pidProfile->iterm_relax_type = ITERM_RELAX_GYRO;
    pidInit(pidProfile);

    currentPidSetpoint = 100;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, 0);
    gyroRate = 10;
    itermErrorRate = -10;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(7, itermErrorRate, calculateTolerance(7));
    gyroRate += 100;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(-10, itermErrorRate, calculateTolerance(-10));

    pidProfile->iterm_relax = ITERM_RELAX_RP_INC;
    pidInit(pidProfile);

    itermErrorRate = -10;
    pidData[FD_PITCH].I = 10;
    currentPidSetpoint = 10;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, -10);
    itermErrorRate = 10;
    pidData[FD_PITCH].I = -10;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, 10);
    itermErrorRate = -10;
    currentPidSetpoint = 10;
    applyItermRelax(FD_PITCH, pidData[FD_PITCH].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, -100);

    pidProfile->iterm_relax_type = ITERM_RELAX_SETPOINT;
    pidInit(pidProfile);

    itermErrorRate = -10;
    currentPidSetpoint = ITERM_RELAX_SETPOINT_THRESHOLD;
    applyItermRelax(FD_YAW, pidData[FD_YAW].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_FLOAT_EQ(itermErrorRate, -10);

    pidProfile->iterm_relax = ITERM_RELAX_RPY;
    pidInit(pidProfile);
    applyItermRelax(FD_YAW, pidData[FD_YAW].I, gyroRate, &itermErrorRate, &currentPidSetpoint);
    EXPECT_NEAR(-3.6, itermErrorRate, calculateTolerance(-3.6));
}

// TODO - Add more tests
TEST(pidControllerTest, testAbsoluteControl) {
    resetTest();
    pidProfile->abs_control_gain = 10;
    pidInit(pidProfile);
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    float gyroRate = 0;

    float itermErrorRate = 10;
    float currentPidSetpoint = 10;

    applyAbsoluteControl(FD_PITCH, gyroRate, &currentPidSetpoint, &itermErrorRate);

    EXPECT_NEAR(10.8, itermErrorRate, calculateTolerance(10.8));
    EXPECT_NEAR(10.8, currentPidSetpoint, calculateTolerance(10.8));

    applyAbsoluteControl(FD_PITCH, gyroRate, &currentPidSetpoint, &itermErrorRate);
    EXPECT_NEAR(10.8, itermErrorRate, calculateTolerance(10.8));
    EXPECT_NEAR(10.8, currentPidSetpoint, calculateTolerance(10.8));

    gyroRate = -53;
    axisError[FD_PITCH] = -60;
    applyAbsoluteControl(FD_PITCH, gyroRate, &currentPidSetpoint, &itermErrorRate);
    EXPECT_NEAR(-79.2, itermErrorRate, calculateTolerance(-79.2));
    EXPECT_NEAR(-79.2, currentPidSetpoint, calculateTolerance(-79.2));
}

TEST(pidControllerTest, testDtermFiltering) {
// TODO
}

TEST(pidControllerTest, testItermRotationHandling) {
    resetTest();
    pidInit(pidProfile);

    rotateItermAndAxisError();
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, 0);
    EXPECT_FLOAT_EQ(pidData[FD_PITCH].I, 0);
    EXPECT_FLOAT_EQ(pidData[FD_YAW].I, 0);

    pidProfile->iterm_rotation = true;
    pidInit(pidProfile);

    rotateItermAndAxisError();
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, 0);
    EXPECT_FLOAT_EQ(pidData[FD_PITCH].I, 0);
    EXPECT_FLOAT_EQ(pidData[FD_YAW].I, 0);

    pidData[FD_ROLL].I = 10;
    pidData[FD_PITCH].I = 1000;
    pidData[FD_YAW].I = 1000;
    gyro.gyroADCf[FD_ROLL] = -1000;
    rotateItermAndAxisError();
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, 10);
    EXPECT_NEAR(860.37, pidData[FD_PITCH].I, calculateTolerance(860.37));
    EXPECT_NEAR(1139.6, pidData[FD_YAW].I, calculateTolerance(1139.6));

    pidProfile->abs_control_gain = 10;
    pidInit(pidProfile);
    pidData[FD_ROLL].I = 10;
    pidData[FD_PITCH].I = 1000;
    pidData[FD_YAW].I = 1000;

    gyro.gyroADCf[FD_ROLL] = -1000;
    // FIXME - axisError changes don't affect the system. This is a potential bug or intendend behaviour?
    axisError[FD_PITCH] = 1000;
    axisError[FD_YAW] = 1000;
    rotateItermAndAxisError();
    EXPECT_FLOAT_EQ(pidData[FD_ROLL].I, 10);
    EXPECT_NEAR(860.37, pidData[FD_PITCH].I, calculateTolerance(860.37));
    EXPECT_NEAR(1139.6, pidData[FD_YAW].I, calculateTolerance(1139.6));
}

TEST(pidControllerTest, testLaunchControl) {
    // The launchControlGain is indirectly tested since when launch control is active the
    // the gain overrides the PID settings. If the logic to use launchControlGain wasn't
    // working then any I calculations would be incorrect.

    resetTest();
    unitLaunchControlActive = true;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // test that feedforward and D are disabled (always zero) when launch control is active
    // set initial state
    pidController(pidProfile, currentTestTime());

    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Move the sticks to induce feedforward
    setStickPosition(FD_ROLL, 0.5f);
    setStickPosition(FD_PITCH, -0.5f);
    setStickPosition(FD_YAW, -0.5f);

    // add gyro activity to induce D
    gyro.gyroADCf[FD_ROLL] = -1000;
    gyro.gyroADCf[FD_PITCH] = 1000;
    gyro.gyroADCf[FD_YAW] = -1000;

    pidController(pidProfile, currentTestTime());

    // validate that feedforwad is still 0
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].F);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].F);

    // validate that D is still 0
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // test NORMAL mode - expect P/I on roll and pitch, P on yaw but I == 0
    unitLaunchControlMode = LAUNCH_CONTROL_MODE_NORMAL;
    resetTest();
    unitLaunchControlActive = true;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());

    gyro.gyroADCf[FD_ROLL] = -20;
    gyro.gyroADCf[FD_PITCH] = 20;
    gyro.gyroADCf[FD_YAW] = -20;
    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(25.62,  pidData[FD_ROLL].P,  calculateTolerance(25.62));
    EXPECT_NEAR(1.56,   pidData[FD_ROLL].I,  calculateTolerance(1.56));
    EXPECT_NEAR(-37.15, pidData[FD_PITCH].P, calculateTolerance(-37.15));
    EXPECT_NEAR(-1.56,  pidData[FD_PITCH].I, calculateTolerance(-1.56));
    EXPECT_NEAR(44.84,  pidData[FD_YAW].P,   calculateTolerance(44.84));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);

    // test PITCHONLY mode - expect P/I only on pitch; I cannot go negative
    unitLaunchControlMode = LAUNCH_CONTROL_MODE_PITCHONLY;
    resetTest();
    unitLaunchControlActive = true;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());

    // first test that pitch I is prevented from going negative
    gyro.gyroADCf[FD_ROLL] = 0;
    gyro.gyroADCf[FD_PITCH] = 20;
    gyro.gyroADCf[FD_YAW] = 0;
    pidController(pidProfile, currentTestTime());

    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);

    gyro.gyroADCf[FD_ROLL] = 20;
    gyro.gyroADCf[FD_PITCH] = -20;
    gyro.gyroADCf[FD_YAW] = 20;
    pidController(pidProfile, currentTestTime());

    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_NEAR(37.15, pidData[FD_PITCH].P, calculateTolerance(37.15));
    EXPECT_NEAR(1.56,  pidData[FD_PITCH].I, calculateTolerance(1.56));
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);

    // test FULL mode - expect P/I on all axes
    unitLaunchControlMode = LAUNCH_CONTROL_MODE_FULL;
    resetTest();
    unitLaunchControlActive = true;
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, currentTestTime());

    gyro.gyroADCf[FD_ROLL] = -20;
    gyro.gyroADCf[FD_PITCH] = 20;
    gyro.gyroADCf[FD_YAW] = -20;
    pidController(pidProfile, currentTestTime());

    EXPECT_NEAR(25.62,  pidData[FD_ROLL].P,  calculateTolerance(25.62));
    EXPECT_NEAR(1.56,   pidData[FD_ROLL].I,  calculateTolerance(1.56));
    EXPECT_NEAR(-37.15, pidData[FD_PITCH].P, calculateTolerance(-37.15));
    EXPECT_NEAR(-1.56,  pidData[FD_PITCH].I, calculateTolerance(-1.56));
    EXPECT_NEAR(44.84,  pidData[FD_YAW].P,   calculateTolerance(44.84));
    EXPECT_NEAR(1.56,   pidData[FD_YAW].I,  calculateTolerance(1.56));
}
