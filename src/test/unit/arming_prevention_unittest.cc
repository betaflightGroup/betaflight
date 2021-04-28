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

extern "C" {
    #include "blackbox/blackbox.h"
    #include "build/debug.h"
    #include "common/maths.h"
    #include "config/feature.h"
    #include "pg/motor.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "config/config.h"
    #include "fc/controlrate_profile.h"
    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/servos.h"
    #include "io/beeper.h"
    #include "io/gps.h"
    #include "rx/rx.h"
    #include "scheduler/scheduler.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"
    #include "telemetry/telemetry.h"
    #include "flight/gps_rescue.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER(pidConfig_t, pidConfig, PG_PID_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);
    PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);

    float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint16_t averageSystemLoadPercent = 0;
    uint8_t cliMode = 0;
    uint8_t debugMode = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    pidProfile_t *currentPidProfile;
    controlRateConfig_t *currentControlRateProfile;
    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    uint32_t targetPidLooptime;
    bool cmsInMenu = false;
    float axisPID_P[3], axisPID_I[3], axisPID_D[3], axisPIDSum[3];
    rxRuntimeState_t rxRuntimeState = {};
    uint16_t GPS_distanceToHome = 0;
    int16_t GPS_directionToHome = 0;
    acc_t acc = {};
    bool mockIsUpright = false;
    uint8_t activePidLoopDenom = 1;
}

uint32_t simulationFeatureFlags = 0;
uint32_t simulationTime = 0;
bool gyroCalibDone = false;
bool simulationHaveRx = false;

#include "gtest/gtest.h"

TEST(ArmingPreventionTest, CalibrationPowerOnGraceAngleThrottleArmSwitch)
{
    // given
    simulationTime = 0;
    gyroCalibDone = false;
    sensorsSet(SENSOR_GYRO);

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // and
    // default channel positions
    rcData[THROTTLE] = 1400;
    rcData[4] = 1800;

    // and
    systemConfigMutable()->powerOnArmingGraceTime = 5;
    setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_CALIBRATING | ARMING_DISABLED_ANGLE | ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // gyro calibration is done
    gyroCalibDone = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_ANGLE | ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // quad is level
    mockIsUpright = true;

    // when
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    rcData[THROTTLE] = 1000;

    // when
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arming grace time has elapsed
    simulationTime += systemConfig()->powerOnArmingGraceTime * 1e6;

    // when
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    rcData[4] = 1000;

    // when
    // arm guard time elapses
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(isArmingDisabled());
}

TEST(ArmingPreventionTest, ArmingGuardRadioLeftOnAndArmed)
{
    // given
    simulationTime = 0;
    gyroCalibDone = false;
    sensorsSet(SENSOR_GYRO);

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // and
    rcData[THROTTLE] = 1000;
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_CALIBRATING, getArmingDisableFlags());

    // given
    // arm channel takes a safe default value from the RX after power on
    rcData[4] = 1500;

    // and
    // a short time passes while calibration is in progress
    simulationTime += 1e6;

    // and
    // during calibration RF link is established and ARM switch is on
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_CALIBRATING | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // calibration is done
    gyroCalibDone = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arm switch is switched off by user
    rcData[4] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // arming enabled as arm switch has been off for sufficient time
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(isArmingDisabled());
}

TEST(ArmingPreventionTest, Prearm)
{
    // given
    simulationTime = 0;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXPREARM;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_NOPREARM, getArmingDisableFlags());

    // given
    // prearm is enabled
    rcData[5] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // arming enabled as arm switch has been off for sufficient time
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(isArmingDisabled());
}

TEST(ArmingPreventionTest, RadioTurnedOnAtAnyTimeArmed)
{
    // given
    simulationTime = 30e6; // 30 seconds after boot
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // and
    rcData[THROTTLE] = 1000;
    mockIsUpright = true;

    // and
    // RX has no link to radio
    simulationHaveRx = false;

    // and
    // arm channel has a safe default value
    rcData[4] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // RF link is established and arm switch is turned on on radio
    simulationHaveRx = true;
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BAD_RX_RECOVERY | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arm switch turned off by user
    rcData[4] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
}

TEST(ArmingPreventionTest, In3DModeAllowArmingWhenEnteringThrottleDeadband)
{
    // given
    simulationFeatureFlags = FEATURE_3D; // Using 3D mode
    simulationTime = 30e6; // 30 seconds after boot
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->midrc = 1500;
    flight3DConfigMutable()->deadband3d_throttle = 5;

    // and
    rcData[THROTTLE] = 1400;
    mockIsUpright = true;
    simulationHaveRx = true;

    // and
    // arm channel has a safe default value
    rcData[4] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // attempt to arm
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // throttle moved to centre
    rcData[THROTTLE] = 1496;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
}

TEST(ArmingPreventionTest, When3DModeDisabledThenNormalThrottleArmingConditionApplies)
{
    // given
    simulationFeatureFlags = FEATURE_3D; // Using 3D mode
    simulationTime = 30e6; // 30 seconds after boot
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOX3D;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;
    rxConfigMutable()->midrc = 1500;
    flight3DConfigMutable()->deadband3d_throttle = 5;

    // and
    // safe throttle value for 3D mode
    rcData[THROTTLE] = 1500;
    mockIsUpright = true;
    simulationHaveRx = true;

    // and
    // arm channel has a safe default value
    rcData[4] = 1100;

    // and
    // disable 3D mode is off (i.e. 3D mode is on)
    rcData[5] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // disable 3D mode
    rcData[5] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // attempt to arm
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // throttle moved low
    rcData[THROTTLE] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arm switch turned off
    rcData[4] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
}

TEST(ArmingPreventionTest, WhenUsingSwitched3DModeThenNormalThrottleArmingConditionApplies)
{
    // given
    simulationFeatureFlags = FEATURE_3D; // Using 3D mode
    simulationTime = 30e6; // 30 seconds after boot
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOX3D;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // and
    rcData[THROTTLE] = 1000;
    mockIsUpright = true;
    simulationHaveRx = true;

    // and
    // arm channel has a safe default value
    rcData[4] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // raise throttle to unsafe position
    rcData[THROTTLE] = 1500;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // attempt to arm
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // throttle moved low
    rcData[THROTTLE] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arm switch turned off
    rcData[4] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
}

TEST(ArmingPreventionTest, GPSRescueWithoutFixPreventsArm)
{
    // given
    simulationFeatureFlags = 0;
    simulationTime = 0;
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXGPSRESCUE;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    rcData[AUX1] = 1000;
    rcData[AUX2] = 1000;
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_GPS, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH|ARMING_DISABLED_GPS, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_GPS, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // receive GPS fix
    ENABLE_STATE(GPS_FIX);

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));
}

TEST(ArmingPreventionTest, GPSRescueSwitchPreventsArm)
{
    // given
    simulationFeatureFlags = 0;
    simulationTime = 0;
    gyroCalibDone = true;
    gpsSol.numSat = 5;
    ENABLE_STATE(GPS_FIX);

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXGPSRESCUE;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    rcData[AUX1] = 1000;
    rcData[AUX2] = 1800; // Start out with rescue enabled
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_RESC, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH|ARMING_DISABLED_RESC, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_RESC, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disable Rescue
    rcData[AUX2] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));
}

TEST(ArmingPreventionTest, ParalyzeOnAtBoot)
{
    // given
    simulationFeatureFlags = 0;
    simulationTime = 0;
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXPARALYZE;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    rcData[AUX1] = 1000;
    rcData[AUX2] = 1800; // Paralyze on at boot
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXPARALYZE));

    // when
    updateActivatedModes();

    // expect
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXPARALYZE));
}

TEST(ArmingPreventionTest, Paralyze)
{
    // given
    simulationFeatureFlags = 0;
    simulationTime = 0;
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXPARALYZE;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(2)->auxChannelIndex = 2;
    modeActivationConditionsMutable(2)->modeId = BOXBEEPERON;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(3)->modeId = BOXVTXPITMODE;
    modeActivationConditionsMutable(3)->linkedTo = BOXPARALYZE;
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    rcData[AUX1] = 1000;
    rcData[AUX2] = 1800; // Start out with paralyze enabled
    rcData[AUX3] = 1000;
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXPARALYZE));

    // given
    simulationTime = 10e6; // 10 seconds after boot

    // when
    updateActivatedModes();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXPARALYZE));

    // given
    // disable paralyze once after the startup timer
    rcData[AUX2] = 1000;

    // when
    updateActivatedModes();

    // enable paralyze again
    rcData[AUX2] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_PARALYZE, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXPARALYZE));
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXVTXPITMODE));
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXBEEPERON));

    // given
    // enable beeper
    rcData[AUX3] = 1800;

    // when
    updateActivatedModes();

    // expect
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXVTXPITMODE));
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXBEEPERON));
    
    // given
    // try exiting paralyze mode and ensure arming and pit mode are still disabled
    rcData[AUX2] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_PARALYZE, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXPARALYZE));
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXVTXPITMODE));
}

// STUBS
extern "C" {
    uint32_t micros(void) { return simulationTime; }
    uint32_t millis(void) { return micros() / 1000; }
    bool rxIsReceivingSignal(void) { return simulationHaveRx; }

    bool featureIsEnabled(uint32_t f) { return simulationFeatureFlags & f; }
    void warningLedFlash(void) {}
    void warningLedDisable(void) {}
    void warningLedUpdate(void) {}
    void beeper(beeperMode_e) {}
    void beeperConfirmationBeeps(uint8_t) {}
    void beeperWarningBeeps(uint8_t) {}
    void beeperSilence(void) {}
    void systemBeep(bool) {}
    void saveConfigAndNotify(void) {}
    void blackboxFinish(void) {}
    bool accIsCalibrationComplete(void) { return true; }
    bool baroIsCalibrationComplete(void) { return true; }
    bool gyroIsCalibrationComplete(void) { return gyroCalibDone; }
    void gyroStartCalibration(bool) {}
    bool isFirstArmingGyroCalibrationRunning(void) { return false; }
    void pidController(const pidProfile_t *, timeUs_t) {}
    void pidStabilisationState(pidStabilisationState_e) {}
    void mixTable(timeUs_t) {};
    void writeMotors(void) {};
    void writeServos(void) {};
    bool calculateRxChannelsAndUpdateFailsafe(timeUs_t) { return true; }
    bool isMixerUsingServos(void) { return false; }
    void gyroUpdate(void) {}
    timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 0; }
    void updateRSSI(timeUs_t) {}
    bool failsafeIsMonitoring(void) { return false; }
    void failsafeStartMonitoring(void) {}
    void failsafeUpdateState(void) {}
    bool failsafeIsActive(void) { return false; }
    void pidResetIterm(void) {}
    void updateAdjustmentStates(void) {}
    void processRcAdjustments(controlRateConfig_t *) {}
    void updateGpsWaypointsAndMode(void) {}
    void mspSerialReleaseSharedTelemetryPorts(void) {}
    void telemetryCheckState(void) {}
    void mspSerialAllocatePorts(void) {}
    void gyroReadTemperature(void) {}
    void updateRcCommands(void) {}
    void applyAltHold(void) {}
    void resetYawAxis(void) {}
    int16_t calculateThrottleAngleCorrection(uint8_t) { return 0; }
    void processRcCommand(void) {}
    void updateGpsStateForHomeAndHoldMode(void) {}
    void blackboxUpdate(timeUs_t) {}
    void transponderUpdate(timeUs_t) {}
    void GPS_reset_home_position(void) {}
    void accStartCalibration(void) {}
    bool accHasBeenCalibrated(void) { return true; }
    void baroSetGroundLevel(void) {}
    void changePidProfile(uint8_t) {}
    void changeControlRateProfile(uint8_t) {}
    void dashboardEnablePageCycling(void) {}
    void dashboardDisablePageCycling(void) {}
    bool imuQuaternionHeadfreeOffsetSet(void) { return true; }
    void rescheduleTask(taskId_e, timeDelta_t) {}
    bool usbCableIsInserted(void) { return false; }
    bool usbVcpIsConnected(void) { return false; }
    void pidSetAntiGravityState(bool) {}
    void osdSuppressStats(bool) {}
    float scaleRangef(float, float, float, float, float) { return 0.0f; }
    bool crashRecoveryModeActive(void) { return false; }
    int32_t getEstimatedAltitudeCm(void) { return 0; }
    bool gpsIsHealthy() { return false; }
    bool isAltitudeOffset(void) { return false; }
    float getCosTiltAngle(void) { return 0.0f; }
    void pidSetItermReset(bool) {}
    void applyAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
    bool isFixedWing(void) { return false; }
    void compassStartCalibration(void) {}
    bool compassIsCalibrationComplete(void) { return true; }
    bool isUpright(void) { return mockIsUpright; }
    void blackboxLogEvent(FlightLogEvent, union flightLogEventData_u *) {};
    void gyroFiltering(timeUs_t) {};
    timeDelta_t rxGetFrameDelta(timeDelta_t *) { return 0; }
    void updateRcRefreshRate(timeUs_t) {};
    uint16_t getAverageSystemLoadPercent(void) { return 0; }
    bool isMotorProtocolEnabled(void) { return true; }
    void pinioBoxTaskControl(void) {}
}
