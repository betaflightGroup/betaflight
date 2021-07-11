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
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/light_led.h"
#include "drivers/motor.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/transponder_ir.h"

#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"

#if defined(USE_DYN_NOTCH_FILTER)
#include "flight/dyn_notch_filter.h"
#endif

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/pidaudio.h"
#include "io/serial.h"
#include "io/servos.h"
#include "io/statusindicator.h"
#include "io/transponder_ir.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#include "core.h"


enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
    ARMING_DELAYED_CRASHFLIP = 2,
    ARMING_DELAYED_LAUNCH_CONTROL = 3,
};

#define GYRO_WATCHDOG_DELAY 80 //  delay for gyro sync

#ifdef USE_RUNAWAY_TAKEOFF
#define RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD         600   // The pidSum threshold required to trigger - corresponds to a pidSum value of 60% (raw 600) in the blackbox viewer
#define RUNAWAY_TAKEOFF_ACTIVATE_DELAY           75000 // (75ms) Time in microseconds where pidSum is above threshold to trigger
#define RUNAWAY_TAKEOFF_DEACTIVATE_STICK_PERCENT 15    // 15% - minimum stick deflection during deactivation phase
#define RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT  100   // 10.0% - pidSum limit during deactivation phase
#define RUNAWAY_TAKEOFF_GYRO_LIMIT_RP            15    // Roll/pitch 15 deg/sec threshold to prevent triggering during bench testing without props
#define RUNAWAY_TAKEOFF_GYRO_LIMIT_YAW           50    // Yaw 50 deg/sec threshold to prevent triggering during bench testing without props
#define RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT    75    // High throttle limit to accelerate deactivation (halves the deactivation delay)

#define DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE      0
#define DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY   1
#define DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY 2
#define DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME  3

#define DEBUG_RUNAWAY_TAKEOFF_TRUE  1
#define DEBUG_RUNAWAY_TAKEOFF_FALSE 0
#endif

#if defined(USE_GPS) || defined(USE_MAG)
int16_t magHold;
#endif

static FAST_DATA_ZERO_INIT uint8_t pidUpdateCounter;

static bool flipOverAfterCrashActive = false;

static timeUs_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

static int lastArmingDisabledReason = 0;
static timeUs_t lastDisarmTimeUs;
static int tryingToArm = ARMING_DELAYED_DISARMED;

#ifdef USE_RUNAWAY_TAKEOFF
static timeUs_t runawayTakeoffDeactivateUs = 0;
static timeUs_t runawayTakeoffAccumulatedUs = 0;
static bool runawayTakeoffCheckDisabled = false;
static timeUs_t runawayTakeoffTriggerUs = 0;
static bool runawayTakeoffTemporarilyDisabled = false;
#endif

#ifdef USE_LAUNCH_CONTROL
static launchControlState_e launchControlState = LAUNCH_CONTROL_DISABLED;

const char * const osdLaunchControlModeNames[] = {
    "NORMAL",
    "PITCHONLY",
    "FULL"
};
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(throttleCorrectionConfig_t, throttleCorrectionConfig, PG_THROTTLE_CORRECTION_CONFIG, 0);

PG_RESET_TEMPLATE(throttleCorrectionConfig_t, throttleCorrectionConfig,
    .throttle_correction_value = 0,      // could 10 with althold or 40 for fpv
    .throttle_correction_angle = 800     // could be 80.0 deg with atlhold or 45.0 for fpv
);

static bool isCalibrating(void)
{
    return (sensors(SENSOR_GYRO) && !gyroIsCalibrationComplete())
#ifdef USE_ACC
        || (sensors(SENSOR_ACC) && !accIsCalibrationComplete())
#endif
#ifdef USE_BARO
        || (sensors(SENSOR_BARO) && !baroIsCalibrationComplete())
#endif
#ifdef USE_MAG
        || (sensors(SENSOR_MAG) && !compassIsCalibrationComplete())
#endif
        ;
}

#ifdef USE_LAUNCH_CONTROL
bool canUseLaunchControl(void)
{
    if (!isFixedWing()
        && !isUsingSticksForArming()     // require switch arming for safety
        && IS_RC_MODE_ACTIVE(BOXLAUNCHCONTROL)
        && (!featureIsEnabled(FEATURE_MOTOR_STOP) || airmodeIsEnabled())  // can't use when motors are stopped
        && !featureIsEnabled(FEATURE_3D) // pitch control is not 3D aware
        && (flightModeFlags == 0)) {     // don't want to use unless in acro mode
        return true;
    }
    return false;
}
#endif

void resetArmingDisabled(void)
{
    lastArmingDisabledReason = 0;
}

#ifdef USE_ACC
static bool accNeedsCalibration(void)
{
    if (sensors(SENSOR_ACC)) {

        // Check to see if the ACC has already been calibrated
        if (accHasBeenCalibrated()) {
            return false;
        }

        // We've determined that there's a detected ACC that has not
        // yet been calibrated. Check to see if anything is using the
        // ACC that would be affected by the lack of calibration.

        // Check for any configured modes that use the ACC
        if (isModeActivationConditionPresent(BOXANGLE) ||
            isModeActivationConditionPresent(BOXHORIZON) ||
            isModeActivationConditionPresent(BOXGPSRESCUE) ||
            isModeActivationConditionPresent(BOXCAMSTAB) ||
            isModeActivationConditionPresent(BOXCALIB) ||
            isModeActivationConditionPresent(BOXACROTRAINER)) {

            return true;
        }

        // Launch Control only requires the ACC if a angle limit is set
        if (isModeActivationConditionPresent(BOXLAUNCHCONTROL) && currentPidProfile->launchControlAngleLimit) {
            return true;
        }

#ifdef USE_OSD
        // Check for any enabled OSD elements that need the ACC
        if (featureIsEnabled(FEATURE_OSD)) {
            if (osdNeedsAccelerometer()) {
                return true;
            }
        }
#endif

#ifdef USE_GPS_RESCUE
        // Check if failsafe will use GPS Rescue
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            return true;
        }
#endif
    }

    return false;
}
#endif

void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        // Check if the power on arming grace time has elapsed
        if ((getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME) && (millis() >= systemConfig()->powerOnArmingGraceTime * 1000)
#ifdef USE_DSHOT
            // We also need to prevent arming until it's possible to send DSHOT commands.
            // Otherwise if the initial arming is in crash-flip the motor direction commands
            // might not be sent.
            && (!isMotorProtocolDshot() || dshotStreamingCommandsAreEnabled())
#endif
        ) {
            // If so, unset the grace time arming disable flag
            unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }

        // Clear the crash flip active status
        flipOverAfterCrashActive = false;

        // If switch is used for arming then check it is not defaulting to on when the RX link recovers from a fault
        if (!isUsingSticksForArming()) {
            static bool hadRx = false;
            const bool haveRx = rxIsReceivingSignal();

            const bool justGotRxBack = !hadRx && haveRx;

            if (justGotRxBack && IS_RC_MODE_ACTIVE(BOXARM)) {
                // If the RX has just started to receive a signal again and the arm switch is on, apply arming restriction
                setArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            } else if (haveRx && !IS_RC_MODE_ACTIVE(BOXARM)) {
                // If RX signal is OK and the arm switch is off, remove arming restriction
                unsetArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            }

            hadRx = haveRx;
        }

        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        }

        if (calculateThrottleStatus() != THROTTLE_LOW) {
            setArmingDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_THROTTLE);
        }

        if (!isUpright() && !IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
            setArmingDisabled(ARMING_DISABLED_ANGLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ANGLE);
        }

        if (getAverageSystemLoadPercent() > LOAD_PERCENTAGE_ONE) {
            setArmingDisabled(ARMING_DISABLED_LOAD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_LOAD);
        }

        if (isCalibrating()) {
            setArmingDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_CALIBRATING);
        }

        if (isModeActivationConditionPresent(BOXPREARM)) {
            if (IS_RC_MODE_ACTIVE(BOXPREARM) && !ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
                unsetArmingDisabled(ARMING_DISABLED_NOPREARM);
            } else {
                setArmingDisabled(ARMING_DISABLED_NOPREARM);
            }
        }

#ifdef USE_GPS_RESCUE
        if (gpsRescueIsConfigured()) {
            if (gpsRescueConfig()->allowArmingWithoutFix || STATE(GPS_FIX) || ARMING_FLAG(WAS_EVER_ARMED) || IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
                unsetArmingDisabled(ARMING_DISABLED_GPS);
            } else {
                setArmingDisabled(ARMING_DISABLED_GPS);
            }
            if (IS_RC_MODE_ACTIVE(BOXGPSRESCUE)) {
                setArmingDisabled(ARMING_DISABLED_RESC);
            } else {
                unsetArmingDisabled(ARMING_DISABLED_RESC);
            }
        }
#endif

#ifdef USE_RPM_FILTER
        // USE_RPM_FILTER will only be defined if USE_DSHOT and USE_DSHOT_TELEMETRY are defined
        // If the RPM filter is anabled and any motor isn't providing telemetry, then disable arming
        if (isRpmFilterEnabled() && !isDshotTelemetryActive()) {
            setArmingDisabled(ARMING_DISABLED_RPMFILTER);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_RPMFILTER);
        }
#endif

#ifdef USE_DSHOT_BITBANG
        if (isDshotBitbangActive(&motorConfig()->dev) && dshotBitbangGetStatus() != DSHOT_BITBANG_STATUS_OK) {
            setArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        }
#endif

        if (IS_RC_MODE_ACTIVE(BOXPARALYZE)) {
            setArmingDisabled(ARMING_DISABLED_PARALYZE);
        }

#ifdef USE_ACC
        if (accNeedsCalibration()) {
            setArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        }
#endif

        if (!isMotorProtocolEnabled()) {
            setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL);
        }

        if (!isUsingSticksForArming()) {
            if (!IS_RC_MODE_ACTIVE(BOXARM)) {
#ifdef USE_RUNAWAY_TAKEOFF
                unsetArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
#endif
                unsetArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
            }

            /* Ignore ARMING_DISABLED_CALIBRATING if we are going to calibrate gyro on first arm */
            bool ignoreGyro = armingConfig()->gyro_cal_on_first_arm
                && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_CALIBRATING));

            /* Ignore ARMING_DISABLED_THROTTLE (once arm switch is on) if we are in 3D mode */
            bool ignoreThrottle = featureIsEnabled(FEATURE_3D)
                 && !IS_RC_MODE_ACTIVE(BOX3D)
                 && !flight3DConfig()->switched_mode3d
                 && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE));

            // If arming is disabled and the ARM switch is on
            if (isArmingDisabled()
                && !ignoreGyro
                && !ignoreThrottle
                && IS_RC_MODE_ACTIVE(BOXARM)) {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            } else if (!IS_RC_MODE_ACTIVE(BOXARM)) {
                unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            }
        }

        if (isArmingDisabled()) {
            warningLedFlash();
        } else {
            warningLedDisable();
        }

        warningLedUpdate();
    }
}

void disarm(flightLogDisarmReason_e reason)
{
    if (ARMING_FLAG(ARMED)) {
        if (!flipOverAfterCrashActive) {
            ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
        }
        DISABLE_ARMING_FLAG(ARMED);
        lastDisarmTimeUs = micros();

#ifdef USE_OSD
        if (flipOverAfterCrashActive || isLaunchControlActive()) {
            osdSuppressStats(true);
        }
#endif

#ifdef USE_BLACKBOX
        flightLogEvent_disarm_t eventData;
        eventData.reason = reason;
        blackboxLogEvent(FLIGHT_LOG_EVENT_DISARM, (flightLogEventData_t*)&eventData);

        if (blackboxConfig()->device && blackboxConfig()->mode != BLACKBOX_MODE_ALWAYS_ON) { // Close the log upon disarm except when logging mode is ALWAYS ON
            blackboxFinish();
        }
#else
        UNUSED(reason);
#endif
        BEEP_OFF;
#ifdef USE_DSHOT
        if (isMotorProtocolDshot() && flipOverAfterCrashActive && !featureIsEnabled(FEATURE_3D)) {
            dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
        }
#endif
#ifdef USE_PERSISTENT_STATS
        if (!flipOverAfterCrashActive) {
            statsOnDisarm();
        }
#endif

        flipOverAfterCrashActive = false;

        // if ARMING_DISABLED_RUNAWAY_TAKEOFF is set then we want to play it's beep pattern instead
        if (!(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
            beeper(BEEPER_DISARMING);      // emit disarm tone
        }
    }
}

void tryArm(void)
{
    if (armingConfig()->gyro_cal_on_first_arm) {
        gyroStartCalibration(true);
    }

    updateArmingStatus();

    if (!isArmingDisabled()) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }

        const timeUs_t currentTimeUs = micros();

#ifdef USE_DSHOT
        if (currentTimeUs - getLastDshotBeaconCommandTimeUs() < DSHOT_BEACON_GUARD_DELAY_US) {
            if (tryingToArm == ARMING_DELAYED_DISARMED) {
                if (IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
                    tryingToArm = ARMING_DELAYED_CRASHFLIP;
#ifdef USE_LAUNCH_CONTROL
                } else if (canUseLaunchControl()) {
                    tryingToArm = ARMING_DELAYED_LAUNCH_CONTROL;
#endif
                } else {
                    tryingToArm = ARMING_DELAYED_NORMAL;
                }
            }
            return;
        }

        if (isMotorProtocolDshot() && isModeActivationConditionPresent(BOXFLIPOVERAFTERCRASH)) {
            if (!(IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH) || (tryingToArm == ARMING_DELAYED_CRASHFLIP))) {
                flipOverAfterCrashActive = false;
                if (!featureIsEnabled(FEATURE_3D)) {
                    dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
                }
            } else {
                flipOverAfterCrashActive = true;
#ifdef USE_RUNAWAY_TAKEOFF
                runawayTakeoffCheckDisabled = false;
#endif
                if (!featureIsEnabled(FEATURE_3D)) {
                    dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);
                }
            }
        }
#endif

#ifdef USE_LAUNCH_CONTROL
        if (!flipOverAfterCrashActive && (canUseLaunchControl() || (tryingToArm == ARMING_DELAYED_LAUNCH_CONTROL))) {
            if (launchControlState == LAUNCH_CONTROL_DISABLED) {  // only activate if it hasn't already been triggered
                launchControlState = LAUNCH_CONTROL_ACTIVE;
            }
        }
#endif

#ifdef USE_OSD
        osdSuppressStats(false);
#endif
        ENABLE_ARMING_FLAG(ARMED);

        resetTryingToArm();

#ifdef USE_ACRO_TRAINER
        pidAcroTrainerInit();
#endif // USE_ACRO_TRAINER

        if (isModeActivationConditionPresent(BOXPREARM)) {
            ENABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
        }
        imuQuaternionHeadfreeOffsetSet();

#if defined(USE_DYN_NOTCH_FILTER)
        resetMaxFFT();
#endif

        disarmAt = currentTimeUs + armingConfig()->auto_disarm_delay * 1e6;   // start disarm timeout, will be extended when throttle is nonzero

        lastArmingDisabledReason = 0;

#ifdef USE_GPS
        GPS_reset_home_position();

        //beep to indicate arming
        if (featureIsEnabled(FEATURE_GPS)) {
            if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
                beeper(BEEPER_ARMING_GPS_FIX);
            } else {
                beeper(BEEPER_ARMING_GPS_NO_FIX);
            }
        } else {
            beeper(BEEPER_ARMING);
        }
#else
        beeper(BEEPER_ARMING);
#endif

#ifdef USE_PERSISTENT_STATS
        statsOnArm();
#endif

#ifdef USE_RUNAWAY_TAKEOFF
        runawayTakeoffDeactivateUs = 0;
        runawayTakeoffAccumulatedUs = 0;
        runawayTakeoffTriggerUs = 0;
#endif
    } else {
       resetTryingToArm();
        if (!isFirstArmingGyroCalibrationRunning()) {
            int armingDisabledReason = ffs(getArmingDisableFlags());
            if (lastArmingDisabledReason != armingDisabledReason) {
                lastArmingDisabledReason = armingDisabledReason;

                beeperWarningBeeps(armingDisabledReason);
            }
        }
    }
}

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

void handleInflightCalibrationStickPosition(void)
{
    if (AccInflightCalibrationMeasurementDone) {
        // trigger saving into eeprom after landing
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    } else {
        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
        if (AccInflightCalibrationArmed) {
            beeper(BEEPER_ACC_CALIBRATION);
        } else {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
        }
    }
}

static void updateInflightCalibrationState(void)
{
    if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rcData[THROTTLE] > rxConfig()->mincheck && !IS_RC_MODE_ACTIVE(BOXARM)) {   // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = false;
    }
    if (IS_RC_MODE_ACTIVE(BOXCALIB)) {      // Use the Calib Option to activate : Calib = TRUE measurement started, Land and Calib = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
            InflightcalibratingA = 50;
        AccInflightCalibrationActive = true;
    } else if (AccInflightCalibrationMeasurementDone && !ARMING_FLAG(ARMED)) {
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    }
}

#if defined(USE_GPS) || defined(USE_MAG)
static void updateMagHold(void)
{
    if (fabsf(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
        if (isUpright()) {
            rcCommand[YAW] -= dif * currentPidProfile->pid[PID_MAG].P / 30;    // 18 deg
        }
    } else
        magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}
#endif

#ifdef USE_VTX_CONTROL
static bool canUpdateVTX(void)
{
#ifdef USE_VTX_RTC6705
    return vtxRTC6705CanUpdate();
#endif
    return true;
}
#endif

#if defined(USE_RUNAWAY_TAKEOFF) || defined(USE_GPS_RESCUE)
// determine if the R/P/Y stick deflection exceeds the specified limit - integer math is good enough here.
bool areSticksActive(uint8_t stickPercentLimit)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis ++) {
        const uint8_t deadband = axis == FD_YAW ? rcControlsConfig()->yaw_deadband : rcControlsConfig()->deadband;
        uint8_t stickPercent = 0;
        if ((rcData[axis] >= PWM_RANGE_MAX) || (rcData[axis] <= PWM_RANGE_MIN)) {
            stickPercent = 100;
        } else {
            if (rcData[axis] > (rxConfig()->midrc + deadband)) {
                stickPercent = ((rcData[axis] - rxConfig()->midrc - deadband) * 100) / (PWM_RANGE_MAX - rxConfig()->midrc - deadband);
            } else if (rcData[axis] < (rxConfig()->midrc - deadband)) {
                stickPercent = ((rxConfig()->midrc - deadband - rcData[axis]) * 100) / (rxConfig()->midrc - deadband - PWM_RANGE_MIN);
            }
        }
        if (stickPercent >= stickPercentLimit) {
            return true;
        }
    }
    return false;
}
#endif

#ifdef USE_RUNAWAY_TAKEOFF
// allow temporarily disabling runaway takeoff prevention if we are connected
// to the configurator and the ARMING_DISABLED_MSP flag is cleared.
void runawayTakeoffTemporaryDisable(uint8_t disableFlag)
{
    runawayTakeoffTemporarilyDisabled = disableFlag;
}
#endif


// calculate the throttle stick percent - integer math is good enough here.
// returns negative values for reversed thrust in 3D mode
int8_t calculateThrottlePercent(void)
{
    uint8_t ret = 0;
    int channelData = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);

    if (featureIsEnabled(FEATURE_3D)
        && !IS_RC_MODE_ACTIVE(BOX3D)
        && !flight3DConfig()->switched_mode3d) {

        if (channelData > (rxConfig()->midrc + flight3DConfig()->deadband3d_throttle)) {
            ret = ((channelData - rxConfig()->midrc - flight3DConfig()->deadband3d_throttle) * 100) / (PWM_RANGE_MAX - rxConfig()->midrc - flight3DConfig()->deadband3d_throttle);
        } else if (channelData < (rxConfig()->midrc - flight3DConfig()->deadband3d_throttle)) {
            ret = -((rxConfig()->midrc - flight3DConfig()->deadband3d_throttle - channelData) * 100) / (rxConfig()->midrc - flight3DConfig()->deadband3d_throttle - PWM_RANGE_MIN);
        }
    } else {
        ret = constrain(((channelData - rxConfig()->mincheck) * 100) / (PWM_RANGE_MAX - rxConfig()->mincheck), 0, 100);
        if (featureIsEnabled(FEATURE_3D)
            && IS_RC_MODE_ACTIVE(BOX3D)
            && flight3DConfig()->switched_mode3d) {

            ret = -ret;  // 3D on a switch is active
        }
    }
    return ret;
}

uint8_t calculateThrottlePercentAbs(void)
{
    return ABS(calculateThrottlePercent());
}

static bool airmodeIsActivated;

bool isAirmodeActivated()
{
    return airmodeIsActivated;
}


/*
 * processRx called from taskUpdateRxMain
 */
bool processRx(timeUs_t currentTimeUs)
{
    timeDelta_t frameAgeUs;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);

    DEBUG_SET(DEBUG_RX_TIMING, 0, MIN(frameDeltaUs / 10, INT16_MAX));
    DEBUG_SET(DEBUG_RX_TIMING, 1, MIN(frameAgeUs / 10, INT16_MAX));

    if (!calculateRxChannelsAndUpdateFailsafe(currentTimeUs)) {
        return false;
    }

    updateRcRefreshRate(currentTimeUs);

    // in 3D mode, we need to be able to disarm by switch at any time
    if (featureIsEnabled(FEATURE_3D)) {
        if (!IS_RC_MODE_ACTIVE(BOXARM))
            disarm(DISARM_REASON_SWITCH);
    }

    updateRSSI(currentTimeUs);

    if (currentTimeUs > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
        failsafeStartMonitoring();
    }
    failsafeUpdateState();

    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const uint8_t throttlePercent = calculateThrottlePercentAbs();

    const bool launchControlActive = isLaunchControlActive();

    if (airmodeIsEnabled() && ARMING_FLAG(ARMED) && !launchControlActive) {
        if (throttlePercent >= rxConfig()->airModeActivateThreshold) {
            airmodeIsActivated = true; // Prevent iterm from being reset
        }
    } else {
        airmodeIsActivated = false;
    }

    /* In airmode iterm should be prevented to grow when Low thottle and Roll + Pitch Centered.
     This is needed to prevent iterm winding on the ground, but keep full stabilisation on 0 throttle while in air */
    if (throttleStatus == THROTTLE_LOW && !airmodeIsActivated && !launchControlActive) {
        pidSetItermReset(true);
        if (currentPidProfile->pidAtMinThrottle)
            pidStabilisationState(PID_STABILISATION_ON);
        else
            pidStabilisationState(PID_STABILISATION_OFF);
    } else {
        pidSetItermReset(false);
        pidStabilisationState(PID_STABILISATION_ON);
    }

#ifdef USE_RUNAWAY_TAKEOFF
    // If runaway_takeoff_prevention is enabled, accumulate the amount of time that throttle
    // is above runaway_takeoff_deactivate_throttle with the any of the R/P/Y sticks deflected
    // to at least runaway_takeoff_stick_percent percent while the pidSum on all axis is kept low.
    // Once the amount of accumulated time exceeds runaway_takeoff_deactivate_delay then disable
    // prevention for the remainder of the battery.

    if (ARMING_FLAG(ARMED)
        && pidConfig()->runaway_takeoff_prevention
        && !runawayTakeoffCheckDisabled
        && !flipOverAfterCrashActive
        && !runawayTakeoffTemporarilyDisabled
        && !isFixedWing()) {

        // Determine if we're in "flight"
        //   - motors running
        //   - throttle over runaway_takeoff_deactivate_throttle_percent
        //   - sticks are active and have deflection greater than runaway_takeoff_deactivate_stick_percent
        //   - pidSum on all axis is less then runaway_takeoff_deactivate_pidlimit
        bool inStableFlight = false;
        if (!featureIsEnabled(FEATURE_MOTOR_STOP) || airmodeIsEnabled() || (throttleStatus != THROTTLE_LOW)) { // are motors running?
            const uint8_t lowThrottleLimit = pidConfig()->runaway_takeoff_deactivate_throttle;
            const uint8_t midThrottleLimit = constrain(lowThrottleLimit * 2, lowThrottleLimit * 2, RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT);
            if ((((throttlePercent >= lowThrottleLimit) && areSticksActive(RUNAWAY_TAKEOFF_DEACTIVATE_STICK_PERCENT)) || (throttlePercent >= midThrottleLimit))
                && (fabsf(pidData[FD_PITCH].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)
                && (fabsf(pidData[FD_ROLL].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)
                && (fabsf(pidData[FD_YAW].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)) {

                inStableFlight = true;
                if (runawayTakeoffDeactivateUs == 0) {
                    runawayTakeoffDeactivateUs = currentTimeUs;
                }
            }
        }

        // If we're in flight, then accumulate the time and deactivate once it exceeds runaway_takeoff_deactivate_delay milliseconds
        if (inStableFlight) {
            if (runawayTakeoffDeactivateUs == 0) {
                runawayTakeoffDeactivateUs = currentTimeUs;
            }
            uint16_t deactivateDelay = pidConfig()->runaway_takeoff_deactivate_delay;
            // at high throttle levels reduce deactivation delay by 50%
            if (throttlePercent >= RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT) {
                deactivateDelay = deactivateDelay / 2;
            }
            if ((cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs) + runawayTakeoffAccumulatedUs) > deactivateDelay * 1000) {
                runawayTakeoffCheckDisabled = true;
            }

        } else {
            if (runawayTakeoffDeactivateUs != 0) {
                runawayTakeoffAccumulatedUs += cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs);
            }
            runawayTakeoffDeactivateUs = 0;
        }
        if (runawayTakeoffDeactivateUs == 0) {
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, runawayTakeoffAccumulatedUs / 1000);
        } else {
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_TRUE);
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, (cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs) + runawayTakeoffAccumulatedUs) / 1000);
        }
    } else {
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, DEBUG_RUNAWAY_TAKEOFF_FALSE);
    }
#endif

#ifdef USE_LAUNCH_CONTROL
    if (ARMING_FLAG(ARMED)) {
        if (launchControlActive && (throttlePercent > currentPidProfile->launchControlThrottlePercent)) {
            // throttle limit trigger reached, launch triggered
            // reset the iterms as they may be at high values from holding the launch position
            launchControlState = LAUNCH_CONTROL_TRIGGERED;
            pidResetIterm();
        }
    } else {
        if (launchControlState == LAUNCH_CONTROL_TRIGGERED) {
            // If trigger mode is MULTIPLE then reset the state when disarmed
            // and the mode switch is turned off.
            // For trigger mode SINGLE we never reset the state and only a single
            // launch is allowed until a reboot.
            if (currentPidProfile->launchControlAllowTriggerReset && !IS_RC_MODE_ACTIVE(BOXLAUNCHCONTROL)) {
                launchControlState = LAUNCH_CONTROL_DISABLED;
            }
        } else {
            launchControlState = LAUNCH_CONTROL_DISABLED;
        }
    }
#endif

    return true;
}

void processRxModes(timeUs_t currentTimeUs)
{
    static bool armedBeeperOn = false;
#ifdef USE_TELEMETRY
    static bool sharedPortTelemetryEnabled = false;
#endif
    const throttleStatus_e throttleStatus = calculateThrottleStatus();

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    const timeUs_t autoDisarmDelayUs = armingConfig()->auto_disarm_delay * 1e6;
    if (ARMING_FLAG(ARMED)
        && featureIsEnabled(FEATURE_MOTOR_STOP)
        && !isFixedWing()
        && !featureIsEnabled(FEATURE_3D)
        && !airmodeIsEnabled()
        && !FLIGHT_MODE(GPS_RESCUE_MODE)  // disable auto-disarm when GPS Rescue is active
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if ((autoDisarmDelayUs > 0) && (currentTimeUs > disarmAt)) {
                    // auto-disarm configured and delay is over
                    disarm(DISARM_REASON_THROTTLE_TIMEOUT);
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low - extend disarm time
                disarmAt = currentTimeUs + autoDisarmDelayUs;

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            // arming is via AUX switch; beep while throttle low
            if (throttleStatus == THROTTLE_LOW) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    } else {
        disarmAt = currentTimeUs + autoDisarmDelayUs;  // extend auto-disarm timer
    }

    if (!(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED))
#ifdef USE_CMS
        && !cmsInMenu
#endif
        ) {
        processRcStickPositions();
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    updateActivatedModes();

#ifdef USE_DSHOT
    /* Enable beep warning when the crash flip mode is active */
    if (flipOverAfterCrashActive) {
        beeper(BEEPER_CRASH_FLIP_MODE);
    }
#endif

    if (!cliMode && !(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED))) {
        processRcAdjustments(currentControlRateProfile);
    }

    bool canUseHorizonMode = true;

    if ((IS_RC_MODE_ACTIVE(BOXANGLE) || failsafeIsActive()) && (sensors(SENSOR_ACC))) {
        // bumpless transfer to Level mode
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
    }

    if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode) {

        DISABLE_FLIGHT_MODE(ANGLE_MODE);

        if (!FLIGHT_MODE(HORIZON_MODE)) {
            ENABLE_FLIGHT_MODE(HORIZON_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(HORIZON_MODE);
    }

#ifdef USE_GPS_RESCUE
    if (ARMING_FLAG(ARMED) && (IS_RC_MODE_ACTIVE(BOXGPSRESCUE) || (failsafeIsActive() && failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE))) {
        if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
            ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    }
#endif

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        LED1_ON;
        // increase frequency of attitude task to reduce drift when in angle or horizon mode
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(500));
    } else {
        LED1_OFF;
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(100));
    }

    if (!IS_RC_MODE_ACTIVE(BOXPREARM) && ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
        DISABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
    }

#if defined(USE_ACC) || defined(USE_MAG)
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
#if defined(USE_GPS) || defined(USE_MAG)
        if (IS_RC_MODE_ACTIVE(BOXMAG)) {
            if (!FLIGHT_MODE(MAG_MODE)) {
                ENABLE_FLIGHT_MODE(MAG_MODE);
                magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            }
        } else {
            DISABLE_FLIGHT_MODE(MAG_MODE);
        }
#endif
        if (IS_RC_MODE_ACTIVE(BOXHEADFREE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADADJ) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
            if (imuQuaternionHeadfreeOffsetSet()) {
               beeper(BEEPER_RX_SET);
            }
        }
    }
#endif

    if (IS_RC_MODE_ACTIVE(BOXPASSTHRU)) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

    if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }

#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        bool enableSharedPortTelemetry = (!isModeActivationConditionPresent(BOXTELEMETRY) && ARMING_FLAG(ARMED)) || (isModeActivationConditionPresent(BOXTELEMETRY) && IS_RC_MODE_ACTIVE(BOXTELEMETRY));
        if (enableSharedPortTelemetry && !sharedPortTelemetryEnabled) {
            mspSerialReleaseSharedTelemetryPorts();
            telemetryCheckState();

            sharedPortTelemetryEnabled = true;
        } else if (!enableSharedPortTelemetry && sharedPortTelemetryEnabled) {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();

            sharedPortTelemetryEnabled = false;
        }
    }
#endif

#ifdef USE_VTX_CONTROL
    vtxUpdateActivatedChannel();

    if (canUpdateVTX()) {
        handleVTXControlButton();
    }
#endif

#ifdef USE_ACRO_TRAINER
    pidSetAcroTrainerState(IS_RC_MODE_ACTIVE(BOXACROTRAINER) && sensors(SENSOR_ACC));
#endif // USE_ACRO_TRAINER

#ifdef USE_RC_SMOOTHING_FILTER
    if (ARMING_FLAG(ARMED) && !rcSmoothingInitializationComplete()) {
        beeper(BEEPER_RC_SMOOTHING_INIT_FAIL);
    }
#endif

    pidSetAntiGravityState(IS_RC_MODE_ACTIVE(BOXANTIGRAVITY) || featureIsEnabled(FEATURE_ANTI_GRAVITY));
}

static FAST_CODE void subTaskPidController(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_PIDLOOP) {startTime = micros();}
    // PID - note this is function pointer set by setPIDController()
    pidController(currentPidProfile, currentTimeUs);
    DEBUG_SET(DEBUG_PIDLOOP, 1, micros() - startTime);

#ifdef USE_RUNAWAY_TAKEOFF
    // Check to see if runaway takeoff detection is active (anti-taz), the pidSum is over the threshold,
    // and gyro rate for any axis is above the limit for at least the activate delay period.
    // If so, disarm for safety
    if (ARMING_FLAG(ARMED)
        && !isFixedWing()
        && pidConfig()->runaway_takeoff_prevention
        && !runawayTakeoffCheckDisabled
        && !flipOverAfterCrashActive
        && !runawayTakeoffTemporarilyDisabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable Runaway Takeoff triggering if GPS Rescue is active
        && (!featureIsEnabled(FEATURE_MOTOR_STOP) || airmodeIsEnabled() || (calculateThrottleStatus() != THROTTLE_LOW))) {

        if (((fabsf(pidData[FD_PITCH].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD)
            || (fabsf(pidData[FD_ROLL].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD)
            || (fabsf(pidData[FD_YAW].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD))
            && ((gyroAbsRateDps(FD_PITCH) > RUNAWAY_TAKEOFF_GYRO_LIMIT_RP)
                || (gyroAbsRateDps(FD_ROLL) > RUNAWAY_TAKEOFF_GYRO_LIMIT_RP)
                || (gyroAbsRateDps(FD_YAW) > RUNAWAY_TAKEOFF_GYRO_LIMIT_YAW))) {

            if (runawayTakeoffTriggerUs == 0) {
                runawayTakeoffTriggerUs = currentTimeUs + RUNAWAY_TAKEOFF_ACTIVATE_DELAY;
            } else if (currentTimeUs > runawayTakeoffTriggerUs) {
                setArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
                disarm(DISARM_REASON_RUNAWAY_TAKEOFF);
            }
        } else {
            runawayTakeoffTriggerUs = 0;
        }
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE, DEBUG_RUNAWAY_TAKEOFF_TRUE);
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY, runawayTakeoffTriggerUs == 0 ? DEBUG_RUNAWAY_TAKEOFF_FALSE : DEBUG_RUNAWAY_TAKEOFF_TRUE);
    } else {
        runawayTakeoffTriggerUs = 0;
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE, DEBUG_RUNAWAY_TAKEOFF_FALSE);
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);
    }
#endif


#ifdef USE_PID_AUDIO
    if (isModeActivationConditionPresent(BOXPIDAUDIO)) {
        pidAudioUpdate();
    }
#endif
}

static FAST_CODE_NOINLINE void subTaskPidSubprocesses(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_PIDLOOP) {
        startTime = micros();
    }

#if defined(USE_GPS) || defined(USE_MAG)
    if (sensors(SENSOR_GPS) || sensors(SENSOR_MAG)) {
        updateMagHold();
    }
#endif

#ifdef USE_BLACKBOX
    if (!cliMode && blackboxConfig()->device) {
        blackboxUpdate(currentTimeUs);
    }
#else
    UNUSED(currentTimeUs);
#endif

    DEBUG_SET(DEBUG_PIDLOOP, 3, micros() - startTime);
}

#ifdef USE_TELEMETRY
#define GYRO_TEMP_READ_DELAY_US 3e6    // Only read the gyro temp every 3 seconds
void subTaskTelemetryPollSensors(timeUs_t currentTimeUs)
{
    static timeUs_t lastGyroTempTimeUs = 0;

    if (cmpTimeUs(currentTimeUs, lastGyroTempTimeUs) >= GYRO_TEMP_READ_DELAY_US) {
        // Read out gyro temperature if used for telemmetry
        gyroReadTemperature();
        lastGyroTempTimeUs = currentTimeUs;
    }
}
#endif

static FAST_CODE void subTaskMotorUpdate(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_CYCLETIME) {
        startTime = micros();
        static uint32_t previousMotorUpdateTime;
        const uint32_t currentDeltaTime = startTime - previousMotorUpdateTime;
        debug[2] = currentDeltaTime;
        debug[3] = currentDeltaTime - targetPidLooptime;
        previousMotorUpdateTime = startTime;
    } else if (debugMode == DEBUG_PIDLOOP) {
        startTime = micros();
    }

    mixTable(currentTimeUs);

#ifdef USE_SERVOS
    // motor outputs are used as sources for servo mixing, so motors must be calculated using mixTable() before servos.
    if (isMixerUsingServos()) {
        writeServos();
    }
#endif

    writeMotors();

#ifdef USE_DSHOT_TELEMETRY_STATS
    if (debugMode == DEBUG_DSHOT_RPM_ERRORS && useDshotTelemetry) {
        const uint8_t motorCount = MIN(getMotorCount(), 4);
        for (uint8_t i = 0; i < motorCount; i++) {
            debug[i] = getDshotTelemetryMotorInvalidPercent(i);
        }
    }
#endif

    DEBUG_SET(DEBUG_PIDLOOP, 2, micros() - startTime);
}

static FAST_CODE_NOINLINE void subTaskRcCommand(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rcData[THROTTLE] <= rxConfig()->mincheck
#ifndef USE_QUAD_MIXER_ONLY
#ifdef USE_SERVOS
                && !((mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && servoConfig()->tri_unarmed_servo)
#endif
                && mixerConfig()->mixerMode != MIXER_AIRPLANE
                && mixerConfig()->mixerMode != MIXER_FLYING_WING
#endif
    ) {
        resetYawAxis();
    }

    processRcCommand();
}

FAST_CODE void taskGyroSample(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    gyroUpdate();
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        pidUpdateCounter = 0;
    }
    pidUpdateCounter++;
}

FAST_CODE bool gyroFilterReady(void)
{
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        return true;
    } else {
        return false;
    }
}

FAST_CODE bool pidLoopReady(void)
{
    if ((pidUpdateCounter % activePidLoopDenom) == (activePidLoopDenom / 2)) {
        return true;
    }
    return false;
}

FAST_CODE void taskFiltering(timeUs_t currentTimeUs)
{
    gyroFiltering(currentTimeUs);

}

// Function for loop trigger
FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)
{

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_GYROPID_SYNC)
    if (lockMainPID() != 0) return;
#endif

    // DEBUG_PIDLOOP, timings for:
    // 0 - gyroUpdate()
    // 1 - subTaskPidController()
    // 2 - subTaskMotorUpdate()
    // 3 - subTaskPidSubprocesses()
    DEBUG_SET(DEBUG_PIDLOOP, 0, micros() - currentTimeUs);

    subTaskRcCommand(currentTimeUs);
    subTaskPidController(currentTimeUs);
    subTaskMotorUpdate(currentTimeUs);
    subTaskPidSubprocesses(currentTimeUs);

    if (debugMode == DEBUG_CYCLETIME) {
        DEBUG_SET(DEBUG_CYCLETIME, 0, getTaskDeltaTimeUs(TASK_SELF));
        DEBUG_SET(DEBUG_CYCLETIME, 1, getAverageSystemLoadPercent());
    }
}

bool isFlipOverAfterCrashActive(void)
{
    return flipOverAfterCrashActive;
}

timeUs_t getLastDisarmTimeUs(void)
{
    return lastDisarmTimeUs;
}

bool isTryingToArm()
{
    return (tryingToArm != ARMING_DELAYED_DISARMED);
}

void resetTryingToArm()
{
    tryingToArm = ARMING_DELAYED_DISARMED;
}

bool isLaunchControlActive(void)
{
#ifdef USE_LAUNCH_CONTROL
    return launchControlState == LAUNCH_CONTROL_ACTIVE;
#else
    return false;
#endif
}
