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

#include <math.h>
#include "platform.h"

#ifdef USE_INTERPOLATED_SP

#include "build/debug.h"

#include "common/maths.h"

#include "fc/rc.h"

#include "flight/pid.h"

#include "interpolated_setpoint.h"

static float setpointDeltaImpl[XYZ_AXIS_COUNT];
static float setpointDelta[XYZ_AXIS_COUNT];

typedef struct laggedMovingAverageCombined_s {
     laggedMovingAverage_t filter;
     float buf[4];
} laggedMovingAverageCombined_t;

laggedMovingAverageCombined_t  setpointDeltaAvg[XYZ_AXIS_COUNT];

static float prevSetpoint[XYZ_AXIS_COUNT]; // equals raw unless interpolated 
static float prevSetpointSpeed[XYZ_AXIS_COUNT]; // equals raw unless interpolated
static float prevAcceleration[XYZ_AXIS_COUNT]; // for accurate duplicate interpolation
static float prevRcCommandDelta[XYZ_AXIS_COUNT]; // for accurate duplicate interpolation

static bool prevDuplicatePacket[XYZ_AXIS_COUNT]; // to identify multiple identical packets
static uint8_t averagingCount;

static float ffMaxRateLimit[XYZ_AXIS_COUNT];
static float ffMaxRate[XYZ_AXIS_COUNT];

void interpolatedSpInit(const pidProfile_t *pidProfile) {
    const float ffMaxRateScale = pidProfile->ff_max_rate_limit * 0.01f;
    averagingCount = pidProfile->ff_interpolate_sp;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        ffMaxRate[i] = applyCurve(i, 1.0f);
        ffMaxRateLimit[i] = ffMaxRate[i] * ffMaxRateScale;
        laggedMovingAverageInit(&setpointDeltaAvg[i].filter, averagingCount, (float *)&setpointDeltaAvg[i].buf[0]);
    }
}

FAST_CODE_NOINLINE float interpolatedSpApply(int axis, bool newRcFrame, ffInterpolationType_t type) {

    if (newRcFrame) {
        float rcCommandDelta = getRcCommandDelta(axis);
        float setpoint = getRawSetpoint(axis);
        const float rxInterval = getCurrentRxRefreshRate() * 1e-6f;
        const float rxRate = 1.0f / rxInterval;
        float setpointSpeed = (setpoint - prevSetpoint[axis]) * rxRate;
        float absPrevSetpointSpeed = fabsf(prevSetpointSpeed[axis]);
        float setpointAcceleration = 0.0f;
        const float ffSmoothFactor = pidGetFfSmoothFactor();
        const float ffJitterFactor = pidGetFfJitterFactor();

        // calculate an attenuator from average of two most recent rcCommand deltas vs jitter threshold
        float ffAttenuator = 1.0f;
        if (ffJitterFactor) {
            if (rcCommandDelta < ffJitterFactor) {
                ffAttenuator = MAX(1.0f - ((rcCommandDelta + prevRcCommandDelta[axis]) / 2.0f) / ffJitterFactor, 0.0f);
                ffAttenuator = 1.0f - ffAttenuator * ffAttenuator;
            }
        }

        // interpolate setpoint if necessary
        if (rcCommandDelta == 0.0f) {
            if (prevDuplicatePacket[axis] == false && fabsf(setpoint) < 0.98f * ffMaxRate[axis]) {
                // first duplicate after movement
                // interpolate rawSetpoint by adding (speed + acceleration) * attenuator to previous setpoint
                setpoint = prevSetpoint[axis] + (prevSetpointSpeed[axis] + prevAcceleration[axis]) * ffAttenuator * rxInterval;
                // recalculate setpointSpeed and (later) acceleration from this new setpoint value
                setpointSpeed = (setpoint - prevSetpoint[axis]) * rxRate;
            }
            prevDuplicatePacket[axis] = true;
        } else {
            // movement!
            if (prevDuplicatePacket[axis] == true) {
                // don't boost the packet after a duplicate, the FF alone is enough, usually
                // in part because after a duplicate, the raw up-step is large, so the jitter attenuator is less active
                ffAttenuator = 0.0f;
            }
            prevDuplicatePacket[axis] = false;
        }
        prevSetpoint[axis] = setpoint;

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 2, lrintf(setpoint)); // setpoint after interpolations
        }

        float absSetpointSpeed = fabsf(setpointSpeed); // unsmoothed for kick prevention

        // calculate acceleration, smooth and attenuate it
        setpointAcceleration = setpointSpeed - prevSetpointSpeed[axis];
        setpointAcceleration = prevAcceleration[axis] + ffSmoothFactor * (setpointAcceleration - prevAcceleration[axis]);
        setpointAcceleration *= ffAttenuator;

        // smooth setpointSpeed but don't attenuate
        setpointSpeed = prevSetpointSpeed[axis] + ffSmoothFactor * (setpointSpeed - prevSetpointSpeed[axis]);

        prevSetpointSpeed[axis] = setpointSpeed;
        prevAcceleration[axis] = setpointAcceleration;
        prevRcCommandDelta[axis] = rcCommandDelta;

        setpointAcceleration *= pidGetDT();
        setpointDeltaImpl[axis] = setpointSpeed * pidGetDT();

        // calculate boost and prevent kick-back spike at max deflection
        const float ffBoostFactor = pidGetFfBoostFactor();
        float boostAmount = 0.0f;
        if (ffBoostFactor) {
            if (fabsf(setpoint) < 0.95f * ffMaxRate[axis] || absSetpointSpeed > 3.0f * absPrevSetpointSpeed) {
                boostAmount = ffBoostFactor * setpointAcceleration;
            }
        }

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 0, lrintf(setpointDeltaImpl[axis] * 100.0f)); // base FF
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 1, lrintf(boostAmount * 100.0f)); // boost amount
            // debug 2 is interpolated setpoint, above
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 3, lrintf(rcCommandDelta * 100.0f)); // rcCommand packet difference
        }

        // add boost to base feed forward
        setpointDeltaImpl[axis] += boostAmount;

        // apply averaging
        if (type == FF_INTERPOLATE_ON) {
            setpointDelta[axis] = setpointDeltaImpl[axis];
        } else {
            setpointDelta[axis] = laggedMovingAverageUpdate(&setpointDeltaAvg[axis].filter, setpointDeltaImpl[axis]);
        }
    }
    return setpointDelta[axis];
}

FAST_CODE_NOINLINE float applyFfLimit(int axis, float value, float Kp, float currentPidSetpoint) {
    switch (axis) {
    case FD_ROLL:
        DEBUG_SET(DEBUG_FF_LIMIT, 0, value);

        break;
    case FD_PITCH:
        DEBUG_SET(DEBUG_FF_LIMIT, 1, value);

        break;
    }

    if (fabsf(currentPidSetpoint) <= ffMaxRateLimit[axis]) {
        value = constrainf(value, (-ffMaxRateLimit[axis] - currentPidSetpoint) * Kp, (ffMaxRateLimit[axis] - currentPidSetpoint) * Kp);
    } else {
        value = 0;
    }

    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 2, value);
    }

    return value;
}

bool shouldApplyFfLimits(int axis)
{
    return ffMaxRateLimit[axis] != 0.0f && axis < FD_YAW;
}
#endif
