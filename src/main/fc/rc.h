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

#include "drivers/time.h"

#include "fc/rc_controls.h"

typedef enum {
    INTERPOLATION_CHANNELS_RP,
    INTERPOLATION_CHANNELS_RPY,
    INTERPOLATION_CHANNELS_RPYT,
    INTERPOLATION_CHANNELS_T,
    INTERPOLATION_CHANNELS_RPT,
} interpolationChannels_e;

#ifdef USE_RC_SMOOTHING_FILTER
#define RC_SMOOTHING_AUTO_FACTOR_MIN 0
#define RC_SMOOTHING_AUTO_FACTOR_MAX 250
#endif

void processRcCommand(void);
float getSetpointRate(int axis);
float getRcDeflection(int axis);
float getRcDeflectionAbs(int axis);
float getThrottlePIDAttenuation(void);
void updateRcCommands(void);
void resetYawAxis(void);
void initRcProcessing(void);
bool isMotorsReversed(void);
bool rcSmoothingIsEnabled(void);
rcSmoothingFilter_t *getRcSmoothingData(void);
bool rcSmoothingAutoCalculate(void);
bool rcSmoothingInitializationComplete(void);
float getRawSetpoint(int axis);
float getRcCommandDelta(int axis);
float applyCurve(int axis, float deflection);
bool getShouldUpdateFf();
void updateRcRefreshRate(timeUs_t currentTimeUs);
uint16_t getCurrentRxRefreshRate(void);
