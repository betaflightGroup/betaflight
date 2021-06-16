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

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/time.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/pid_init.h"

#include "io/beeper.h"
#include "io/ledstrip.h"
#include "io/pidaudio.h"

#include "osd/osd.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "rc_adjustments.h"

#define ADJUSTMENT_RANGE_COUNT_INVALID -1

PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 2);

uint8_t pidAudioPositionToModeMap[7] = {
    // on a pot with a center detent, it's easy to have center area for off/default, then three positions to the left and three to the right.
    // current implementation yields RC values as below.

    PID_AUDIO_PIDSUM_X,     //   900 - ~1071 - Min
    PID_AUDIO_PIDSUM_Y,     // ~1071 - ~1242
    PID_AUDIO_PIDSUM_XY,    // ~1242 - ~1414
    PID_AUDIO_OFF,          // ~1414 - ~1585 - Center
    PID_AUDIO_OFF,          // ~1585 - ~1757
    PID_AUDIO_OFF,          // ~1757 - ~1928
    PID_AUDIO_OFF,          // ~1928 -  2100 - Max

    // Note: Last 3 positions are currently pending implementations and use PID_AUDIO_OFF for now.
};

STATIC_UNIT_TESTED int stepwiseAdjustmentCount = ADJUSTMENT_RANGE_COUNT_INVALID;
STATIC_UNIT_TESTED timedAdjustmentState_t stepwiseAdjustments[MAX_ADJUSTMENT_RANGE_COUNT];

STATIC_UNIT_TESTED int continuosAdjustmentCount;
STATIC_UNIT_TESTED continuosAdjustmentState_t continuosAdjustments[MAX_ADJUSTMENT_RANGE_COUNT];

static void blackboxLogInflightAdjustmentEvent(adjustmentFunction_e adjustmentFunction, int32_t newValue)
{
#ifndef USE_BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newValue);
#else
    if (blackboxConfig()->device) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newValue = newValue;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

// sync with adjustmentFunction_e
static const adjustmentConfig_t defaultAdjustmentConfigs[ADJUSTMENT_FUNCTION_COUNT - 1] = {
    {
        .adjustmentFunction = ADJUSTMENT_RC_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_RC_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_THROTTLE_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_RATE_PROFILE,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_RC_RATE_YAW,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_F,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_FEEDFORWARD_TRANSITION,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_HORIZON_STRENGTH,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 255 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PID_AUDIO,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = ARRAYLEN(pidAudioPositionToModeMap) }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_F,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_F,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_F,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_OSD_PROFILE,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }, {
        .adjustmentFunction = ADJUSTMENT_LED_PROFILE,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }
};

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
static const char * const adjustmentLabels[] = {
    "RC RATE",
    "RC EXPO",
    "THROTTLE EXPO",
    "ROLL RATE",
    "YAW RATE",
    "PITCH/ROLL P",
    "PITCH/ROLL I",
    "PITCH/ROLL D",
    "YAW P",
    "YAW I",
    "YAW D",
    "RATE PROFILE",
    "PITCH RATE",
    "ROLL RATE",
    "PITCH P",
    "PITCH I",
    "PITCH D",
    "ROLL P",
    "ROLL I",
    "ROLL D",
    "RC RATE YAW",
    "PITCH/ROLL F",
    "FF TRANSITION",
    "HORIZON STRENGTH",
    "ROLL RC RATE",
    "PITCH RC RATE",
    "ROLL RC EXPO",
    "PITCH RC EXPO",
    "PID AUDIO",
    "PITCH F",
    "ROLL F",
    "YAW F",
    "OSD PROFILE",
};

static int adjustmentRangeNameIndex = 0;
static int adjustmentRangeValue = -1;
#endif

static int applyStepAdjustment(controlRateConfig_t *controlRateConfig, uint8_t adjustmentFunction, int delta)
{
    beeperConfirmationBeeps(delta > 0 ? 2 : 1);
    int newValue;
    switch (adjustmentFunction) {
    case ADJUSTMENT_RC_RATE:
    case ADJUSTMENT_ROLL_RC_RATE:
        newValue = constrain((int)controlRateConfig->rcRates[FD_ROLL] + delta, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustmentFunction == ADJUSTMENT_ROLL_RC_RATE) {
            break;
        }
        // fall through for combined ADJUSTMENT_RC_EXPO
        FALLTHROUGH;
    case ADJUSTMENT_PITCH_RC_RATE:
        newValue = constrain((int)controlRateConfig->rcRates[FD_PITCH] + delta, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_RATE, newValue);
        break;
    case ADJUSTMENT_RC_EXPO:
    case ADJUSTMENT_ROLL_RC_EXPO:
        newValue = constrain((int)controlRateConfig->rcExpo[FD_ROLL] + delta, 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
        controlRateConfig->rcExpo[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_EXPO, newValue);
        if (adjustmentFunction == ADJUSTMENT_ROLL_RC_EXPO) {
            break;
        }
        // fall through for combined ADJUSTMENT_RC_EXPO
        FALLTHROUGH;
    case ADJUSTMENT_PITCH_RC_EXPO:
        newValue = constrain((int)controlRateConfig->rcExpo[FD_PITCH] + delta, 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
        controlRateConfig->rcExpo[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_EXPO, newValue);
        break;
    case ADJUSTMENT_THROTTLE_EXPO:
        newValue = constrain((int)controlRateConfig->thrExpo8 + delta, 0, 100); // FIXME magic numbers repeated in cli.c
        controlRateConfig->thrExpo8 = newValue;
        initRcProcessing();
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_THROTTLE_EXPO, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_RATE:
    case ADJUSTMENT_PITCH_RATE:
        newValue = constrain((int)controlRateConfig->rates[FD_PITCH] + delta, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RATE, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_RATE) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_RATE
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_RATE:
        newValue = constrain((int)controlRateConfig->rates[FD_ROLL] + delta, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RATE, newValue);
        break;
    case ADJUSTMENT_YAW_RATE:
        newValue = constrain((int)controlRateConfig->rates[FD_YAW] + delta, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_RATE, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_P:
    case ADJUSTMENT_PITCH_P:
        newValue = constrain((int)currentPidProfile->pid[PID_PITCH].P + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_PITCH].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_P, newValue);

        if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_P
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_P:
        newValue = constrain((int)currentPidProfile->pid[PID_ROLL].P + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_ROLL].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_P, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_I:
    case ADJUSTMENT_PITCH_I:
        newValue = constrain((int)currentPidProfile->pid[PID_PITCH].I + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_PITCH].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_I, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_I) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_I
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_I:
        newValue = constrain((int)currentPidProfile->pid[PID_ROLL].I + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_ROLL].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_I, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_D:
    case ADJUSTMENT_PITCH_D:
        newValue = constrain((int)currentPidProfile->pid[PID_PITCH].D + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_PITCH].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_D, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_D) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_D
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_D:
        newValue = constrain((int)currentPidProfile->pid[PID_ROLL].D + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_ROLL].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_D, newValue);
        break;
    case ADJUSTMENT_YAW_P:
        newValue = constrain((int)currentPidProfile->pid[PID_YAW].P + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_YAW].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_P, newValue);
        break;
    case ADJUSTMENT_YAW_I:
        newValue = constrain((int)currentPidProfile->pid[PID_YAW].I + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_YAW].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_I, newValue);
        break;
    case ADJUSTMENT_YAW_D:
        newValue = constrain((int)currentPidProfile->pid[PID_YAW].D + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_YAW].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_D, newValue);
        break;
    case ADJUSTMENT_RC_RATE_YAW:
        newValue = constrain((int)controlRateConfig->rcRates[FD_YAW] + delta, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RC_RATE_YAW, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_F:
    case ADJUSTMENT_PITCH_F:
        newValue = constrain(currentPidProfile->pid[PID_PITCH].F + delta, 0, 2000);
        currentPidProfile->pid[PID_PITCH].F = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_F, newValue);

        if (adjustmentFunction == ADJUSTMENT_PITCH_F) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_F
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_F:
        newValue = constrain(currentPidProfile->pid[PID_ROLL].F + delta, 0, 2000);
        currentPidProfile->pid[PID_ROLL].F = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_F, newValue);
        break;
    case ADJUSTMENT_YAW_F:
        newValue = constrain(currentPidProfile->pid[PID_YAW].F + delta, 0, 2000);
        currentPidProfile->pid[PID_YAW].F = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_F, newValue);
        break;
    case ADJUSTMENT_FEEDFORWARD_TRANSITION:
        newValue = constrain(currentPidProfile->feedforwardTransition + delta, 1, 100); // FIXME magic numbers repeated in cli.c
        currentPidProfile->feedforwardTransition = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_FEEDFORWARD_TRANSITION, newValue);
        break;
    default:
        newValue = -1;
        break;
    };

    return newValue;
}

static int applyAbsoluteAdjustment(controlRateConfig_t *controlRateConfig, adjustmentFunction_e adjustmentFunction, int value)
{
    int newValue;

    switch (adjustmentFunction) {
    case ADJUSTMENT_RC_RATE:
    case ADJUSTMENT_ROLL_RC_RATE:
        newValue = constrain(value, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustmentFunction == ADJUSTMENT_ROLL_RC_RATE) {
            break;
        }
        // fall through for combined ADJUSTMENT_RC_EXPO
        FALLTHROUGH;
    case ADJUSTMENT_PITCH_RC_RATE:
        newValue = constrain(value, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_RATE, newValue);
        break;
    case ADJUSTMENT_RC_EXPO:
    case ADJUSTMENT_ROLL_RC_EXPO:
        newValue = constrain(value, 1, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
        controlRateConfig->rcExpo[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_EXPO, newValue);
        if (adjustmentFunction == ADJUSTMENT_ROLL_RC_EXPO) {
            break;
        }
        // fall through for combined ADJUSTMENT_RC_EXPO
        FALLTHROUGH;
    case ADJUSTMENT_PITCH_RC_EXPO:
        newValue = constrain(value, 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
        controlRateConfig->rcExpo[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_EXPO, newValue);
        break;
    case ADJUSTMENT_THROTTLE_EXPO:
        newValue = constrain(value, 0, 100); // FIXME magic numbers repeated in cli.c
        controlRateConfig->thrExpo8 = newValue;
        initRcProcessing();
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_THROTTLE_EXPO, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_RATE:
    case ADJUSTMENT_PITCH_RATE:
        newValue = constrain(value, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RATE, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_RATE) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_RATE
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_RATE:
        newValue = constrain(value, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RATE, newValue);
        break;
    case ADJUSTMENT_YAW_RATE:
        newValue = constrain(value, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_RATE, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_P:
    case ADJUSTMENT_PITCH_P:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_PITCH].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_P, newValue);

        if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_P
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_P:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_ROLL].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_P, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_I:
    case ADJUSTMENT_PITCH_I:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_PITCH].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_I, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_I) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_I
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_I:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_ROLL].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_I, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_D:
    case ADJUSTMENT_PITCH_D:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_PITCH].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_D, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_D) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_D
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_D:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_ROLL].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_D, newValue);
        break;
    case ADJUSTMENT_YAW_P:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_YAW].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_P, newValue);
        break;
    case ADJUSTMENT_YAW_I:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_YAW].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_I, newValue);
        break;
    case ADJUSTMENT_YAW_D:
        newValue = constrain(value, 0, 200); // FIXME magic numbers repeated in cli.c
        currentPidProfile->pid[PID_YAW].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_D, newValue);
        break;
    case ADJUSTMENT_RC_RATE_YAW:
        newValue = constrain(value, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RC_RATE_YAW, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_F:
    case ADJUSTMENT_PITCH_F:
        newValue = constrain(value, 0, 2000);
        currentPidProfile->pid[PID_PITCH].F = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_F, newValue);

        if (adjustmentFunction == ADJUSTMENT_PITCH_F) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_F
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_F:
        newValue = constrain(value, 0, 2000);
        currentPidProfile->pid[PID_ROLL].F = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_F, newValue);
        break;
    case ADJUSTMENT_YAW_F:
        newValue = constrain(value, 0, 2000);
        currentPidProfile->pid[PID_YAW].F = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_F, newValue);
        break;
    case ADJUSTMENT_FEEDFORWARD_TRANSITION:
        newValue = constrain(value, 1, 100); // FIXME magic numbers repeated in cli.c
        currentPidProfile->feedforwardTransition = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_FEEDFORWARD_TRANSITION, newValue);
        break;
    default:
        newValue = -1;
        break;
    };

    return newValue;
}

static uint8_t applySelectAdjustment(adjustmentFunction_e adjustmentFunction, uint8_t position)
{
    uint8_t beeps = 0;

    switch (adjustmentFunction) {
    case ADJUSTMENT_RATE_PROFILE:
        if (getCurrentControlRateProfileIndex() != position) {
            changeControlRateProfile(position);
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RATE_PROFILE, position);

            beeps = position + 1;
        }
        break;
    case ADJUSTMENT_HORIZON_STRENGTH:
        {
            uint8_t newValue = constrain(position, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            if (currentPidProfile->pid[PID_LEVEL].D != newValue) {
                beeps = ((newValue - currentPidProfile->pid[PID_LEVEL].D) / 8) + 1;
                currentPidProfile->pid[PID_LEVEL].D = newValue;
                blackboxLogInflightAdjustmentEvent(ADJUSTMENT_HORIZON_STRENGTH, position);
            }
        }
        break;
    case ADJUSTMENT_PID_AUDIO:
#ifdef USE_PID_AUDIO
        {
            pidAudioModes_e newMode = pidAudioPositionToModeMap[position];
            if (newMode != pidAudioGetMode()) {
                pidAudioSetMode(newMode);
            }
        }
#endif
        break;
    case ADJUSTMENT_OSD_PROFILE:
#ifdef USE_OSD_PROFILES
        if (getCurrentOsdProfileIndex() != (position + 1)) {
            changeOsdProfileIndex(position+1);
        }
#endif
        break;
    case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
        if (getLedProfile() != position) {
            setLedProfile(position);
        }
#endif
        break;

    default:
        break;
    }

    if (beeps) {
        beeperConfirmationBeeps(beeps);
    }

    return position;
}

#define ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET 1

static void calcActiveAdjustmentRanges(void)
{
    adjustmentRange_t defaultAdjustmentRange;
    memset(&defaultAdjustmentRange, 0, sizeof(defaultAdjustmentRange));

    stepwiseAdjustmentCount = 0;
    continuosAdjustmentCount = 0;
    for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
        const adjustmentRange_t * const adjustmentRange = adjustmentRanges(i);
        if (memcmp(adjustmentRange, &defaultAdjustmentRange, sizeof(defaultAdjustmentRange)) != 0) {
            const adjustmentConfig_t *adjustmentConfig = &defaultAdjustmentConfigs[adjustmentRange->adjustmentConfig - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
            if (adjustmentRange->adjustmentCenter == 0 && adjustmentConfig->mode != ADJUSTMENT_MODE_SELECT) {
                timedAdjustmentState_t *adjustmentState = &stepwiseAdjustments[stepwiseAdjustmentCount++];
                adjustmentState->adjustmentRangeIndex = i;
                adjustmentState->timeoutAt = 0;
                adjustmentState->ready = true;
            } else {
                continuosAdjustmentState_t *adjustmentState = &continuosAdjustments[continuosAdjustmentCount++];
                adjustmentState->adjustmentRangeIndex = i;
                adjustmentState->lastRcData = 0;
            }
        }
    }
}

#define VALUE_DISPLAY_LATENCY_MS 2000

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
static void updateOsdAdjustmentData(int newValue, adjustmentFunction_e adjustmentFunction)
{
    static timeMs_t lastValueChangeMs;

    timeMs_t currentTimeMs = millis();
    if (newValue != -1
        && adjustmentFunction != ADJUSTMENT_RATE_PROFILE  // Rate profile already has an OSD element
#ifdef USE_OSD_PROFILES
        && adjustmentFunction != ADJUSTMENT_OSD_PROFILE
#endif
        ) {
        adjustmentRangeNameIndex = adjustmentFunction;
        adjustmentRangeValue = newValue;

        lastValueChangeMs = currentTimeMs;
    }

    if (cmp32(currentTimeMs, lastValueChangeMs + VALUE_DISPLAY_LATENCY_MS) >= 0) {
        adjustmentRangeNameIndex = 0;
    }
}
#endif

#define RESET_FREQUENCY_2HZ (1000 / 2)

static void processStepwiseAdjustments(controlRateConfig_t *controlRateConfig, const bool canUseRxData)
{
    const timeMs_t now = millis();

    for (int index = 0; index < stepwiseAdjustmentCount; index++) {
        timedAdjustmentState_t *adjustmentState = &stepwiseAdjustments[index];
        const adjustmentRange_t *const adjustmentRange = adjustmentRanges(adjustmentState->adjustmentRangeIndex);
        const adjustmentConfig_t *adjustmentConfig = &defaultAdjustmentConfigs[adjustmentRange->adjustmentConfig - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        const adjustmentFunction_e adjustmentFunction = adjustmentConfig->adjustmentFunction;

        if (!isRangeActive(adjustmentRange->auxChannelIndex, &adjustmentRange->range) ||
            adjustmentFunction == ADJUSTMENT_NONE) {
            adjustmentState->timeoutAt = 0;

            continue;
        }

        if (cmp32(now, adjustmentState->timeoutAt) >= 0) {
            adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
            adjustmentState->ready = true;
        }

        if (!canUseRxData) {
            continue;
        }

        const uint8_t channelIndex = NON_AUX_CHANNEL_COUNT + adjustmentRange->auxSwitchChannelIndex;

        if (adjustmentConfig->mode == ADJUSTMENT_MODE_STEP) {
            int delta;
            if (rcData[channelIndex] > rxConfig()->midrc + 200) {
                delta = adjustmentConfig->data.step;
            } else if (rcData[channelIndex] < rxConfig()->midrc - 200) {
                delta = -adjustmentConfig->data.step;
            } else {
                // returning the switch to the middle immediately resets the ready state
                adjustmentState->ready = true;
                adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
                continue;
            }
            if (!adjustmentState->ready) {
                continue;
            }

            int newValue = applyStepAdjustment(controlRateConfig, adjustmentFunction, delta);

            setConfigDirty();

            pidInitConfig(currentPidProfile);

            adjustmentState->ready = false;

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
            updateOsdAdjustmentData(newValue, adjustmentConfig->adjustmentFunction);
#else
            UNUSED(newValue);
#endif
        }
    }
}

static void setConfigDirtyIfNotPermanent(const channelRange_t *range)
{
    if (!(range->startStep == MIN_MODE_RANGE_STEP && range->endStep == MAX_MODE_RANGE_STEP)) {
        // Only set the configuration dirty if this range is NOT permanently enabled (and the config thus never used).
        setConfigDirty();
    }
}

static void processContinuosAdjustments(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < continuosAdjustmentCount; i++) {
        continuosAdjustmentState_t *adjustmentState = &continuosAdjustments[i];
        const adjustmentRange_t * const adjustmentRange = adjustmentRanges(adjustmentState->adjustmentRangeIndex);
        const uint8_t channelIndex = NON_AUX_CHANNEL_COUNT + adjustmentRange->auxSwitchChannelIndex;
        const adjustmentConfig_t *adjustmentConfig = &defaultAdjustmentConfigs[adjustmentRange->adjustmentConfig - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        const adjustmentFunction_e adjustmentFunction = adjustmentConfig->adjustmentFunction;

        if (isRangeActive(adjustmentRange->auxChannelIndex, &adjustmentRange->range) &&
            adjustmentFunction != ADJUSTMENT_NONE) {

            if (rcData[channelIndex] != adjustmentState->lastRcData) {
                int newValue = -1;

                if (adjustmentConfig->mode == ADJUSTMENT_MODE_SELECT) {
                    int switchPositions = adjustmentConfig->data.switchPositions;
                    if (adjustmentFunction == ADJUSTMENT_RATE_PROFILE && systemConfig()->rateProfile6PosSwitch) {
                        switchPositions =  6;
                    }
                    const uint16_t rangeWidth = (2100 - 900) / switchPositions;
                    const uint8_t position = (constrain(rcData[channelIndex], 900, 2100 - 1) - 900) / rangeWidth;
                    newValue = applySelectAdjustment(adjustmentFunction, position);

                    setConfigDirtyIfNotPermanent(&adjustmentRange->range);
                } else {
                    // If setting is defined for step adjustment and center value has been specified, apply values directly (scaled) from aux channel
                    if (adjustmentRange->adjustmentCenter &&
                        (adjustmentConfig->mode == ADJUSTMENT_MODE_STEP)) {
                        int value = (((rcData[channelIndex] - PWM_RANGE_MIDDLE) * adjustmentRange->adjustmentScale) / (PWM_RANGE_MIDDLE - PWM_RANGE_MIN)) + adjustmentRange->adjustmentCenter;

                        newValue = applyAbsoluteAdjustment(controlRateConfig, adjustmentFunction, value);

                        setConfigDirtyIfNotPermanent(&adjustmentRange->range);

                        pidInitConfig(currentPidProfile);
                    }
                }
#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
                updateOsdAdjustmentData(newValue, adjustmentConfig->adjustmentFunction);
#else
                UNUSED(newValue);
#endif
                adjustmentState->lastRcData = rcData[channelIndex];
            }
        } else {
            adjustmentState->lastRcData = 0;
        }
    }
}

void processRcAdjustments(controlRateConfig_t *controlRateConfig)
{
    const bool canUseRxData = rxIsReceivingSignal();

    // Recalculate the new active adjustments if required
    if (stepwiseAdjustmentCount == ADJUSTMENT_RANGE_COUNT_INVALID) {
        calcActiveAdjustmentRanges();
    }

    processStepwiseAdjustments(controlRateConfig, canUseRxData);

    if (canUseRxData) {
        processContinuosAdjustments(controlRateConfig);
    }

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
    // Hide the element if there is no change
    updateOsdAdjustmentData(-1, 0);
#endif
}

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
const char *getAdjustmentsRangeName(void)
{
    if (adjustmentRangeNameIndex > 0) {
        return &adjustmentLabels[adjustmentRangeNameIndex - 1][0];
    } else {
        return NULL;
    }
}

int getAdjustmentsRangeValue(void)
{
    return adjustmentRangeValue;
}
#endif

void activeAdjustmentRangeReset(void)
{
    stepwiseAdjustmentCount = ADJUSTMENT_RANGE_COUNT_INVALID;
}
