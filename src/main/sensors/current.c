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

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/adc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "current.h"

const char * const currentMeterSourceNames[CURRENT_METER_COUNT] = {
    "NONE", "ADC", "VIRTUAL", "ESC", "MSP"
};

const uint8_t currentMeterIds[] = {
    CURRENT_METER_ID_BATTERY_1,
#ifdef USE_VIRTUAL_CURRENT_METER
    CURRENT_METER_ID_VIRTUAL_1,
#endif
#ifdef USE_ESC_SENSOR
    CURRENT_METER_ID_ESC_COMBINED_1,
    CURRENT_METER_ID_ESC_MOTOR_1,
    CURRENT_METER_ID_ESC_MOTOR_2,
    CURRENT_METER_ID_ESC_MOTOR_3,
    CURRENT_METER_ID_ESC_MOTOR_4,
    CURRENT_METER_ID_ESC_MOTOR_5,
    CURRENT_METER_ID_ESC_MOTOR_6,
    CURRENT_METER_ID_ESC_MOTOR_7,
    CURRENT_METER_ID_ESC_MOTOR_8,
    CURRENT_METER_ID_ESC_MOTOR_9,
    CURRENT_METER_ID_ESC_MOTOR_10,
    CURRENT_METER_ID_ESC_MOTOR_11,
    CURRENT_METER_ID_ESC_MOTOR_12,
#endif
#ifdef USE_MSP_CURRENT_METER
    CURRENT_METER_ID_MSP_1,
#endif
};

const uint8_t supportedCurrentMeterCount = ARRAYLEN(currentMeterIds);

//
// ADC/Virtual/ESC/MSP shared
//

void currentMeterReset(currentMeter_t *meter)
{
    meter->amperage = 0;
    meter->amperageLatest = 0;
    meter->mAhDrawn = 0;
}

//
// ADC/Virtual shared
//

static pt1Filter_t adciBatFilter;

#ifndef CURRENT_METER_SCALE_DEFAULT
#define CURRENT_METER_SCALE_DEFAULT 400 // for Allegro ACS758LCB-100U (40mV/A)
#endif

#ifndef CURRENT_METER_OFFSET_DEFAULT
#define CURRENT_METER_OFFSET_DEFAULT 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig, PG_CURRENT_SENSOR_ADC_CONFIG, 0);

PG_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig,
    .scale = CURRENT_METER_SCALE_DEFAULT,
    .offset = CURRENT_METER_OFFSET_DEFAULT,
);

#ifdef USE_VIRTUAL_CURRENT_METER
PG_REGISTER(currentSensorVirtualConfig_t, currentSensorVirtualConfig, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, 0);
#endif

static int32_t currentMeterADCToCentiamps(const uint16_t src)
{

    const currentSensorADCConfig_t *config = currentSensorADCConfig();

    int32_t millivolts = ((uint32_t)src * getVrefMv()) / 4096;
    // y=x/m+b m is scale in (mV/10A) and b is offset in (mA)
    int32_t centiAmps = config->scale ? (millivolts * 10000 / (int32_t)config->scale + (int32_t)config->offset) / 10 : 0;

    DEBUG_SET(DEBUG_CURRENT_SENSOR, 0, millivolts);
    DEBUG_SET(DEBUG_CURRENT_SENSOR, 1, centiAmps);

    return centiAmps; // Returns Centiamps to maintain compatability with the rest of the code
}

#if defined(USE_ADC) || defined(USE_VIRTUAL_CURRENT_METER)
static void updateCurrentmAhDrawnState(currentMeterMAhDrawnState_t *state, int32_t amperageLatest, int32_t lastUpdateAt)
{
    state->mAhDrawnF = state->mAhDrawnF + (amperageLatest * lastUpdateAt / (100.0f * 1000 * 3600));
    state->mAhDrawn = state->mAhDrawnF;
}
#endif

//
// ADC
//

currentMeterADCState_t currentMeterADCState;

void currentMeterADCInit(void)
{
    memset(&currentMeterADCState, 0, sizeof(currentMeterADCState_t));
    pt1FilterInit(&adciBatFilter, pt1FilterGain(GET_BATTERY_LPF_FREQUENCY(batteryConfig()->ibatLpfPeriod), HZ_TO_INTERVAL(50)));
}

void currentMeterADCRefresh(int32_t lastUpdateAt)
{
#ifdef USE_ADC
    const uint16_t iBatSample = adcGetChannel(ADC_CURRENT);
    currentMeterADCState.amperageLatest = currentMeterADCToCentiamps(iBatSample);
    currentMeterADCState.amperage = currentMeterADCToCentiamps(pt1FilterApply(&adciBatFilter, iBatSample));

    updateCurrentmAhDrawnState(&currentMeterADCState.mahDrawnState, currentMeterADCState.amperageLatest, lastUpdateAt);
#else
    UNUSED(lastUpdateAt);
    UNUSED(currentMeterADCToCentiamps);

    currentMeterADCState.amperageLatest = 0;
    currentMeterADCState.amperage = 0;
#endif
}

void currentMeterADCRead(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterADCState.amperageLatest;
    meter->amperage = currentMeterADCState.amperage;
    meter->mAhDrawn = currentMeterADCState.mahDrawnState.mAhDrawn;

    DEBUG_SET(DEBUG_CURRENT_SENSOR, 2, meter->amperageLatest);
    DEBUG_SET(DEBUG_CURRENT_SENSOR, 3, meter->mAhDrawn);
}

//
// VIRTUAL
//

#ifdef USE_VIRTUAL_CURRENT_METER
currentSensorVirtualState_t currentMeterVirtualState;

void currentMeterVirtualInit(void)
{
    memset(&currentMeterVirtualState, 0, sizeof(currentSensorVirtualState_t));
}

void currentMeterVirtualRefresh(int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset)
{
    currentMeterVirtualState.amperage = (int32_t)currentSensorVirtualConfig()->offset;
    if (armed) {
        if (throttleLowAndMotorStop) {
            throttleOffset = 0;
        }

        int throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50); // FIXME magic number 50. Possibly use thrustLinearization if configured.
        currentMeterVirtualState.amperage += throttleFactor * (int32_t)currentSensorVirtualConfig()->scale / 1000;
    }
    updateCurrentmAhDrawnState(&currentMeterVirtualState.mahDrawnState, currentMeterVirtualState.amperage, lastUpdateAt);
}

void currentMeterVirtualRead(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterVirtualState.amperage;
    meter->amperage = currentMeterVirtualState.amperage;
    meter->mAhDrawn = currentMeterVirtualState.mahDrawnState.mAhDrawn;
}
#endif

//
// ESC
//

#ifdef USE_ESC_SENSOR
currentMeterESCState_t currentMeterESCState;

void currentMeterESCInit(void)
{
    memset(&currentMeterESCState, 0, sizeof(currentMeterESCState_t));
}

void currentMeterESCRefresh(int32_t lastUpdateAt)
{
    UNUSED(lastUpdateAt);

    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData && escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        currentMeterESCState.amperage = escData->current + escSensorConfig()->offset / 10;
        currentMeterESCState.mAhDrawn = escData->consumption + escSensorConfig()->offset * millis() / (1000.0f * 3600);
    } else {
        currentMeterESCState.amperage = 0;
        currentMeterESCState.mAhDrawn = 0;
    }
}

void currentMeterESCReadCombined(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterESCState.amperage;
    meter->amperage = currentMeterESCState.amperage;
    meter->mAhDrawn = currentMeterESCState.mAhDrawn;
}

void currentMeterESCReadMotor(uint8_t motorNumber, currentMeter_t *meter)
{
    escSensorData_t *escData = getEscSensorData(motorNumber);
    if (escData && escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        meter->amperage = escData->current;
        meter->amperageLatest = escData->current;
        meter->mAhDrawn = escData->consumption;
    } else {
        currentMeterReset(meter);
    }
}
#endif


#ifdef USE_MSP_CURRENT_METER
#include "common/streambuf.h"

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

currentMeterMSPState_t currentMeterMSPState;

void currentMeterMSPSet(uint16_t amperage, uint16_t mAhDrawn)
{
    // We expect the FC's MSP_ANALOG response handler to call this function
    currentMeterMSPState.amperage = amperage;
    currentMeterMSPState.mAhDrawn = mAhDrawn;
}

void currentMeterMSPInit(void)
{
    memset(&currentMeterMSPState, 0, sizeof(currentMeterMSPState_t));
}

void currentMeterMSPRefresh(timeUs_t currentTimeUs)
{
    // periodically request MSP_ANALOG
    static timeUs_t streamRequestAt = 0;
    if (cmp32(currentTimeUs, streamRequestAt) > 0) {
        streamRequestAt = currentTimeUs + ((1000 * 1000) / 10); // 10hz

        mspSerialPush(SERIAL_PORT_ALL, MSP_ANALOG, NULL, 0, MSP_DIRECTION_REQUEST);
    }
}

void currentMeterMSPRead(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterMSPState.amperage;
    meter->amperage = currentMeterMSPState.amperage;
    meter->mAhDrawn = currentMeterMSPState.mAhDrawn;
}
#endif

//
// API for current meters using IDs
//
// This API is used by MSP, for configuration/status.
//

void currentMeterRead(currentMeterId_e id, currentMeter_t *meter)
{
    if (id == CURRENT_METER_ID_BATTERY_1) {
        currentMeterADCRead(meter);
    }
#ifdef USE_VIRTUAL_CURRENT_METER
    else if (id == CURRENT_METER_ID_VIRTUAL_1) {
        currentMeterVirtualRead(meter);
    }
#endif
#ifdef USE_MSP_CURRENT_METER
    else if (id == CURRENT_METER_ID_MSP_1) {
        currentMeterMSPRead(meter);
    }
#endif
#ifdef USE_ESC_SENSOR
    else if (id == CURRENT_METER_ID_ESC_COMBINED_1) {
        currentMeterESCReadCombined(meter);
    }
    else if (id >= CURRENT_METER_ID_ESC_MOTOR_1 && id <= CURRENT_METER_ID_ESC_MOTOR_20 ) {
        int motor = id - CURRENT_METER_ID_ESC_MOTOR_1;
        currentMeterESCReadMotor(motor, meter);
    }
#endif
    else {
        currentMeterReset(meter);
    }
}
