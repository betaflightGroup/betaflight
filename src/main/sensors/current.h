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

#include "common/time.h"
#include "current_ids.h"

typedef enum {
    CURRENT_METER_NONE = 0,
    CURRENT_METER_ADC,
    CURRENT_METER_VIRTUAL,
    CURRENT_METER_ESC,
    CURRENT_METER_MSP,
    CURRENT_METER_COUNT
} currentMeterSource_e;

extern const char * const currentMeterSourceNames[CURRENT_METER_COUNT];

typedef struct currentMeter_s {
    int32_t amperage;           // current read by current sensor in centiampere (1/100th A)
    int32_t amperageLatest;     // current read by current sensor in centiampere (1/100th A) (unfiltered)
    int32_t mAhDrawn;           // milliampere hours drawn from the battery since start
} currentMeter_t;

// WARNING - do not mix usage of CURRENT_SENSOR_* and CURRENT_METER_*, they are separate concerns.

typedef struct currentMeterMAhDrawnState_s {
    int32_t mAhDrawn;           // milliampere hours drawn from the battery since start
    float mAhDrawnF;
} currentMeterMAhDrawnState_t;

//
// Sensors
//

typedef enum {
    CURRENT_SENSOR_VIRTUAL = 0,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_ESC,
    CURRENT_SENSOR_MSP
} currentSensor_e;


//
// ADC
//

typedef struct currentMeterADCState_s {
    currentMeterMAhDrawnState_t mahDrawnState;
    int32_t amperage;           // current read by current sensor in centiampere (1/100th A)
    int32_t amperageLatest;     // current read by current sensor in centiampere (1/100th A) (unfiltered)
} currentMeterADCState_t;

typedef struct currentSensorADCConfig_s {
    int16_t scale;              // scale the current sensor output voltage to milliamps. Value in mV/10A
    int16_t offset;             // offset of the current sensor in mA
} currentSensorADCConfig_t;

PG_DECLARE(currentSensorADCConfig_t, currentSensorADCConfig);

//
// Virtual
//

typedef struct currentMeterVirtualState_s {
    currentMeterMAhDrawnState_t mahDrawnState;
    int32_t amperage;           // current read by current sensor in centiamperes (1/100th A)
} currentSensorVirtualState_t;

typedef struct currentSensorVirtualConfig_s {
    int16_t scale;              // scale the throttle to centiamperes, using a hardcoded thrust linearization function (see Battery.md)
    uint16_t offset;            // offset of the current sensor in centiamperes (1/100th A)
} currentSensorVirtualConfig_t;

PG_DECLARE(currentSensorVirtualConfig_t, currentSensorVirtualConfig);

//
// ESC
//

typedef struct currentMeterESCState_s {
    int32_t mAhDrawn;           // milliampere hours drawn from the battery since start
    int32_t amperage;           // current read by current sensor in centiampere (1/100th A)
} currentMeterESCState_t;


//
// MSP
//

typedef struct currentMeterMSPState_s {
    int32_t mAhDrawn;           // milliampere hours drawn from the battery since start
    int32_t amperage;           // current read by current sensor in centiampere (1/100th A)
} currentMeterMSPState_t;


//
// Current Meter API
//

void currentMeterReset(currentMeter_t *meter);

void currentMeterADCInit(void);
void currentMeterADCRefresh(int32_t lastUpdateAt);
void currentMeterADCRead(currentMeter_t *meter);

void currentMeterVirtualInit(void);
void currentMeterVirtualRefresh(int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset);
void currentMeterVirtualRead(currentMeter_t *meter);

void currentMeterESCInit(void);
void currentMeterESCRefresh(int32_t lastUpdateAt);
void currentMeterESCReadCombined(currentMeter_t *meter);
void currentMeterESCReadMotor(uint8_t motorNumber, currentMeter_t *meter);

void currentMeterMSPInit(void);
void currentMeterMSPRefresh(timeUs_t currentTimeUs);
void currentMeterMSPRead(currentMeter_t *meter);
void currentMeterMSPSet(uint16_t amperage, uint16_t mAhDrawn);

//
// API for reading current meters by id.
//
extern const uint8_t supportedCurrentMeterCount;
extern const uint8_t currentMeterIds[];
void currentMeterRead(currentMeterId_e id, currentMeter_t *currentMeter);
