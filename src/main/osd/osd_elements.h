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

#include "drivers/display.h"

#include "osd/osd.h"

typedef enum {
    OSD_ELEMENT_TYPE_1 = 0,
    OSD_ELEMENT_TYPE_2,
    OSD_ELEMENT_TYPE_3,
    OSD_ELEMENT_TYPE_4
} osdElementType_e;

typedef struct osdElementParms_s {
    uint8_t item;
    uint8_t elemPosX;
    uint8_t elemPosY;
    osdElementType_e type;
    char *buff;
    displayPort_t *osdDisplayPort;
    bool drawElement;
    uint8_t attr;
} osdElementParms_t;

typedef void (*osdElementDrawFn)(osdElementParms_t *element);

int osdConvertTemperatureToSelectedUnit(int tempInDegreesCelcius);
void osdFormatDistanceString(char *result, int distance, char leadingSymbol);
bool osdFormatRtcDateTime(char *buffer);
void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time);
void osdFormatTimer(char *buff, bool showSymbol, bool usePrecision, int timerIndex);
float osdGetMetersToSelectedUnit(int32_t meters);
char osdGetMetersToSelectedUnitSymbol(void);
int32_t osdGetSpeedToSelectedUnit(int32_t value);
char osdGetSpeedToSelectedUnitSymbol(void);
char osdGetTemperatureSymbolForSelectedUnit(void);
void osdAddActiveElements(void);
uint8_t osdGetActiveElement();
uint8_t osdGetActiveElementCount();
bool osdDrawNextActiveElement(displayPort_t *osdDisplayPort, timeUs_t currentTimeUs);
void osdDrawActiveElementsBackground(displayPort_t *osdDisplayPort);
void osdElementsInit(bool backgroundLayerFlag);
void osdSyncBlink();
void osdResetAlarms(void);
void osdUpdateAlarms(void);
bool osdElementsNeedAccelerometer(void);
