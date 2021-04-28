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
#include <string.h>

#include "platform.h"

#if defined(USE_CRSF_CMS_TELEMETRY)

#include "cms/cms.h"

#include "common/maths.h"
#include "common/printf.h"
#include "common/time.h"

#include "drivers/display.h"
#include "drivers/time.h"

#include "displayport_crsf.h"

#define CRSF_DISPLAY_PORT_OPEN_DELAY_MS     400
#define CRSF_DISPLAY_PORT_CLEAR_DELAY_MS    45

static crsfDisplayPortScreen_t crsfScreen;
static timeMs_t delayTransportUntilMs = 0;

displayPort_t crsfDisplayPort;

static int crsfGrab(displayPort_t *displayPort)
{
    return displayPort->grabCount = 1;
}

static int crsfClearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    memset(crsfScreen.buffer, ' ', sizeof(crsfScreen.buffer));
    crsfScreen.updated = false;
    crsfScreen.reset = true;
    delayTransportUntilMs = millis() + CRSF_DISPLAY_PORT_CLEAR_DELAY_MS;
    return 0;
}

static int crsfRelease(displayPort_t *displayPort)
{
    displayPort->grabCount = 0;
    return crsfClearScreen(displayPort);
}

static int crsfDrawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int crsfScreenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}


static int crsfWriteString(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, const char *s)
{
    UNUSED(displayPort);
    UNUSED(attr);
    if (row >= crsfScreen.rows || col >= crsfScreen.cols) {
        return 0;
    }
    const size_t truncLen = MIN((int)strlen(s), crsfScreen.cols-col);  // truncate at colCount
    char *rowStart = &crsfScreen.buffer[row * crsfScreen.cols + col];
    crsfScreen.updated |= memcmp(rowStart, s, truncLen);
    if (crsfScreen.updated) {
        memcpy(rowStart, s, truncLen);
    }
    return 0;
}

static int crsfWriteChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, uint8_t c)
{
    char s[1];
    tfp_sprintf(s, "%c", c);
    return crsfWriteString(displayPort, col, row, attr, s);
}

static bool crsfIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static bool crsfIsSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static int crsfHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static void crsfRedraw(displayPort_t *displayPort)
{
    displayPort->rows = crsfScreen.rows;
    displayPort->cols = crsfScreen.cols;
}

static uint32_t crsfTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static const displayPortVTable_t crsfDisplayPortVTable = {
    .grab = crsfGrab,
    .release = crsfRelease,
    .clearScreen = crsfClearScreen,
    .drawScreen = crsfDrawScreen,
    .screenSize = crsfScreenSize,
    .writeString = crsfWriteString,
    .writeChar = crsfWriteChar,
    .isTransferInProgress = crsfIsTransferInProgress,
    .heartbeat = crsfHeartbeat,
    .redraw = crsfRedraw,
    .isSynced = crsfIsSynced,
    .txBytesFree = crsfTxBytesFree,
    .layerSupported = NULL,
    .layerSelect = NULL,
    .layerCopy = NULL,
};

crsfDisplayPortScreen_t *crsfDisplayPortScreen(void)
{
    return &crsfScreen;
}

void crsfDisplayPortMenuOpen(void)
{
    if (cmsInMenu) {
        return;
    }
    if (cmsDisplayPortSelect(&crsfDisplayPort)) {
        cmsMenuOpen();
        delayTransportUntilMs = millis() + CRSF_DISPLAY_PORT_OPEN_DELAY_MS;
    }
}

void crsfDisplayPortMenuExit(void)
{
    if (!cmsInMenu) {
        return;
    }
    uint8_t exitMenu = CMS_EXIT;
    cmsMenuExit(&crsfDisplayPort, &exitMenu);
}

void crsfDisplayPortSetDimensions(uint8_t rows, uint8_t cols)
{
    crsfScreen.rows = MIN(rows, CRSF_DISPLAY_PORT_ROWS_MAX);
    crsfScreen.cols = MIN(cols, CRSF_DISPLAY_PORT_COLS_MAX);
    crsfRedraw(&crsfDisplayPort);
}

void crsfDisplayPortRefresh(void)
{
    if (!cmsInMenu) {
        crsfDisplayPortMenuOpen();
        return;
    }
    crsfScreen.updated = true;
    crsfScreen.reset = true;
    delayTransportUntilMs = millis() + CRSF_DISPLAY_PORT_CLEAR_DELAY_MS;
}

bool crsfDisplayPortIsReady(void)
{
    const timeMs_t currentTimeMs = millis();
    const bool delayExpired = (currentTimeMs > delayTransportUntilMs);
    const bool cmsReady = (cmsInMenu && (pCurrentDisplay == &crsfDisplayPort));
    return (bool)(delayExpired && cmsReady);
}

static displayPort_t *displayPortCrsfInit()
{
    crsfDisplayPortSetDimensions(CRSF_DISPLAY_PORT_ROWS_MAX, CRSF_DISPLAY_PORT_COLS_MAX);
    displayInit(&crsfDisplayPort, &crsfDisplayPortVTable, DISPLAYPORT_DEVICE_TYPE_CRSF);

    return &crsfDisplayPort;
}

void crsfDisplayportRegister(void)
{
    cmsDisplayPortRegister(displayPortCrsfInit());
}
#endif
