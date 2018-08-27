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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#if defined(USE_HOTT_TEXTMODE) && defined(USE_CMS)

// Fixme:
// Let the CMS believe we have a bigger display to avoid empty rows and columns
// Better make this configurable in CMS
#define ROW_CORRECTION 1
#define COL_CORRECTION 2

#include "common/utils.h"
#include "cms/cms.h"
#include "telemetry/hott.h"

displayPort_t hottDisplayPort;

static int hottDrawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int hottScreenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static int hottWriteChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t c)
{
    UNUSED(displayPort);

    if (row < ROW_CORRECTION || row >= HOTT_TEXTMODE_DISPLAY_ROWS + ROW_CORRECTION || col < COL_CORRECTION || col >= HOTT_TEXTMODE_DISPLAY_COLUMNS + COL_CORRECTION) {
        return 0;
    }
    row -= ROW_CORRECTION;
    col -= COL_CORRECTION;

    hottTextmodeWriteChar(col, row, c);
    return 0;
}

static int hottWriteString(displayPort_t *displayPort, uint8_t col, uint8_t row, const char *s)
{
    while (*s) {
        hottWriteChar(displayPort, col++, row, *(s++));
    }
    return 0;
}

static int hottClearScreen(displayPort_t *displayPort)
{
    for (int row = 0; row < displayPort->rows; row++) {
        for (int col = 0; col < displayPort->cols; col++) {
            hottWriteChar(displayPort, col, row, ' ');
        }
    }
    return 0;
}

static bool hottIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static int hottHeartbeat(displayPort_t *displayPort)
{
    if (!hottTextmodeIsAlive()) {
        cmsMenuExit(displayPort, (void *)CMS_EXIT_SAVE);
    }

    return 0;
}

static void hottResync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
}

static uint32_t hottTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static int hottGrab(displayPort_t *displayPort)
{
    hottTextmodeGrab();
    return displayPort->grabCount = 1;
}

static int hottRelease(displayPort_t *displayPort)
{
    int cnt = displayPort->grabCount = 0;
    hottClearScreen(displayPort);
    hottTextmodeExit();
    return cnt;
}

static const displayPortVTable_t hottVTable = {
    .grab = hottGrab,
    .release = hottRelease,
    .clearScreen = hottClearScreen,
    .drawScreen = hottDrawScreen,
    .screenSize = hottScreenSize,
    .writeString = hottWriteString,
    .writeChar = hottWriteChar,
    .isTransferInProgress = hottIsTransferInProgress,
    .heartbeat = hottHeartbeat,
    .resync = hottResync,
    .txBytesFree = hottTxBytesFree};

displayPort_t *displayPortHottInit()
{
    hottDisplayPort.device = NULL;
    displayInit(&hottDisplayPort, &hottVTable);
    hottDisplayPort.rows = HOTT_TEXTMODE_DISPLAY_ROWS + ROW_CORRECTION * 2;
    hottDisplayPort.cols = HOTT_TEXTMODE_DISPLAY_COLUMNS + COL_CORRECTION * 2;
    return &hottDisplayPort;
}

void hottDisplayportRegister()
{
    cmsDisplayPortRegister(displayPortHottInit());
}

void hottCmsOpen()
{
    if (!cmsInMenu) {
        cmsDisplayPortSelect(&hottDisplayPort);
        cmsMenuOpen();
    }
}

void hottSetCmsKey(uint8_t hottKey, bool keepCmsOpen)
{
    switch (hottKey) {
    case HOTTV4_BUTTON_DEC:
        cmsSetExternKey(CMS_KEY_UP);
        break;
    case HOTTV4_BUTTON_INC:
        cmsSetExternKey(CMS_KEY_DOWN);
        break;
    case HOTTV4_BUTTON_SET:
        if (cmsInMenu) {
            cmsMenuExit(pCurrentDisplay, (void *)CMS_EXIT_SAVE);
        }
        cmsSetExternKey(CMS_KEY_NONE);
        break;
    case HOTTV4_BUTTON_NEXT:
        cmsSetExternKey(CMS_KEY_RIGHT);
        break;
    case HOTTV4_BUTTON_PREV:
        cmsSetExternKey(CMS_KEY_LEFT);
        if (keepCmsOpen) { // Make sure CMS is open until textmode is closed.
            cmsMenuOpen();
        }
        break;
    default:
        cmsSetExternKey(CMS_KEY_NONE);
        break;
    }
}

#endif
