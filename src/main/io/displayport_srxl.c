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

#include "platform.h"

#if defined(USE_SPEKTRUM_CMS_TELEMETRY)

#include "cms/cms.h"

#include "common/utils.h"

#include "drivers/display.h"

#include "telemetry/srxl.h"

#include "displayport_srxl.h"

displayPort_t srxlDisplayPort;

static int srxlDrawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int srxlScreenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static int srxlWriteChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, uint8_t c)
{
    UNUSED(displayPort);
    UNUSED(attr);

    return (spektrumTmTextGenPutChar(col, row, c));
}


static int srxlWriteString(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, const char *s)
{
    while (*s) {
        srxlWriteChar(displayPort, col++, row, attr, *(s++));
    }
    return 0;
}

static int srxlClearScreen(displayPort_t *displayPort)
{
    for (int row = 0; row < SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS; row++) {
        for (int col= 0; col < SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS; col++) {
            srxlWriteChar(displayPort, col, row, DISPLAYPORT_ATTR_NONE, ' ');
        }
    }
    srxlWriteString(displayPort, 1, 0, DISPLAYPORT_ATTR_NONE, "BETAFLIGHT");

    if (displayPort->grabCount == 0) {
        srxlWriteString(displayPort, 0, 2, DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT1);
        srxlWriteString(displayPort, 2, 3, DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT2);
        srxlWriteString(displayPort, 2, 4, DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT3);
    }
    return 0;
}

static bool srxlIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static bool srxlIsSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static int srxlHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static void srxlRedraw(displayPort_t *displayPort)
{
    UNUSED(displayPort);
}

static uint32_t srxlTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static int srxlGrab(displayPort_t *displayPort)
{
    return displayPort->grabCount = 1;
}

static int srxlRelease(displayPort_t *displayPort)
{
    int cnt = displayPort->grabCount = 0;
    srxlClearScreen(displayPort);
    return cnt;
}

static const displayPortVTable_t srxlVTable = {
    .grab = srxlGrab,
    .release = srxlRelease,
    .clearScreen = srxlClearScreen,
    .drawScreen = srxlDrawScreen,
    .screenSize = srxlScreenSize,
    .writeString = srxlWriteString,
    .writeChar = srxlWriteChar,
    .isTransferInProgress = srxlIsTransferInProgress,
    .heartbeat = srxlHeartbeat,
    .redraw = srxlRedraw,
    .isSynced = srxlIsSynced,
    .txBytesFree = srxlTxBytesFree,
    .layerSupported = NULL,
    .layerSelect = NULL,
    .layerCopy = NULL,
};

static displayPort_t *displayPortSrxlInit()
{
    srxlDisplayPort.device = NULL;
    displayInit(&srxlDisplayPort, &srxlVTable, DISPLAYPORT_DEVICE_TYPE_SRXL);
    srxlDisplayPort.rows = SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS;
    srxlDisplayPort.cols = SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS;

    return &srxlDisplayPort;
}

void srxlDisplayportRegister(void)
{
    cmsDisplayPortRegister(displayPortSrxlInit());
}
#endif
