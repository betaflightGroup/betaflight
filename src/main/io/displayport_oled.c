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

#include "platform.h"

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/display_ug2864hsweg01.h"

static displayPort_t oledDisplayPort;

static int oledGrab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int oledRelease(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int oledClearScreen(displayPort_t *displayPort, displayClearOption_e options)
{
    UNUSED(options);

    i2c_OLED_clear_display_quick(displayPort->device);
    return 0;
}

static int oledDrawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int oledScreenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static int oledWriteString(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    UNUSED(attr);

    i2c_OLED_set_xy(displayPort->device, x, y);
    i2c_OLED_send_string(displayPort->device, s);
    return 0;
}

static int oledWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    UNUSED(attr);

    i2c_OLED_set_xy(displayPort->device, x, y);
    i2c_OLED_send_char(displayPort->device, c);
    return 0;
}

static bool oledIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static bool oledIsSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static int oledHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static void oledRedraw(displayPort_t *displayPort)
{
    UNUSED(displayPort);
}

static uint32_t oledTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static const displayPortVTable_t oledVTable = {
    .grab = oledGrab,
    .release = oledRelease,
    .clearScreen = oledClearScreen,
    .drawScreen = oledDrawScreen,
    .screenSize = oledScreenSize,
    .writeString = oledWriteString,
    .writeChar = oledWriteChar,
    .isTransferInProgress = oledIsTransferInProgress,
    .heartbeat = oledHeartbeat,
    .redraw = oledRedraw,
    .isSynced = oledIsSynced,
    .txBytesFree = oledTxBytesFree,
    .layerSupported = NULL,
    .layerSelect = NULL,
    .layerCopy = NULL,
};

displayPort_t *displayPortOledInit(void *device)
{
    oledDisplayPort.device = device;
    displayInit(&oledDisplayPort, &oledVTable, DISPLAYPORT_DEVICE_TYPE_OLED);
    oledDisplayPort.rows = SCREEN_CHARACTER_ROW_COUNT;
    oledDisplayPort.cols = SCREEN_CHARACTER_COLUMN_COUNT;
    return &oledDisplayPort;
}
