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

#include "common/utils.h"

#include "drivers/display_canvas.h"
#include "drivers/osd.h"

#include "display.h"

void displayClearScreen(displayPort_t *instance, displayClearOption_e options)
{
    instance->vTable->clearScreen(instance, options);
    instance->cleared = true;
    instance->cursorRow = -1;
}

void displayDrawScreen(displayPort_t *instance)
{
    instance->vTable->drawScreen(instance);
}

int displayScreenSize(const displayPort_t *instance)
{
    return instance->vTable->screenSize(instance);
}

void displayGrab(displayPort_t *instance)
{
    instance->vTable->grab(instance);
    instance->vTable->clearScreen(instance, DISPLAY_CLEAR_WAIT);
    ++instance->grabCount;
}

void displayRelease(displayPort_t *instance)
{
    instance->vTable->release(instance);
    --instance->grabCount;
}

void displayReleaseAll(displayPort_t *instance)
{
    instance->vTable->release(instance);
    instance->grabCount = 0;
}

bool displayIsGrabbed(const displayPort_t *instance)
{
    // can be called before initialised
    return (instance && instance->grabCount > 0);
}

void displaySetXY(displayPort_t *instance, uint8_t x, uint8_t y)
{
    instance->posX = x;
    instance->posY = y;
}

int displayWrite(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, const char *text)
{
    instance->posX = x + strlen(text);
    instance->posY = y;
    return instance->vTable->writeString(instance, x, y, attr, text);
}

int displayWriteChar(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    instance->posX = x + 1;
    instance->posY = y;
    return instance->vTable->writeChar(instance, x, y, attr, c);
}

bool displayIsTransferInProgress(const displayPort_t *instance)
{
    return instance->vTable->isTransferInProgress(instance);
}

bool displayIsSynced(const displayPort_t *instance)
{
    return instance->vTable->isSynced(instance);
}

void displayHeartbeat(displayPort_t *instance)
{
    instance->vTable->heartbeat(instance);
}

void displayRedraw(displayPort_t *instance)
{
    instance->vTable->redraw(instance);
}

uint16_t displayTxBytesFree(const displayPort_t *instance)
{
    return instance->vTable->txBytesFree(instance);
}

bool displayLayerSupported(displayPort_t *instance, displayPortLayer_e layer)
{
    if (layer == DISPLAYPORT_LAYER_FOREGROUND) {
        // Every device must support the foreground (default) layer
        return true;
    } else if (layer < DISPLAYPORT_LAYER_COUNT && instance->vTable->layerSupported) {
        return instance->vTable->layerSupported(instance, layer);
    }
    return false;
}

bool displayLayerSelect(displayPort_t *instance, displayPortLayer_e layer)
{
    if (instance->vTable->layerSelect) {
        return instance->vTable->layerSelect(instance, layer);
    }
    return false;
}

bool displayLayerCopy(displayPort_t *instance, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    if (instance->vTable->layerCopy && sourceLayer != destLayer) {
        return instance->vTable->layerCopy(instance, destLayer, sourceLayer);
    }
    return false;
}

bool displayWriteFontCharacter(displayPort_t *instance, uint16_t addr, const osdCharacter_t *chr)
{
    if (instance->vTable->writeFontCharacter) {
        return instance->vTable->writeFontCharacter(instance, addr, chr);
    }
    return false;
}

void displaySetBackgroundType(displayPort_t *instance, displayPortBackground_e backgroundType)
{
    if (instance->vTable->setBackgroundType) {
        instance->vTable->setBackgroundType(instance, backgroundType);
    }
}

bool displayCheckReady(displayPort_t *instance, bool rescan)
{
    if (instance->vTable->checkReady) {
        return instance->vTable->checkReady(instance, rescan);
    }
    // Drivers that don't provide a checkReady method are
    // assumed to be immediately ready (either by actually
    // begin ready very quickly or by blocking)
    return true;
}

void displayBeginTransaction(displayPort_t *instance, displayTransactionOption_e opts)
{
    if (instance->vTable->beginTransaction) {
        instance->vTable->beginTransaction(instance, opts);
    }
}

void displayCommitTransaction(displayPort_t *instance)
{
    if (instance->vTable->commitTransaction) {
        instance->vTable->commitTransaction(instance);
    }
}

bool displayGetCanvas(displayCanvas_t *canvas, const displayPort_t *instance)
{
#if defined(USE_CANVAS)
    if (canvas && instance->vTable->getCanvas && instance->vTable->getCanvas(canvas, instance)) {
        canvas->gridElementWidth = canvas->width / instance->cols;
        canvas->gridElementHeight = canvas->height / instance->rows;
        return true;
    }
#else
    UNUSED(canvas);
    UNUSED(instance);
#endif
    return false;
}

bool displaySupportsOsdSymbols(displayPort_t *instance)
{
    // Assume device types that support OSD display will support the OSD symbols (since the OSD logic will use them)
    if ((instance->deviceType == DISPLAYPORT_DEVICE_TYPE_MAX7456)
        || (instance->deviceType == DISPLAYPORT_DEVICE_TYPE_MSP)
        || (instance->deviceType == DISPLAYPORT_DEVICE_TYPE_SPRACINGPIXELOSD)
        || (instance->deviceType == DISPLAYPORT_DEVICE_TYPE_FRSKYOSD)) {
        return true;
    } else {
        return false;
    }
}

void displayInit(displayPort_t *instance, const displayPortVTable_t *vTable, displayPortDeviceType_e deviceType)
{
    instance->vTable = vTable;

    instance->useFullscreen = false;
    instance->cleared = false;
    instance->grabCount = 0;
    instance->cursorRow = -1;
    instance->deviceType = deviceType;

    displayBeginTransaction(instance, DISPLAY_TRANSACTION_OPT_NONE);
    displayClearScreen(instance, DISPLAY_CLEAR_WAIT);
    displayCommitTransaction(instance);

}
