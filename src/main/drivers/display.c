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

#include "display.h"

void displayClearScreen(displayPort_t *instance)
{
    instance->vTable->clearScreen(instance);
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
    instance->vTable->clearScreen(instance);
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

int displayWrite(displayPort_t *instance, uint8_t x, uint8_t y, const char *s)
{
    instance->posX = x + strlen(s);
    instance->posY = y;
    return instance->vTable->writeString(instance, x, y, s);
}

int displayWriteChar(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t c)
{
    instance->posX = x + 1;
    instance->posY = y;
    return instance->vTable->writeChar(instance, x, y, c);
}

#ifdef USE_MAX7456_EXTENDED
int displayWriteExtended(displayPort_t *instance, uint8_t x, uint8_t y, const char *s)
{
    instance->posX = x + strlen(s);
    instance->posY = y;
    return instance->vTable->writeExtended(instance, x, y, s);
}
int displayWriteCharExtended(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t c)
{
    instance->posX = x + 1;
    instance->posY = y;
    return instance->vTable->writeCharExtended(instance, x, y, c);
}
bool displayIsExtended(const displayPort_t *instance)
{
    return instance->vTable->isExtended();
}
#endif

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

void displayResync(displayPort_t *instance)
{
    instance->vTable->resync(instance);
}

uint16_t displayTxBytesFree(const displayPort_t *instance)
{
    return instance->vTable->txBytesFree(instance);
}

void displayInit(displayPort_t *instance, const displayPortVTable_t *vTable)
{
    instance->vTable = vTable;
    instance->vTable->clearScreen(instance);
    instance->useFullscreen = false;
    instance->cleared = true;
    instance->grabCount = 0;
    instance->cursorRow = -1;
}
