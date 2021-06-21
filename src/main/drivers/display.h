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

typedef enum {
    DISPLAYPORT_DEVICE_TYPE_MAX7456 = 0,
    DISPLAYPORT_DEVICE_TYPE_OLED,
    DISPLAYPORT_DEVICE_TYPE_MSP,
    DISPLAYPORT_DEVICE_TYPE_FRSKYOSD,
    DISPLAYPORT_DEVICE_TYPE_CRSF,
    DISPLAYPORT_DEVICE_TYPE_HOTT,
    DISPLAYPORT_DEVICE_TYPE_SRXL,
} displayPortDeviceType_e;

typedef enum {
    DISPLAYPORT_ATTR_NONE = 0,
    DISPLAYPORT_ATTR_INFO,
    DISPLAYPORT_ATTR_WARNING,
    DISPLAYPORT_ATTR_CRITICAL,
} displayPortAttr_e;

#define DISPLAYPORT_ATTR_BLINK  0x80 // Device local blink bit or'ed into displayPortAttr_e

typedef enum {
    DISPLAYPORT_LAYER_FOREGROUND,
    DISPLAYPORT_LAYER_BACKGROUND,
    DISPLAYPORT_LAYER_COUNT,
} displayPortLayer_e;

typedef enum {
    DISPLAY_TRANSACTION_OPT_NONE = 0,
    DISPLAY_TRANSACTION_OPT_PROFILED = 1 << 0,
    DISPLAY_TRANSACTION_OPT_RESET_DRAWING = 1 << 1,
} displayTransactionOption_e;

typedef enum {
    DISPLAY_BACKGROUND_TRANSPARENT,
    DISPLAY_BACKGROUND_BLACK,
    DISPLAY_BACKGROUND_GRAY,
    DISPLAY_BACKGROUND_LTGRAY,
    DISPLAY_BACKGROUND_COUNT    // must be the last entry
} displayPortBackground_e;

struct displayCanvas_s;
struct osdCharacter_s;
struct displayPortVTable_s;

typedef struct displayPort_s {
    const struct displayPortVTable_s *vTable;
    void *device;
    uint8_t rows;
    uint8_t cols;
    uint8_t posX;
    uint8_t posY;

    // CMS state
    bool useFullscreen;
    bool cleared;
    int8_t cursorRow;
    int8_t grabCount;

    // Displayport device capability
    bool useDeviceBlink;

    // The type of display device
    displayPortDeviceType_e deviceType;
} displayPort_t;

typedef struct displayPortVTable_s {
    int (*grab)(displayPort_t *displayPort);
    int (*release)(displayPort_t *displayPort);
    int (*clearScreen)(displayPort_t *displayPort);
    int (*drawScreen)(displayPort_t *displayPort);
    int (*screenSize)(const displayPort_t *displayPort);
    int (*writeString)(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *text);
    int (*writeChar)(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c);
    bool (*isTransferInProgress)(const displayPort_t *displayPort);
    int (*heartbeat)(displayPort_t *displayPort);
    void (*redraw)(displayPort_t *displayPort);
    bool (*isSynced)(const displayPort_t *displayPort);
    uint32_t (*txBytesFree)(const displayPort_t *displayPort);
    bool (*layerSupported)(displayPort_t *displayPort, displayPortLayer_e layer);
    bool (*layerSelect)(displayPort_t *displayPort, displayPortLayer_e layer);
    bool (*layerCopy)(displayPort_t *displayPort, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
    bool (*writeFontCharacter)(displayPort_t *instance, uint16_t addr, const struct osdCharacter_s *chr);
    bool (*checkReady)(displayPort_t *displayPort, bool rescan);
    void (*beginTransaction)(displayPort_t *displayPort, displayTransactionOption_e opts);
    void (*commitTransaction)(displayPort_t *displayPort);
    bool (*getCanvas)(struct displayCanvas_s *canvas, const displayPort_t *displayPort);
    void (*setBackgroundType)(displayPort_t *displayPort, displayPortBackground_e backgroundType);
} displayPortVTable_t;

void displayGrab(displayPort_t *instance);
void displayRelease(displayPort_t *instance);
void displayReleaseAll(displayPort_t *instance);
bool displayIsGrabbed(const displayPort_t *instance);
void displayClearScreen(displayPort_t *instance);
int displayDrawScreen(displayPort_t *instance);
int displayScreenSize(const displayPort_t *instance);
void displaySetXY(displayPort_t *instance, uint8_t x, uint8_t y);
int displayWrite(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, const char *s);
int displayWriteChar(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, uint8_t c);
bool displayIsTransferInProgress(const displayPort_t *instance);
bool displayHeartbeat(displayPort_t *instance);
void displayRedraw(displayPort_t *instance);
bool displayIsSynced(const displayPort_t *instance);
uint16_t displayTxBytesFree(const displayPort_t *instance);
bool displayWriteFontCharacter(displayPort_t *instance, uint16_t addr, const struct osdCharacter_s *chr);
bool displayCheckReady(displayPort_t *instance, bool rescan);
void displayBeginTransaction(displayPort_t *instance, displayTransactionOption_e opts);
void displayCommitTransaction(displayPort_t *instance);
bool displayGetCanvas(struct displayCanvas_s *canvas, const displayPort_t *instance);
void displayInit(displayPort_t *instance, const displayPortVTable_t *vTable, displayPortDeviceType_e deviceType);
bool displayLayerSupported(displayPort_t *instance, displayPortLayer_e layer);
bool displayLayerSelect(displayPort_t *instance, displayPortLayer_e layer);
bool displayLayerCopy(displayPort_t *instance, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
void displaySetBackgroundType(displayPort_t *instance, displayPortBackground_e backgroundType);
bool displaySupportsOsdSymbols(displayPort_t *instance);
