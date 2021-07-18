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

/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working Pixel OSD system.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPRACING_PIXEL_OSD

#include "common/utils.h"

#include "build/debug.h"

#include "drivers/display.h"
#include "drivers/display_canvas.h"
#include "drivers/spracingpixelosd/spracing_pixel_osd.h"
#include "drivers/spracingpixelosd/spracing_pixel_osd_library.h"
#include "drivers/spracingpixelosd/framebuffer.h"
#include "drivers/spracingpixelosd/framebuffer_canvas.h"
#include "drivers/osd.h"
#include "drivers/osd/font_max7456_12x18.h"

#include "osd/osd.h"

#include "pg/spracing_pixel_osd.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "io/displayport_spracing_pixel_osd.h"


bool pixelOSDInitialised = false;

displayPort_t spracingPixelOSDDisplayPort;
uint8_t *frameBuffer = NULL;

PG_REGISTER_WITH_RESET_FN(displayPortProfile_t, displayPortProfileSPRacingPixelOSD, PG_DISPLAY_PORT_SPRACING_PIXEL_OSD_CONFIG, 0);

void pgResetFn_displayPortProfileSPRacingPixelOSD(displayPortProfile_t *displayPortProfile)
{
    displayPortProfile->colAdjust = 0;
    displayPortProfile->rowAdjust = 0;

    displayPortProfile->invert = false;         // UNSUPPORTED
    displayPortProfile->blackBrightness = 0;    // UNSUPPORTED
    displayPortProfile->whiteBrightness = 0;    // UNSUPPORTED
}

static int grab(displayPort_t *displayPort)
{
    // FIXME this should probably not have a dependency on the OSD or OSD slave code
    UNUSED(displayPort);
#ifdef USE_OSD
    resumeRefreshAt = 0;
#endif

    return 0;
}

static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    return 0;
}

static int clearScreen(displayPort_t *displayPort, displayClearOption_e options)
{
    UNUSED(displayPort);

    frameBuffer_eraseBegin(frameBuffer);

    if (options & DISPLAY_CLEAR_WAIT) {
        // clearing the framebuffer is an expensive operation and handled in hardware asynchronously, see the framebuffer API.
        // scheduling the clearing of the screen prior to needing to actually render to the screen is recommended.

        frameBuffer_eraseWaitForComplete();
    }

    return 0;
}

static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return (16 * 30); // TODO
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    UNUSED(displayPort);
    UNUSED(attr);

    frameBuffer_slowWriteString(frameBuffer, x * FONT_MAX7456_WIDTH, y * FONT_MAX7456_HEIGHT, (uint8_t *)s, strlen(s));

    return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    UNUSED(displayPort);
    UNUSED(attr);

    frameBuffer_slowWriteCharacter(frameBuffer, x * FONT_MAX7456_WIDTH, y * FONT_MAX7456_HEIGHT, c);

    return 0;
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);

    return spracingPixelOSDIsFrameRenderingComplete();
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);

    // TODO
    return true;
}

static void redraw(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    displayPort->rows = 16; // FIXME hardcoded to PAL.
    displayPort->cols = 30;

    // TODO
}

static int heartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

//
// Layer support
//
// Foreground only for now.

static bool layerSupported(displayPort_t *displayPort, displayPortLayer_e layer)
{
    UNUSED(displayPort);
    if (layer == DISPLAYPORT_LAYER_FOREGROUND) {
        return true;
    } else {
        return false;
    }
}

static bool layerSelect(displayPort_t *displayPort, displayPortLayer_e layer)
{
    UNUSED(displayPort);
    if (layerSupported(displayPort, layer)) {
        return true;
    } else {
        return false;
    }
}

static bool layerCopy(displayPort_t *displayPort, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    UNUSED(displayPort);
    UNUSED(sourceLayer);
    UNUSED(destLayer);
    return false;
}

static bool writeFontCharacter(displayPort_t *instance, uint16_t addr, const osdCharacter_t *chr)
{
    UNUSED(instance);
    UNUSED(chr);
    UNUSED(addr);

    return false;
}

static void drawCharacter(displayCanvas_t *displayCanvas, int x, int y, uint16_t chr, displayCanvasBitmapOption_t opts)
{
    UNUSED(displayCanvas);
    UNUSED(opts); // NOT CURRENTLY SUPPORTED

    frameBuffer_slowWriteCharacter(frameBuffer, x, y, chr & 0xFF);
}

static void drawString(displayCanvas_t *displayCanvas, int x, int y, const char *s, displayCanvasBitmapOption_t opts)
{
    UNUSED(displayCanvas);
    UNUSED(opts); // NOT CURRENTLY SUPPORTED

    frameBuffer_slowWriteString(frameBuffer, x, y, (uint8_t *)s, strlen(s));
}

static const displayCanvasVTable_t spracingPixelOsdCanvasVTable = {
    .setStrokeColor = frameBufferCanvasSetStrokeColor,
//    .setFillColor = setFillColor,
//    .setStrokeAndFillColor = setStrokeAndFillColor,
//    .setColorInversion = setColorInversion,
//    .setPixel = setPixel,
//    .setPixelToStrokeColor = setPixelToStrokeColor,
//    .setPixelToFillColor = setPixelToFillColor,
//    .setStrokeWidth = setStrokeWidth,
//    .setLineOutlineType = setLineOutlineType,
//    .setLineOutlineColor = setLineOutlineColor,
//
//    .clipToRect = clipToRect,
//    .clearRect = clearRect,
//    .resetDrawingState = resetDrawingState,
    .drawCharacter = drawCharacter,
//    .drawCharacterMask = drawCharacterMask,
    .drawString = drawString,
//    .drawStringMask = drawStringMask,
    .moveToPoint = frameBufferCanvasMoveToPoint,
    .strokeLineToPoint = frameBufferCanvasStrokeLineToPoint,
//    .strokeTriangle = strokeTriangle,
//    .fillTriangle = fillTriangle,
//    .fillStrokeTriangle = fillStrokeTriangle,
//    .strokeRect = strokeRect,
//    .fillRect = fillRect,
//    .fillStrokeRect = fillStrokeRect,
//    .strokeEllipseInRect = strokeEllipseInRect,
//    .fillEllipseInRect = fillEllipseInRect,
//    .fillStrokeEllipseInRect = fillStrokeEllipseInRect,
//
//    .ctmReset = ctmReset,
//    .ctmSet = ctmSet,
//    .ctmTranslate = ctmTranslate,
//    .ctmScale = ctmScale,
//    .ctmRotate = ctmRotate,
//
//    .contextPush = contextPush,
//    .contextPop = contextPop,
};

static bool getCanvas(displayCanvas_t *canvas, const displayPort_t *instance)
{
    UNUSED(instance);

    canvas->vTable = &spracingPixelOsdCanvasVTable;
    canvas->width = SPRACING_PIXEL_OSD_HORIZONTAL_RESOLUTION;
    canvas->height = SPRACING_PIXEL_OSD_PAL_VISIBLE_LINES; // FIXME hardcoded to PAL for now.
    return true;
}

static void beginTransaction(displayPort_t *instance, displayTransactionOption_e opts)
{
    UNUSED(instance);
    UNUSED(opts);

    spracingPixelOSDBeginRendering();
    frameBuffer = spracingPixelOSDGetActiveFrameBuffer();
}

spracingPixelOSDFrameState_t frameState;

spracingPixelOSDSyncVoltages_t *syncVoltages;

static void commitTransaction(displayPort_t *instance)
{
    UNUSED(instance);

    spracingPixelOSDLibraryVTable->refreshFrameState(&frameState);
#if USE_SPRACING_PIXELOSD_DEBUG_OVERLAY
    spracingPixelOSDRenderDebugOverlay(frameBuffer, &frameState, syncVoltages);
#endif

    spracingPixelOSDLibraryVTable->frameBufferCommit(frameBuffer);

    spracingPixelOSDEndRendering();

    if (debugMode == DEBUG_SPRACING_PIXEL_OSD) {
      debug[0] = syncVoltages->minimumLevelForLineThreshold;
      debug[1] = syncVoltages->minimumLevelForValidFrameMv;
      debug[2] = syncVoltages->maximumLevelForValidFrameMv;
      debug[3] = syncVoltages->syncThresholdMv;
    }
}

static const displayPortVTable_t spracingPixelOSDVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .redraw = redraw,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
    .layerSupported = layerSupported,
    .layerSelect = layerSelect,
    .layerCopy = layerCopy,
    .writeFontCharacter = writeFontCharacter,
    .beginTransaction = beginTransaction,
    .commitTransaction = commitTransaction,
    .getCanvas = getCanvas,
};

displayPort_t *spracingPixelOSDDisplayPortInit(const vcdProfile_t *vcdProfile)
{
    pixelOSDInitialised = spracingPixelOSDInit(vcdProfile);

    if (!pixelOSDInitialised) {
        return NULL;
    }

    syncVoltages = spracingPixelOSDLibraryVTable->getSyncVoltages();

    spracingPixelOSDDisplayPort.frameBufferCount = 2;

    displayInit(&spracingPixelOSDDisplayPort, &spracingPixelOSDVTable, DISPLAYPORT_DEVICE_TYPE_SPRACINGPIXELOSD);

    redraw(&spracingPixelOSDDisplayPort);
    return &spracingPixelOSDDisplayPort;
}
#endif // USE_SPRACING_PIXEL_OSD
