/*
 * This file is part of Cleanflight, Betaflight and INAV
 *
 * Cleanflight, Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Cleanflight, Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_FRSKYOSD)

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/display_canvas.h"

#include "io/displayport_frsky_osd.h"
#include "io/frsky_osd.h"

static displayPort_t frskyOsdDisplayPort;

static int grab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
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
    UNUSED(options);
    frskyOsdClearScreen();
    return 0;
}

static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    frskyOsdUpdate();

    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return frskyOsdGetGridRows() * frskyOsdGetGridCols();
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    UNUSED(displayPort);
    UNUSED(attr);

    frskyOsdDrawStringInGrid(x, y, s);
    return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    UNUSED(displayPort);
    UNUSED(attr);

    frskyOsdDrawCharInGrid(x, y, c);
    return 0;
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static void updateGridSize(displayPort_t *displayPort)
{
    displayPort->rows = frskyOsdGetGridRows();
    displayPort->cols = frskyOsdGetGridCols();
}

static void redraw(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    // TODO(agh): Do we need to flush the screen here?
    // MAX7456's driver does a full redraw in redraw(),
    // so some callers might be expecting that.
    frskyOsdUpdate();
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

static bool writeFontCharacter(displayPort_t *instance, uint16_t addr, const osdCharacter_t *chr)
{
    UNUSED(instance);

    return frskyOsdWriteFontCharacter(addr, chr);
}

static bool checkReady(displayPort_t *instance, bool rescan)
{
    UNUSED(rescan);

    if (frskyOsdIsReady()) {
        updateGridSize(instance);
        return true;
    }
    return false;
}

static void beginTransaction(displayPort_t *instance, displayTransactionOption_e opts)
{
    UNUSED(instance);

    frskyOsdTransactionOptions_e frskyOpts = 0;
    if (opts & DISPLAY_TRANSACTION_OPT_PROFILED) {
        frskyOpts |= FRSKY_OSD_TRANSACTION_OPT_PROFILED;
    }
    if (opts & DISPLAY_TRANSACTION_OPT_RESET_DRAWING) {
        frskyOpts |= FRSKY_OSD_TRANSACTION_OPT_RESET_DRAWING;
    }

    frskyOsdBeginTransaction(frskyOpts);
}

static void commitTransaction(displayPort_t *instance)
{
    UNUSED(instance);

    frskyOsdCommitTransaction();
}

static frskyOsdColor_e frskyOsdGetColor(displayCanvasColor_e color)
{
    switch (color)
    {
        case DISPLAY_CANVAS_COLOR_BLACK:
            return FRSKY_OSD_COLOR_BLACK;
        case DISPLAY_CANVAS_COLOR_TRANSPARENT:
            return FRSKY_OSD_COLOR_TRANSPARENT;
        case DISPLAY_CANVAS_COLOR_WHITE:
            return FRSKY_OSD_COLOR_WHITE;
        case DISPLAY_CANVAS_COLOR_GRAY:
            return FRSKY_OSD_COLOR_GRAY;
    }
    return FRSKY_OSD_COLOR_BLACK;
}

static void setStrokeColor(displayCanvas_t *displayCanvas, displayCanvasColor_e color)
{
    UNUSED(displayCanvas);

    frskyOsdSetStrokeColor(frskyOsdGetColor(color));
}

static void setFillColor(displayCanvas_t *displayCanvas, displayCanvasColor_e color)
{
    UNUSED(displayCanvas);

    frskyOsdSetFillColor(frskyOsdGetColor(color));
}

static void setStrokeAndFillColor(displayCanvas_t *displayCanvas, displayCanvasColor_e color)
{
    UNUSED(displayCanvas);

    frskyOsdSetStrokeAndFillColor(frskyOsdGetColor(color));
}

static void setColorInversion(displayCanvas_t *displayCanvas, bool inverted)
{
    UNUSED(displayCanvas);

    frskyOsdSetColorInversion(inverted);
}

static void setPixel(displayCanvas_t *displayCanvas, int x, int y, displayCanvasColor_e color)
{
    UNUSED(displayCanvas);

    frskyOsdSetPixel(x, y, frskyOsdGetColor(color));
}

static void setPixelToStrokeColor(displayCanvas_t *displayCanvas, int x, int y)
{
    UNUSED(displayCanvas);

    frskyOsdSetPixelToStrokeColor(x, y);
}

static void setPixelToFillColor(displayCanvas_t *displayCanvas, int x, int y)
{
    UNUSED(displayCanvas);

    frskyOsdSetPixelToFillColor(x, y);
}

static void setStrokeWidth(displayCanvas_t *displayCanvas, unsigned w)
{
    UNUSED(displayCanvas);

    frskyOsdSetStrokeWidth(w);
}

static void setLineOutlineType(displayCanvas_t *displayCanvas, displayCanvasOutlineType_e outlineType)
{
    UNUSED(displayCanvas);

    frskyOsdSetLineOutlineType((frskyOsdLineOutlineType_e)outlineType);
}

static void setLineOutlineColor(displayCanvas_t *displayCanvas, displayCanvasColor_e outlineColor)
{
    UNUSED(displayCanvas);

    frskyOsdSetLineOutlineColor((frskyOsdColor_e)outlineColor);
}

static void clipToRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h)
{
    UNUSED(displayCanvas);

    frskyOsdClipToRect(x, y, w, h);
}

static void clearRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h)
{
    UNUSED(displayCanvas);

    frskyOsdClearRect(x, y, w, h);
}

static void resetDrawingState(displayCanvas_t *displayCanvas)
{
    UNUSED(displayCanvas);

    frskyOsdResetDrawingState();
}

static void drawCharacter(displayCanvas_t *displayCanvas, int x, int y, uint16_t chr, displayCanvasBitmapOption_t opts)
{
    UNUSED(displayCanvas);

    frskyOsdDrawCharacter(x, y, chr, opts);
}

static void drawCharacterMask(displayCanvas_t *displayCanvas, int x, int y, uint16_t chr, displayCanvasColor_e color, displayCanvasBitmapOption_t opts)
{
    UNUSED(displayCanvas);

    frskyOsdDrawCharacterMask(x, y, chr, frskyOsdGetColor(color), opts);
}

static void drawString(displayCanvas_t *displayCanvas, int x, int y, const char *s, displayCanvasBitmapOption_t opts)
{
    UNUSED(displayCanvas);

    frskyOsdDrawString(x, y, s, opts);
}

static void drawStringMask(displayCanvas_t *displayCanvas, int x, int y, const char *s, displayCanvasColor_e color, displayCanvasBitmapOption_t opts)
{
    UNUSED(displayCanvas);

    frskyOsdDrawStringMask(x, y, s, frskyOsdGetColor(color), opts);
}

static void moveToPoint(displayCanvas_t *displayCanvas, int x, int y)
{
    UNUSED(displayCanvas);

    frskyOsdMoveToPoint(x, y);
}

static void strokeLineToPoint(displayCanvas_t *displayCanvas, int x, int y)
{
    UNUSED(displayCanvas);

    frskyOsdStrokeLineToPoint(x, y);
}

static void strokeTriangle(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3)
{
    UNUSED(displayCanvas);

    frskyOsdStrokeTriangle(x1, y1, x2, y2, x3, y3);
}

static void fillTriangle(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3)
{
    UNUSED(displayCanvas);

    frskyOsdFillTriangle(x1, y1, x2, y2, x3, y3);
}

static void fillStrokeTriangle(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3)
{
    UNUSED(displayCanvas);

    frskyOsdFillStrokeTriangle(x1, y1, x2, y2, x3, y3);
}

static void strokeRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h)
{
    UNUSED(displayCanvas);

    frskyOsdStrokeRect(x, y, w, h);
}

static void fillRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h)
{
    UNUSED(displayCanvas);

    frskyOsdFillRect(x, y, w, h);
}

static void fillStrokeRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h)
{
    UNUSED(displayCanvas);

    frskyOsdFillStrokeRect(x, y, w, h);
}

static void strokeEllipseInRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h)
{
    UNUSED(displayCanvas);

    frskyOsdStrokeEllipseInRect(x, y, w, h);
}

static void fillEllipseInRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h)
{
    UNUSED(displayCanvas);

    frskyOsdFillEllipseInRect(x, y, w, h);
}

static void fillStrokeEllipseInRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h)
{
    UNUSED(displayCanvas);

    frskyOsdFillStrokeEllipseInRect(x, y, w, h);
}

static void ctmReset(displayCanvas_t *displayCanvas)
{
    UNUSED(displayCanvas);

    frskyOsdCtmReset();
}

static void ctmSet(displayCanvas_t *displayCanvas, float m11, float m12, float m21, float m22, float m31, float m32)
{
    UNUSED(displayCanvas);

    frskyOsdCtmSet(m11, m12, m21, m22, m31, m32);
}

static void ctmTranslate(displayCanvas_t *displayCanvas, float tx, float ty)
{
    UNUSED(displayCanvas);

    frskyOsdCtmTranslate(tx, ty);
}

static void ctmScale(displayCanvas_t *displayCanvas, float sx, float sy)
{
    UNUSED(displayCanvas);

    frskyOsdCtmScale(sx, sy);
}

static void ctmRotate(displayCanvas_t *displayCanvas, float r)
{
    UNUSED(displayCanvas);

    frskyOsdCtmRotate(r);
}

static void contextPush(displayCanvas_t *displayCanvas)
{
    UNUSED(displayCanvas);

    frskyOsdContextPush();
}

static void contextPop(displayCanvas_t *displayCanvas)
{
    UNUSED(displayCanvas);

    frskyOsdContextPop();
}


static const displayCanvasVTable_t frskyOsdCanvasVTable = {
    .setStrokeColor = setStrokeColor,
    .setFillColor = setFillColor,
    .setStrokeAndFillColor = setStrokeAndFillColor,
    .setColorInversion = setColorInversion,
    .setPixel = setPixel,
    .setPixelToStrokeColor = setPixelToStrokeColor,
    .setPixelToFillColor = setPixelToFillColor,
    .setStrokeWidth = setStrokeWidth,
    .setLineOutlineType = setLineOutlineType,
    .setLineOutlineColor = setLineOutlineColor,

    .clipToRect = clipToRect,
    .clearRect = clearRect,
    .resetDrawingState = resetDrawingState,
    .drawCharacter = drawCharacter,
    .drawCharacterMask = drawCharacterMask,
    .drawString = drawString,
    .drawStringMask = drawStringMask,
    .moveToPoint = moveToPoint,
    .strokeLineToPoint = strokeLineToPoint,
    .strokeTriangle = strokeTriangle,
    .fillTriangle = fillTriangle,
    .fillStrokeTriangle = fillStrokeTriangle,
    .strokeRect = strokeRect,
    .fillRect = fillRect,
    .fillStrokeRect = fillStrokeRect,
    .strokeEllipseInRect = strokeEllipseInRect,
    .fillEllipseInRect = fillEllipseInRect,
    .fillStrokeEllipseInRect = fillStrokeEllipseInRect,

    .ctmReset = ctmReset,
    .ctmSet = ctmSet,
    .ctmTranslate = ctmTranslate,
    .ctmScale = ctmScale,
    .ctmRotate = ctmRotate,

    .contextPush = contextPush,
    .contextPop = contextPop,
};

static bool getCanvas(displayCanvas_t *canvas, const displayPort_t *instance)
{
    UNUSED(instance);

    canvas->vTable = &frskyOsdCanvasVTable;
    canvas->width = frskyOsdGetPixelWidth();
    canvas->height = frskyOsdGetPixelHeight();
    return true;
}

static const displayPortVTable_t frskyOsdVTable = {
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
    .txBytesFree = txBytesFree,
    .writeFontCharacter = writeFontCharacter,
    .checkReady = checkReady,
    .beginTransaction = beginTransaction,
    .commitTransaction = commitTransaction,
    .getCanvas = getCanvas,
};

displayPort_t *frskyOsdDisplayPortInit(const videoSystem_e videoSystem)
{
    if (frskyOsdInit(videoSystem)) {
        displayInit(&frskyOsdDisplayPort, &frskyOsdVTable, DISPLAYPORT_DEVICE_TYPE_FRSKYOSD);
        redraw(&frskyOsdDisplayPort);
        return &frskyOsdDisplayPort;
    }
    return NULL;
}

#endif // USE_FRSKYOSD
