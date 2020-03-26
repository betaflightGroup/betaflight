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

#include "platform.h"

#include "spracingpixelosd_framebuffer_api.h"

#include "framebuffer.h"

#include "framebuffer_canvas.h"

extern uint8_t *frameBuffer;

typedef struct frameBufferCanvasContext_s {
    int x;
    int y;
    displayCanvasColor_e strokeColor;
} frameBufferCanvasContext_t;


static frameBufferCanvasContext_t defaultContext = {
    .x = 0,
    .y = 0,
    .strokeColor = DISPLAY_CANVAS_COLOR_WHITE,
};
static frameBufferCanvasContext_t *context = &defaultContext;

// sync with displayCanvasColor_e and FRAME_PIXEL_*

static const uint8_t canvasColorToFrameBufferModeMap[] = {
    FRAME_PIXEL_BLACK,
    FRAME_PIXEL_TRANSPARENT,
    FRAME_PIXEL_WHITE,
    FRAME_PIXEL_GREY
};

void frameBufferCanvasSetStrokeColor(displayCanvas_t *displayCanvas, displayCanvasColor_e color)
{
    UNUSED(displayCanvas);
    context->strokeColor = color;
}

void frameBufferCanvasMoveToPoint(displayCanvas_t *displayCanvas, int x, int y)
{
    UNUSED(displayCanvas);

    context->x = x;
    context->y = y;
}

void frameBufferCanvasStrokeLineToPoint(displayCanvas_t *displayCanvas, int x, int y)
{
    UNUSED(displayCanvas);

    framebuffer_drawLine(frameBuffer, context->x, context->y, x, y, canvasColorToFrameBufferModeMap[context->strokeColor]);
}

