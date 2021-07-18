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

#include "build/debug.h"

#include "common/maths.h"

#include "drivers/osd/font_max7456_12x18.h"
#include "drivers/osd/fonts.h"
#include "drivers/time.h"

#include "configuration.h"

#include "spracingpixelosd_conf.h"
#include "spracingpixelosd_framebuffer_api.h"

#include "framebuffer.h"

static void Error_Handler(void) { while (1) { } }

//
// Frame Buffer
//

#define FRAME_PIXEL_MASK        ((1 << 1) | (1 << 0))

#define BLOCK_TRANSPARENT ((FRAME_PIXEL_TRANSPARENT << 6) | (FRAME_PIXEL_TRANSPARENT << 4) | (FRAME_PIXEL_TRANSPARENT << 2) | (FRAME_PIXEL_TRANSPARENT << 0))
#define BLOCK_BLACK ((FRAME_PIXEL_BLACK << 6) | (FRAME_PIXEL_BLACK << 4) | (FRAME_PIXEL_BLACK << 2) | (FRAME_PIXEL_BLACK  << 0))
#define BLOCK_DEBUG ((FRAME_PIXEL_WHITE << 6) | (FRAME_PIXEL_BLACK << 4) | (FRAME_PIXEL_GREY << 2) | (FRAME_PIXEL_TRANSPARENT << 0))
#define BLOCK_FILL BLOCK_TRANSPARENT

#define PIXELS_PER_BYTE (8 / BITS_PER_PIXEL)

// frame buffer memory configured via linker scripts
extern uint8_t __frame_buffer_start;
extern uint8_t __frame_buffer_end;


uint8_t *frameBuffers = &__frame_buffer_start;

//
// Frame Buffer API
//

uint8_t *frameBuffer_getBuffer(uint8_t index)
{
    uint8_t *frameBuffer = frameBuffers + (FRAME_BUFFER_SIZE * index);
    return frameBuffer;
}

uint8_t frameBuffer_getBufferIndex(uint8_t *frameBuffer)
{
    return (frameBuffer - frameBuffers) / FRAME_BUFFER_SIZE;
}

DMA_RAM uint32_t fillColor __attribute__((aligned(32)));

// there is a tradeoff between the overhead of setting up streams and the time it takes for a stream to complete

// Examples for framebuffer size (((360 / 8) * 2) * 288) = 25920 bytes
// 25920 bytes / 4096 / 4 = 2 streams (using maximum block count, longest transfer smallest ram use)
// 25920 bytes / 2048 / 4 = 4 streams
// 25920 bytes / 1024 / 4 = 7 streams (using smaller block count, higher overhead and ram use)

#define MDMA_MAXIMUM_BLOCK_COUNT 4096
#define MDMA_BLOCK_COUNT 4096
#define PARALLEL_STREAM_COUNT (int)((FRAME_BUFFER_SIZE / MDMA_BLOCK_COUNT / sizeof(uint32_t)) + 1) // + 1 to avoid integer rounding
static MDMA_HandleTypeDef     frameBuffer_MDMA_Handle_Erase_Parallel[PARALLEL_STREAM_COUNT] = { 0 };
volatile bool streamActive[PARALLEL_STREAM_COUNT] = {0}; // updated by ISR callbacks

FAST_CODE void MDMA_IRQHandler(void)
{
    for (int i = 0; i < PARALLEL_STREAM_COUNT; i ++) {
        if (!streamActive[i]) {
          continue;
        }

        MDMA_HandleTypeDef *handle = &frameBuffer_MDMA_Handle_Erase_Parallel[i];
        HAL_MDMA_IRQHandler(handle);
    }
}

void frameBuffer_xferBlockCpltHandler( struct __MDMA_HandleTypeDef * hmdma)
{
    for (int i = 0; i < PARALLEL_STREAM_COUNT; i ++) {
        if (!streamActive[i]) {
          continue;
        }

        MDMA_HandleTypeDef *handle = &frameBuffer_MDMA_Handle_Erase_Parallel[i];
        bool handleIsForThisStream = (hmdma == handle);
        if (!handleIsForThisStream) {
            continue;
        }
        streamActive[i] = false;
    }
}

void frameBuffer_eraseInit(void)
{
    __HAL_RCC_MDMA_CLK_ENABLE();

    static MDMA_HandleTypeDef     frameBuffer_MDMA_Handle_Erase = { 0 };

    frameBuffer_MDMA_Handle_Erase.Init.Request              = MDMA_REQUEST_SW;
    frameBuffer_MDMA_Handle_Erase.Init.TransferTriggerMode  = MDMA_REPEAT_BLOCK_TRANSFER;
    frameBuffer_MDMA_Handle_Erase.Init.Priority             = MDMA_PRIORITY_HIGH;
    frameBuffer_MDMA_Handle_Erase.Init.Endianness           = MDMA_LITTLE_ENDIANNESS_PRESERVE;

    frameBuffer_MDMA_Handle_Erase.Init.DataAlignment        = MDMA_DATAALIGN_PACKENABLE;
    frameBuffer_MDMA_Handle_Erase.Init.BufferTransferLength = 128;

    frameBuffer_MDMA_Handle_Erase.Init.DestinationInc       = MDMA_DEST_INC_WORD;
    frameBuffer_MDMA_Handle_Erase.Init.DestDataSize         = MDMA_DEST_DATASIZE_WORD;
    frameBuffer_MDMA_Handle_Erase.Init.DestBurst            = MDMA_DEST_BURST_SINGLE;
    frameBuffer_MDMA_Handle_Erase.Init.DestBlockAddressOffset    = 0;

    frameBuffer_MDMA_Handle_Erase.Init.SourceInc            = MDMA_SRC_INC_DISABLE;
    frameBuffer_MDMA_Handle_Erase.Init.SourceDataSize       = MDMA_SRC_DATASIZE_WORD;
    frameBuffer_MDMA_Handle_Erase.Init.SourceBurst          = MDMA_SOURCE_BURST_SINGLE;
    frameBuffer_MDMA_Handle_Erase.Init.SourceBlockAddressOffset  = 0;

    static const MDMA_Channel_TypeDef *mdmaChannelInstances[] = {
        MDMA_Channel0, MDMA_Channel1, MDMA_Channel2, MDMA_Channel3,
        MDMA_Channel4, MDMA_Channel5, MDMA_Channel6, MDMA_Channel7,
        MDMA_Channel8, MDMA_Channel9, MDMA_Channel10, MDMA_Channel11,
        MDMA_Channel12, MDMA_Channel13, MDMA_Channel14, MDMA_Channel15
    };

    for (int i = 0; i < PARALLEL_STREAM_COUNT; i ++) {
      MDMA_HandleTypeDef *handle = &frameBuffer_MDMA_Handle_Erase_Parallel[i];

      memcpy(handle, &frameBuffer_MDMA_Handle_Erase, sizeof(MDMA_HandleTypeDef));

      handle->Instance = (MDMA_Channel_TypeDef *)mdmaChannelInstances[i];
      handle->XferCpltCallback = frameBuffer_xferBlockCpltHandler;

      if (HAL_MDMA_Init(handle) != HAL_OK) {
          Error_Handler();
      }
    }

    HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);

    /* Enable the MDMA channel global Interrupt */
    HAL_NVIC_EnableIRQ(MDMA_IRQn);

    fillColor = BLOCK_FILL << 24 | BLOCK_FILL << 16 | BLOCK_FILL << 8 | BLOCK_FILL;
}

void frameBuffer_eraseBegin(uint8_t *frameBuffer)
{
    TIME_SECTION_BEGIN(0);

    uint32_t hal_status;

    uint32_t bytesRemaining = FRAME_BUFFER_SIZE;

    MDMA_HandleTypeDef *handle;

    int streamIndex = 0;
    while (bytesRemaining > 0) {

        handle = &frameBuffer_MDMA_Handle_Erase_Parallel[streamIndex];


        uint32_t bytesToFill = MDMA_BLOCK_COUNT * sizeof(fillColor);
        if (bytesRemaining < bytesToFill) {
            bytesToFill = bytesRemaining;
        }

        uint32_t offset = FRAME_BUFFER_SIZE - bytesRemaining;

        if (streamActive[streamIndex]) {
            // if a stream is active the previous caller didn't wait for completion.

            // reset HAL state so a new transfer can be started.
            hal_status = HAL_MDMA_Abort(handle);
        }

        streamActive[streamIndex] = true; // must be set *before* the ISR handler can be called.

        hal_status = HAL_MDMA_Start_IT(handle, (uint32_t)&fillColor,
                                                  (uint32_t)frameBuffer + offset,
                                                  sizeof(fillColor),
                                                  bytesToFill / sizeof(fillColor));
        if(hal_status != HAL_OK)
        {
          /* Transfer Error */
          Error_Handler();
        }


        bytesRemaining -= bytesToFill;
        streamIndex++;
    }
    TIME_SECTION_END(0);
}

bool frameBuffer_eraseInProgress(void)
{
    for (int i = 0; i < PARALLEL_STREAM_COUNT; i ++) {
        if (!streamActive[i]) {
            continue;
        }

        return true;
    }
    return false;
}

void frameBuffer_eraseWaitForComplete(void)
{
    while (frameBuffer_eraseInProgress()) {
        NOOP;
    }
}

//
// Frame Buffer Drawing API
//

// Macro to swap two variables using XOR swap.
#define SWAP_XOR(a, b) { a ^= b; b ^= a; a ^= b; }

// unoptimized, avoid over-use.
void frameBuffer_setPixel(uint8_t *frameBuffer, uint16_t x, uint16_t y, uint8_t mode)
{
    uint8_t *lineBuffer = frameBuffer + (y * FRAME_BUFFER_LINE_SIZE);

    uint8_t pixelOffsetInBlock = (PIXELS_PER_BYTE - 1) - (x % PIXELS_PER_BYTE);

    uint8_t pixelBitOffset = BITS_PER_PIXEL * pixelOffsetInBlock;

    uint8_t mask = ~(FRAME_PIXEL_MASK << pixelBitOffset);

    uint8_t before = lineBuffer[x / PIXELS_PER_BYTE];
    uint8_t withMaskCleared = before & mask;
    lineBuffer[x / PIXELS_PER_BYTE] = withMaskCleared |
            (mode << pixelBitOffset);
}

void framebuffer_drawVerticalLine(uint8_t *frameBuffer, uint16_t x, uint16_t y0, uint16_t y1, uint8_t mode)
{
   if(y0 > y1)
   {
       SWAP_XOR(y0, y1);
   }

   uint8_t pixelOffsetInBlock = (PIXELS_PER_BYTE - 1) - (x % PIXELS_PER_BYTE);

   uint8_t pixelBitOffset = BITS_PER_PIXEL * pixelOffsetInBlock;

   uint8_t mask = ~(FRAME_PIXEL_MASK << pixelBitOffset);

   uint16_t blockOffset = x / PIXELS_PER_BYTE;

   uint16_t lineCount = y1 - y0;
   uint8_t *firstBlock = frameBuffer + (y0 * FRAME_BUFFER_LINE_SIZE) + blockOffset;
   uint8_t *lastBlock = firstBlock + (lineCount * FRAME_BUFFER_LINE_SIZE);

   for (uint8_t *block = firstBlock; block <= lastBlock; block += FRAME_BUFFER_LINE_SIZE) {
       uint8_t blockValue = *block;
       uint8_t blockValueWithMaskCleared = blockValue & mask;
       *block = blockValueWithMaskCleared | (mode << pixelBitOffset);
   }
}

void framebuffer_drawLine(uint8_t *frameBuffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t mode)
{
    // Based on http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    bool steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (steep) {
        SWAP_XOR(x0, y0);
        SWAP_XOR(x1, y1);
    }
    if(x0 > x1)
    {
        SWAP_XOR(x0, x1);
        SWAP_XOR(y0, y1);
    }
    int deltax = x1 - x0;
    int deltay = ABS(y1 - y0);
    int error = deltax / 2;
    int ystep;
    int y = y0;
    int x;
    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }
    for (x = x0; x < x1; x++) {

        // FIXME hardcoded to PAL for now.
        if (steep) {
            if (x >= 0 && y >= 0 && x < SPRACING_PIXEL_OSD_PAL_VISIBLE_LINES && y < SPRACING_PIXEL_OSD_HORIZONTAL_RESOLUTION) {
                frameBuffer_setPixel(frameBuffer, y, x, mode);
            }
        } else {
            if (x >= 0 && y >= 0 && x < SPRACING_PIXEL_OSD_HORIZONTAL_RESOLUTION && y < SPRACING_PIXEL_OSD_PAL_VISIBLE_LINES) {
                frameBuffer_setPixel(frameBuffer, x, y, mode);
            }
        }
        error -= deltay;
        if (error < 0) {
            y     += ystep;
            error += deltax;
        }
    }
}

void framebuffer_drawRectangle(uint8_t *frameBuffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t mode)
{
    framebuffer_drawLine(frameBuffer, x0, y0, x1, y0, mode); // top
    framebuffer_drawLine(frameBuffer, x0, y0, x0, y1, mode); // left
    framebuffer_drawLine(frameBuffer, x1, y0, x1, y1, mode); // right
    framebuffer_drawLine(frameBuffer, x0, y1, x1, y1, mode); // bottom
}

void frameBuffer_createTestPattern1(uint8_t *frameBuffer)
{
    for (int lineIndex = 0; lineIndex < SPRACING_PIXEL_OSD_PAL_VISIBLE_LINES; lineIndex++) {
        uint8_t *lineBuffer = frameBuffer + (lineIndex * FRAME_BUFFER_LINE_SIZE);

        if (lineIndex & 0x8) {
            continue; // empty vertical band.
        }

        for (int i = 0; i < SPRACING_PIXEL_OSD_HORIZONTAL_RESOLUTION / PIXELS_PER_BYTE; i ++) {

            lineBuffer[i] =
                    (FRAME_PIXEL_WHITE << (BITS_PER_PIXEL * 3)) |
                    (FRAME_PIXEL_GREY << (BITS_PER_PIXEL * 2)) |
                    (FRAME_PIXEL_BLACK << (BITS_PER_PIXEL * 1)) |
                    (FRAME_PIXEL_TRANSPARENT << (BITS_PER_PIXEL * 0));
        }
    }
}

void frameBuffer_createTestPattern2(uint8_t *frameBuffer)
{
    for (int lineIndex = 0; lineIndex < SPRACING_PIXEL_OSD_PAL_VISIBLE_LINES; lineIndex++) {
        int x;

        x = lineIndex;
        frameBuffer_setPixel(frameBuffer, x, lineIndex, FRAME_PIXEL_BLACK);
        frameBuffer_setPixel(frameBuffer, x+1, lineIndex, FRAME_PIXEL_WHITE);
        frameBuffer_setPixel(frameBuffer, x+2, lineIndex, FRAME_PIXEL_BLACK);

        x = SPRACING_PIXEL_OSD_HORIZONTAL_RESOLUTION - 1 - lineIndex;
        frameBuffer_setPixel(frameBuffer, x, lineIndex, FRAME_PIXEL_BLACK);
        frameBuffer_setPixel(frameBuffer, x-1, lineIndex, FRAME_PIXEL_WHITE);
        frameBuffer_setPixel(frameBuffer, x-2, lineIndex, FRAME_PIXEL_BLACK);

    }
}

// unoptimized for now
void frameBuffer_slowWriteCharacter(uint8_t *frameBuffer, uint16_t x, uint16_t y, uint8_t characterIndex)
{
    uint16_t fontCharacterOffset = characterIndex * FONT_MAX7456_12x18_BYTES_PER_CHARACTER;

    for (int row = 0; row < FONT_MAX7456_HEIGHT; row++) {
        uint16_t fy = y + row;
        uint16_t fx = x;

        for (int b = 0; b < 3; b++) {
            uint8_t c = font_max7456_12x18[fontCharacterOffset];
            fontCharacterOffset++;

            for (int p = 0; p <= 3; p++) {
                uint8_t mp = (c >> (2 * (3 - p))) & ((1 << 1) | (1 << 0)); // extract max7456 pixel from character
                uint8_t mode = FRAME_PIXEL_TRANSPARENT;

                if (mp == ((0 << 1) | (0 << 0))) {
                    mode = FRAME_PIXEL_BLACK;
                } else if (mp == ((1 << 1) | (0 << 0))) {
                    mode = FRAME_PIXEL_WHITE;
                }
                if (mode != 0xFF) {
                    frameBuffer_setPixel(frameBuffer, fx, fy, mode);
                }
                fx++;
            }
        }

    }
}

// unoptimized for now
void frameBuffer_slowWriteString(uint8_t *frameBuffer, uint16_t x, uint16_t y, const uint8_t *message, uint8_t messageLength)
{
    uint16_t fx = x;
    for (int mi = 0; mi < messageLength; mi++) {
#if USE_FONT_MAPPING
        uint8_t c = font_max7456_12x18_asciiToFontMapping[message[mi]];
#else
        uint8_t c = message[mi];
#endif
        if (c) {
          // only render non-null characters, i.e skip nulls or characters without a mapping.

          frameBuffer_slowWriteCharacter(frameBuffer, fx, y, c);
        }
        fx+= FONT_MAX7456_WIDTH;
    }
}
