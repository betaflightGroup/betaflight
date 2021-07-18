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
 * Overview:
 *
 * The SP Racing Pixel OSD is a software On-Screen Display (OSD) solution.
 *
 * Basic features:
 * - PAL/NTSC camera detection
 * - Internal video signal generation for when no-camera is connected.
 * - 4 'colors' (transparent, black, white, grey)
 * - In-memory double-buffered framebuffers.
 * - Direct control of every single pixel, i.e. not character based.  Hence `PIXEL` OSD.
 *
 * System overview:
 * - Bootloader installs OSD system library into RAM.
 * - Bootloader loads FC firmware.
 * - FC looks for OSD system library, checks descriptor (API version, etc).
 * - FC reserves hardware resources and prevents use by other parts of the system (some GPIO pins, timers, DMA channels, etc)
 * - FC provides callbacks to OSD system library during initalisation.
 * - FC continually checks state and calls OSD system functions based on OSD state (e.g. for camera sync detection, etc)
 * - FC renders into framebuffer and tells OSD system that the frame is ready.
 * - OSD system calls FC vsync callback when frame is about to be rendered.
 * - FC renders into alternate framebuffer.
 * - The FC and OSD then repeats the cycle of rendering one framebuffer while the other is being displayed.
 *
 * Hardware details (H750):
 * - Reserved timers: TIM1, TIM2, TIM15
 * - Reserved DMA streams: DMA2 Stream 6 and 7.
 * - Reserved peripherals: ADC1, DAC (Both channels), COMP2
 * - Some areas of RAM are reserved for OSD code and runtime data.
 *
 * Implementation:
 * - vtables are used for access to the library functions, standard calling convention.
 * - library descriptor (code and API version) must be checked before calling any library function.
 * - all framebuffer drawing code is implemented in the FC system.
 * - DisplayPort and Canvas API's are used for rendering.
 */

/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working Pixel OSD system.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPRACING_PIXEL_OSD

#include "spracingpixelosd_api.h"
#include "spracingpixelosd_framebuffer_api.h"

#include "common/printf.h"
#include "drivers/display.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/osd.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/spracingpixelosd/framebuffer.h"
#include "drivers/spracingpixelosd/spracing_pixel_osd_library.h"
#include "drivers/spracingpixelosd/configuration.h"

#include "pg/spracing_pixel_osd.h"
#include "pg/vcd.h"

#include "drivers/osd/font_max7456_12x18.h"


#include "spracing_pixel_osd.h"

spracingPixelOSDState_t *pixelOSDState;
volatile bool frameRenderingComplete = false;
static volatile uint8_t frameBufferIndex = 0;

//
// Low-level handlers
//

FAST_CODE static void SYNC_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    UNUSED(descriptor);
    spracingPixelOSDLibraryVTable->syncDMAHandler();
}

FAST_CODE static void PIXEL_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    UNUSED(descriptor);
    spracingPixelOSDLibraryVTable->pixelDMAHandler();
}

FAST_CODE static void ADC_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    UNUSED(descriptor);
    spracingPixelOSDLibraryVTable->adcDMAHandler();
}

FAST_CODE void COMP1_IRQHandler(void)
{
    // don't call this if comparator that caused the interrupt is not the one used by the OSD system
    // if your target is using both comparators then you will have to handle this here.
    // IMPORTANT: CPU cycles count here!
    spracingPixelOSDLibraryVTable->comparatorIRQHandler();
}

//
// Framebuffer handling
//

static void onVSync(void) // ISR callback
{
    if (frameRenderingComplete) {
        if (frameBufferIndex == 0) {
            frameBufferIndex = 1;
        } else {
            frameBufferIndex = 0;
        }

        frameRenderingComplete = false;
    }
}

void spracingPixelOSDBeginRendering(void)
{
    frameRenderingComplete = false;
}

void spracingPixelOSDEndRendering(void)
{
    frameRenderingComplete = true;
}

bool spracingPixelOSDIsFrameRenderingComplete(void)
{
    return frameRenderingComplete;
}

uint8_t *spracingPixelOSDGetActiveFrameBuffer(void)
{
    uint8_t *activeFrameBuffer = frameBuffer_getBuffer(frameBufferIndex);
    return activeFrameBuffer;
}

void frameBufferInit(void)
{
    //
    // Frame
    //

    frameBuffer_eraseInit();

    uint8_t * fb0 = frameBuffer_getBuffer(0);
    uint8_t * fb1 = frameBuffer_getBuffer(1);

    frameBuffer_eraseBegin(fb0);
    frameBuffer_eraseWaitForComplete();
    frameBuffer_eraseBegin(fb1);
    frameBuffer_eraseWaitForComplete();

    //frameBuffer_createTestPattern1(fb0);
    //frameBuffer_createTestPattern1(fb1);

    //frameBuffer_createTestPattern2(fb0);

    framebuffer_drawRectangle(fb0, 0, 0, SPRACING_PIXEL_OSD_HORIZONTAL_RESOLUTION - 1, SPRACING_PIXEL_OSD_PAL_VISIBLE_LINES - 1, FRAME_PIXEL_WHITE);

    frameBuffer_slowWriteString(fb0, 50, 150, (uint8_t*)"SP RACING PIXEL OSD", 19);
}

static void configureDMAHandlers(void)
{
    //
    // ADC DMA
    //

    dmaResource_t *adcDmaRef = dmaGetRefByIdentifier(DMA2_ST5_HANDLER);
    uint16_t adcDmaParam = 0;

    dmaInit(dmaGetIdentifier(adcDmaRef), OWNER_OSD, 0);
    dmaSetHandler(dmaGetIdentifier(adcDmaRef), ADC_DMA_IRQHandler, NVIC_PRIO_VIDEO_ADC, adcDmaParam);

    //
    // Sync Detecton
    //

    HAL_NVIC_SetPriority(COMP_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_VIDEO_SYNC), NVIC_PRIORITY_SUB(NVIC_PRIO_VIDEO_SYNC));

    //
    // Sync Generation DMA
    //

    ioTag_t syncIoTag = timerioTagGetByUsage(TIM_USE_VIDEO_SYNC, 0);
    const timerHardware_t *syncTimerHardware = timerGetByTag(syncIoTag);

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *syncDmaSpec = dmaGetChannelSpecByTimer(syncTimerHardware);

    if (!syncDmaSpec) {
        return;
    }

    const dmaResource_t *syncDmaRef = syncDmaSpec->ref;
    uint32_t syncDmaChannel = syncDmaSpec->channel;
#else
    const dmaResource_t *syncDmaRef = syncTimerHardware->dmaRef;
    uint32_t syncDmaChannel = syncTimerHardware->dmaChannel;
#endif

    UNUSED(syncDmaChannel);

    uint16_t syncTimerChannel = syncTimerHardware->channel;
    uint16_t syncDmaIndex = timerDmaIndex(syncTimerChannel);

    dmaInit(dmaGetIdentifier(syncDmaRef), OWNER_OSD, 0);
    dmaSetHandler(dmaGetIdentifier(syncDmaRef), SYNC_DMA_IRQHandler, NVIC_PRIO_VIDEO_DMA, syncDmaIndex);

    //
    // Pixel Generation DMA
    //


    ioTag_t pixelIoTag = timerioTagGetByUsage(TIM_USE_VIDEO_PIXEL, 0);
    const timerHardware_t *pixelTimerHardware = timerGetByTag(pixelIoTag);

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *pixelDmaSpec = dmaGetChannelSpecByTimer(pixelTimerHardware);

    if (!pixelDmaSpec) {
        return;
    }

    const dmaResource_t *pixelDmaRef = pixelDmaSpec->ref;
    uint32_t pixelDmaChannel = pixelDmaSpec->channel;
#else
    const dmaResource_t *pixelDmaRef = pixelTimerHardware->dmaRef;
    uint32_t pixelDmaChannel = pixelTimerHardware->dmaChannel;
#endif

    UNUSED(pixelDmaChannel);

    uint16_t pixelTimerChannel = pixelTimerHardware->channel;
    uint16_t pixelDmaIndex = timerDmaIndex(pixelTimerChannel);

    dmaInit(dmaGetIdentifier(pixelDmaRef), OWNER_OSD, 0);
    dmaSetHandler(dmaGetIdentifier(pixelDmaRef), PIXEL_DMA_IRQHandler, NVIC_PRIO_VIDEO_DMA, pixelDmaIndex);
}

typedef struct spracingPixelOSDIO_s {
    IO_t blackPin;
    IO_t whitePin;
    IO_t syncInPin;
    IO_t debug1Pin;
    IO_t debug2Pin;
#ifdef DEBUG_BLANKING
    IO_t blankingDebugPin;
#endif
#ifdef DEBUG_GATING
    IO_t gatingDebugPin;
#endif
    IO_t whiteSourceSelectPin;
    IO_t maskEnablePin;
} spracingPixelOSDIO_t;

static spracingPixelOSDIO_t spracingPixelOSDIO = {
    .blackPin               = IO_NONE,
    .whitePin               = IO_NONE,
    .syncInPin              = IO_NONE,
    .debug1Pin              = IO_NONE,
    .debug2Pin              = IO_NONE,
#ifdef DEBUG_BLANKING
    .blankingDebugPin       = IO_NONE,
#endif
#ifdef DEBUG_GATING
    .gatingDebugPin         = IO_NONE,
#endif
    .whiteSourceSelectPin   = IO_NONE,
    .maskEnablePin          = IO_NONE,
};

static void reserveGPIOPins(void)
{
    spracingPixelOSDIO.blackPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_BLACK_PIN));
    spracingPixelOSDIO.maskEnablePin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_MASK_ENABLE_PIN));
    spracingPixelOSDIO.whitePin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_WHITE_PIN));
    spracingPixelOSDIO.whiteSourceSelectPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN));
    spracingPixelOSDIO.syncInPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_SYNC_IN_PIN));
#ifdef DEBUG_BLANKING
    spracingPixelOSDIO.blankingDebugPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN));
#endif
#ifdef DEBUG_GATING
    spracingPixelOSDIO.gatingDebugPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN));
#endif
    spracingPixelOSDIO.debug1Pin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN));
    spracingPixelOSDIO.debug2Pin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN));

    IOInit(spracingPixelOSDIO.blackPin, OWNER_OSD, 0);
    IOInit(spracingPixelOSDIO.maskEnablePin, OWNER_OSD, 0);
    IOInit(spracingPixelOSDIO.whitePin, OWNER_OSD, 0);
    IOInit(spracingPixelOSDIO.whiteSourceSelectPin, OWNER_OSD, 0);
    IOInit(spracingPixelOSDIO.syncInPin, OWNER_OSD, 0);

#ifdef DEBUG_BLANKING
    IOInit(spracingPixelOSDIO.blankingDebugPin, OWNER_OSD, 0);
#endif
#ifdef DEBUG_GATING
    IOInit(spracingPixelOSDIO.gatingDebugPin, OWNER_OSD, 0);
#endif

    IOInit(spracingPixelOSDIO.debug1Pin, OWNER_OSD, 0);
    IOInit(spracingPixelOSDIO.debug2Pin, OWNER_OSD, 0);
}

void spracingPixelOSDIOInit(void)
{
    reserveGPIOPins(); // *always* reserve GPIO pins to prevent the firmware re-using them, see unusedPinsInit().
}

bool spracingPixelOSDInit(const vcdProfile_t *vcdProfile)
{
    static const spracingPixelOSDHostAPI_t pixelOSDHostAPI = {
        .micros = micros,
        .onVSync = onVSync,
    };

    if (!spracingPixelOSDIsLibraryAvailable()) {
        return false;
    }

    frameBufferInit();

    configureDMAHandlers();
    // TODO reserve timers TIM1 and TIM15

    spracingPixelOSDDefaultConfig_t defaultConfig = {
        .flags = (vcdProfile->video_system == VIDEO_SYSTEM_PAL ? PIXELOSD_CF_VIDEO_SYSTEM_PAL : PIXELOSD_CF_VIDEO_SYSTEM_NTSC),
    };

    spracingPixelOSDLibraryVTable->init(&pixelOSDHostAPI, &defaultConfig);
    pixelOSDState = spracingPixelOSDLibraryVTable->getState();
    if ((pixelOSDState->flags & PIXELOSD_FLAG_INITIALISED) == 0) {
        return false;
    }

    return true;
}

#ifdef USE_SPRACING_PIXELOSD_DEBUG_OVERLAY
void spracingPixelOSDRenderDebugOverlay(uint8_t *frameBuffer, spracingPixelOSDFrameState_t* frameState, spracingPixelOSDSyncVoltages_t *syncVoltages)
{
    static const char *detectedVideoSystemNames[] = { "????", "PAL", "NTSC" };

    typedef enum {
        VIDEO_SYSTEM_UNKNOWN = 0,
        VIDEO_SYSTEM_PAL,
        VIDEO_SYSTEM_NTSC
    } spracingPixelOSDDetectedVideoSystem_e;

    static uint8_t messageBuffer[31];

    uint16_t debugY = FONT_MAX7456_HEIGHT * 13;

    char frameBufferCode1 = ' ';
    char frameBufferCode2 = ' ';
    if (frameBuffer_getBufferIndex(frameBuffer) == 0) {
      frameBufferCode1 = 'A';
    } else {
      frameBufferCode2 = 'B';
    }

    extern spracingPixelOSDState_t *pixelOSDState;

    spracingPixelOSDDetectedVideoSystem_e detectedVideoSystem = VIDEO_SYSTEM_UNKNOWN;
    if (pixelOSDState->flags & PIXELOSD_FLAG_NTSC_DETECTED) {
        detectedVideoSystem = VIDEO_SYSTEM_NTSC;
    } else if (pixelOSDState->flags & PIXELOSD_FLAG_PAL_DETECTED) {
        detectedVideoSystem = VIDEO_SYSTEM_PAL;
    }

    tfp_sprintf((char *)messageBuffer, "E:%04X PE:%04X VF:%04X M:%4s%c",
            frameState->frameErrorCounter,
            frameState->totalPulseErrors,
            frameState->validFrameCounter,
            detectedVideoSystemNames[detectedVideoSystem],
            frameBufferCode1
    );
    int messageLength = strlen((char *)messageBuffer);
    frameBuffer_slowWriteString(frameBuffer, (360 - (12 * messageLength)) / 2, debugY, messageBuffer, messageLength);

    debugY += FONT_MAX7456_HEIGHT;

    tfp_sprintf((char *)messageBuffer, "L:%04d FL:%04d FH:%04d T:%04d%c",
            syncVoltages->minimumLevelForLineThreshold,
            syncVoltages->minimumLevelForValidFrameMv,
            syncVoltages->maximumLevelForValidFrameMv,
            syncVoltages->syncThresholdMv,
            frameBufferCode2
    );
    messageLength = strlen((char *)messageBuffer);
    frameBuffer_slowWriteString(frameBuffer, (360 - (12 * messageLength)) / 2, debugY, messageBuffer, messageLength);

}
#endif // USE_SPRACING_PIXELOSD_DEBUG_OVERLAY

#endif // USE_SPRACING_PIXEL_OSD
