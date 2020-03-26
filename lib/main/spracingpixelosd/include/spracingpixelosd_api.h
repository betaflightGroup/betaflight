/*-
 * Copyright (c) 2020 Dominic Clifton
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * SP Racing Pixel OSD Library by Dominic Clifton.
 */

#pragma once

#include <spracingpixelosd_conf.h>

#define PIXELOSD_FLAG_INITIALISED       (1 << 0)
#define PIXELOSD_FLAG_ERROR             (1 << 1)
#define PIXELOSD_FLAG_SERVICE_REQUIRED  (1 << 2)
#define PIXELOSD_FLAG_VSYNC             (1 << 3)
#define PIXELOSD_FLAG_FIELD_SYNC        (1 << 4)
#define PIXELOSD_FLAG_NTSC_DETECTED     (1 << 5)
#define PIXELOSD_FLAG_PAL_DETECTED      (1 << 6)

// CF = Configuration Flag
#define PIXELOSD_CF_VIDEO_SYSTEM_PAL    (1 << 0)
#define PIXELOSD_CF_VIDEO_SYSTEM_NTSC   (0 << 1)

// EC = ErrorCode
#define PIXELOSD_EC_UNSUPPORTED_TIMER_BUS_CLK       1

typedef struct spracingPixelOSDFrameState_s {
    uint16_t frameErrorCounter;
    uint16_t validFrameCounter;
    uint16_t totalPulseErrors;
} spracingPixelOSDFrameState_t;

typedef struct spracingPixelOSDSyncVoltages_s {
    uint16_t minimumLevelForLineThreshold;
    uint16_t minimumLevelForValidFrameMv;
    uint16_t maximumLevelForValidFrameMv;
    uint16_t syncThresholdMv;
} spracingPixelOSDSyncVoltages_t;

typedef struct spracingPixelOSDState_s {
    uint32_t flags;
    uint16_t errorCode;
} spracingPixelOSDState_t;

typedef struct pixelOSDDefaultConfig_s {
  uint32_t flags;
} spracingPixelOSDDefaultConfig_t;

typedef struct spracingPixelOSDHostAPI_s {
  // called by the OSD system when it needs to know the time, in microseconds.
  uint32_t (*micros)(void);

  // called by the OSD system when the next frame is about to be rendered
  // this is an ISR callback, primarily used by the OSD client system to perform the following tasks:
  // 1) swap frame buffers
  // 2) signal to the rest of the system that the previous framebuffer can now be rendered into.
  void (*onVSync)(void);
} spracingPixelOSDHostAPI_t;

typedef struct spracingPixelOSDLibraryVTable_s {
    // call once at system startup, non-reentrant.
    void (*init)(const spracingPixelOSDHostAPI_t *hostAPI, const spracingPixelOSDDefaultConfig_t *defaultConfig);

    // call once after initialisation, store the state handle for future use, contents not valid until `refreshState` is called.
    // memory for pixelOSDState_t is managed by OSD system
    spracingPixelOSDState_t *(*getState)(void);

    // call periodically and act on the updated spracingPixelOSDState obtained by spracingPixelOSDState_t as appropriate
    void (*refreshState)(uint32_t currentTimeUs);

    // memory for spracingPixelOSDFrameState_t is managed by OSD client system
    void (*refreshFrameState)(spracingPixelOSDFrameState_t *);

    // memory for spracingPixelOSDFrameState_t is managed by OSD system
    spracingPixelOSDSyncVoltages_t *(*getSyncVoltages)(void);

    // call when indicated by the PIXELOSD_FLAG_SERVICE_REQUIRED
    void (*service)(uint32_t currentTimeUs);

    // call once the framebuffer has been prepared
    void (*frameBufferCommit)(uint8_t *frameBuffer);

    void (*comparatorIRQHandler)(void);
    void (*syncDMAHandler)(void);
    void (*pixelDMAHandler)(void);

} spracingPixelOSDLibraryVTable_t;

typedef struct spracingPixelOSDLibraryDescriptor_s {
    int16_t apiVersion;
    int16_t code;
} spracingPixelOSDLibraryDescriptor_t;

#define SPRACINGPIXELOSD_LIBRARY_CODE 0x4f30
#define SPRACINGPIXELOSD_LIBRARY_API_VERSION 0x01

