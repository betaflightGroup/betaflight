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

#include "drivers/nvic.h"
#include "drivers/io.h"
#include "dma.h"

#include "drivers/bus_spi.h"
#include "drivers/time.h"

#include "sdcard.h"
#include "sdcard_standard.h"

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    #define SDCARD_PROFILING
#endif

#define SDCARD_TIMEOUT_INIT_MILLIS                  200
#define SDCARD_MAX_CONSECUTIVE_FAILURES             8

typedef enum {
    // In these states we run at the initialization 400kHz clockspeed:
    SDCARD_STATE_NOT_PRESENT = 0,
    SDCARD_STATE_RESET,
    SDCARD_STATE_CARD_INIT_IN_PROGRESS,
    SDCARD_STATE_INITIALIZATION_RECEIVE_CID,

    // In these states we run at full clock speed
    SDCARD_STATE_READY,
    SDCARD_STATE_READING,
    SDCARD_STATE_SENDING_WRITE,
    SDCARD_STATE_WAITING_FOR_WRITE,
    SDCARD_STATE_WRITING_MULTIPLE_BLOCKS,
    SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE
} sdcardState_e;

typedef struct sdcard_t {
    struct {
        uint8_t *buffer;
        uint32_t blockIndex;
        uint8_t chunkIndex;

        sdcard_operationCompleteCallback_c callback;
        uint32_t callbackData;

#ifdef SDCARD_PROFILING
        uint32_t profileStartTime;
#endif
    } pendingOperation;

    uint32_t operationStartTime;

    uint8_t failureCount;

    uint8_t version;
    bool highCapacity;

    uint32_t multiWriteNextBlock;
    uint32_t multiWriteBlocksRemain;

    sdcardState_e state;

    sdcardMetadata_t metadata;
    sdcardCSD_t csd;

#ifdef SDCARD_PROFILING
    sdcard_profilerCallback_c profiler;
#endif
    bool enabled;
    bool detectionInverted;
    IO_t cardDetectPin;

#ifdef USE_SDCARD_SPI
    extDevice_t dev;
    uint8_t idleCount;
#endif

#ifdef USE_SDCARD_SDIO
    dmaIdentifier_e dmaIdentifier;
    uint8_t useCache;
#endif

    uint8_t dmaChannel;
} sdcard_t;

extern sdcard_t sdcard;

STATIC_ASSERT(sizeof(sdcardCSD_t) == 16, sdcard_csd_bitfields_didnt_pack_properly);

bool sdcard_isInserted(void);

typedef struct sdcardVTable_s {
    void (*sdcard_preInit)(const sdcardConfig_t *config);
    void (*sdcard_init)(const sdcardConfig_t *config, const spiPinConfig_t *spiConfig);
    bool (*sdcard_readBlock)(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);
    sdcardOperationStatus_e (*sdcard_beginWriteBlocks)(uint32_t blockIndex, uint32_t blockCount);
    sdcardOperationStatus_e (*sdcard_writeBlock)(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);
    bool (*sdcard_poll)(void);
    bool (*sdcard_isFunctional)(void);
    bool (*sdcard_isInitialized)(void);
    const sdcardMetadata_t* (*sdcard_getMetadata)(void);
#ifdef SDCARD_PROFILING
    void (*sdcardSdio_setProfilerCallback)(sdcard_profilerCallback_c callback);
#endif
} sdcardVTable_t;

#ifdef USE_SDCARD_SPI
extern sdcardVTable_t sdcardSpiVTable;
#endif
#ifdef USE_SDCARD_SDIO
extern sdcardVTable_t sdcardSdioVTable;
#endif
