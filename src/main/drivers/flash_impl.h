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
 * Author: jflyper
 */

#pragma once

#include "drivers/bus.h"
#include "drivers/dma.h"

struct flashVTable_s;

typedef enum {
    FLASHIO_NONE = 0,
    FLASHIO_SPI,
    FLASHIO_QUADSPI
} flashDeviceIoMode_e;

typedef struct flashDeviceIO_s {
    union {
        extDevice_t *dev; // Device interface dependent handle (spi/i2c)
    #ifdef USE_QUADSPI
        QUADSPI_TypeDef *quadSpi;
    #endif
    } handle;
    flashDeviceIoMode_e mode;
} flashDeviceIO_t;

typedef struct flashDevice_s {
    const struct flashVTable_s *vTable;
    flashGeometry_t geometry;
    uint32_t currentWriteAddress;
    bool isLargeFlash;
    // Whether we've performed an action that could have made the device busy
    // for writes. This allows us to avoid polling for writable status
    // when it is definitely ready already.
    bool couldBeBusy;
    uint32_t timeoutAt;
    flashDeviceIO_t io;
    void (*callback)(uint32_t arg);
    uint32_t callbackArg;
} flashDevice_t;

typedef struct flashVTable_s {
    bool (*isReady)(flashDevice_t *fdevice);
    bool (*waitForReady)(flashDevice_t *fdevice);
    void (*eraseSector)(flashDevice_t *fdevice, uint32_t address);
    void (*eraseCompletely)(flashDevice_t *fdevice);
    void (*pageProgramBegin)(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length));
    uint32_t (*pageProgramContinue)(flashDevice_t *fdevice, uint8_t const **buffers, uint32_t *bufferSizes, uint32_t bufferCount);
    void (*pageProgramFinish)(flashDevice_t *fdevice);
    void (*pageProgram)(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length));
    void (*flush)(flashDevice_t *fdevice);
    int (*readBytes)(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length);
    const flashGeometry_t *(*getGeometry)(flashDevice_t *fdevice);
} flashVTable_t;
