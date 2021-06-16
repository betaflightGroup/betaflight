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
 *
 * Author: Dominic Clifton - Initial implementation and testing.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_FLASH_W25Q128FV) && defined(USE_QUADSPI)

#define USE_FLASH_WRITES_USING_4LINES
#define USE_FLASH_READS_USING_4LINES

#include "build/debug.h"
#include "common/utils.h"

#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/flash_w25q128fv.h"
#include "drivers/bus_quadspi.h"

// JEDEC ID
#define JEDEC_ID_WINBOND_W25Q128FV_SPI      0xEF4018
#define JEDEC_ID_WINBOND_W25Q128FV_QUADSPI  0xEF6018
#define JEDEC_ID_WINBOND_W25Q128JV_QUADSPI  0xEF7018

// Device size parameters
#define W25Q128FV_PAGE_SIZE         2048
#define W25Q128FV_PAGES_PER_BLOCK   64
#define W25Q128FV_BLOCKS_PER_DIE 1024
#define W25Q128FV_BLOCK_SIZE (W25Q128FV_PAGES_PER_BLOCK * W25Q128FV_PAGE_SIZE)

// Sizes
#define W25Q128FV_STATUS_REGISTER_BITS        8
#define W25Q128FV_ADDRESS_BITS                24


// Instructions
#define W25Q128FV_INSTRUCTION_RDID             0x9F

#define W25Q128FV_INSTRUCTION_ENABLE_RESET     0x66
#define W25Q128FV_INSTRUCTION_RESET_DEVICE     0x99

#define W25Q128FV_INSTRUCTION_READ_STATUS1_REG  0x05
#define W25Q128FV_INSTRUCTION_READ_STATUS2_REG  0x35
#define W25Q128FV_INSTRUCTION_READ_STATUS3_REG  0x15

#define W25Q128FV_INSTRUCTION_WRITE_STATUS1_REG 0x01
#define W25Q128FV_INSTRUCTION_WRITE_STATUS2_REG 0x31
#define W25Q128FV_INSTRUCTION_WRITE_STATUS3_REG 0x11

#define W25Q128FV_INSTRUCTION_WRITE_ENABLE       0x06
#define W25Q128FV_INSTRUCTION_VOLATILE_WRITE_ENABLE 0x50
#define W25Q128FV_INSTRUCTION_BLOCK_ERASE_64KB   0xD8
#define W25Q128FV_INSTRUCTION_CHIP_ERASE         0xC7

#define W25Q128FV_INSTRUCTION_ENTER_QPI_MODE     0x38

#define W25Q128FV_INSTRUCTION_FAST_READ          0x0B
#define W25Q128FV_INSTRUCTION_FAST_READ_QUAD_OUTPUT 0x6B

#define W25Q128FV_INSTRUCTION_PAGE_PROGRAM       0x02
#define W25Q128FV_INSTRUCTION_QUAD_PAGE_PROGRAM  0x32

#define W25Q128FV_SR1_BIT_WRITE_IN_PROGRESS     (1 << 0)
#define W25Q128FV_SR1_BIT_WRITE_ENABLED         (1 << 1)

#define W25Q128FV_SR2_BIT_QUAD_ENABLE           (1 << 1)


//#define W25Q128FV_INSTRUCTION_WRITE_DISABLE    0x04
//#define W25Q128FV_INSTRUCTION_PAGE_PROGRAM     0x02

// Timings (2ms minimum to avoid 1 tick advance in consecutive calls to HAL_GetTick).
#define W25Q128FV_TIMEOUT_PAGE_READ_MS          4
#define W25Q128FV_TIMEOUT_RESET_MS              2           // tRST = 30us
#define W25Q128FV_TIMEOUT_BLOCK_ERASE_64KB_MS   2000        // tBE2max = 2000ms, tBE2typ = 150ms
#define W25Q128FV_TIMEOUT_CHIP_ERASE_MS        (200 * 1000) // tCEmax 200s, tCEtyp = 40s

#define W25Q128FV_TIMEOUT_PAGE_PROGRAM_MS       2           // tPPmax = 700us, tPPtyp = 250us
#define W25Q128FV_TIMEOUT_WRITE_ENABLE_MS       2


typedef enum {
    INITIAL_MODE_SPI = 0,
    INITIAL_MODE_QUADSPI,
} w25q128fv_initialMode_e;

typedef struct w25q128fvState_s {
    w25q128fv_initialMode_e initialMode;
    uint32_t currentWriteAddress;
} w25q128fvState_t;

w25q128fvState_t w25q128fvState = { 0 };

static bool w25q128fv_waitForReady(flashDevice_t *fdevice);
static void w25q128fv_waitForTimeout(flashDevice_t *fdevice);

static void w25q128fv_setTimeout(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t now = HAL_GetTick();
    fdevice->timeoutAt = now + timeoutMillis;
}

static void w25q128fv_performOneByteCommand(flashDeviceIO_t *io, uint8_t command)
{
    QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;
    quadSpiTransmit1LINE(quadSpi, command, 0, NULL, 0);
}

static void w25q128fv_performCommandWithAddress(flashDeviceIO_t *io, uint8_t command, uint32_t address)
{
    QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

    quadSpiInstructionWithAddress1LINE(quadSpi, command, 0, address & 0xffffff, W25Q128FV_ADDRESS_BITS);
}

static void w25q128fv_writeEnable(flashDevice_t *fdevice)
{
    w25q128fv_performOneByteCommand(&fdevice->io, W25Q128FV_INSTRUCTION_WRITE_ENABLE);
}

static uint8_t w25q128fv_readRegister(flashDeviceIO_t *io, uint8_t command)
{
    QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

    uint8_t in[1];
    quadSpiReceive1LINE(quadSpi, command, 0, in, W25Q128FV_STATUS_REGISTER_BITS / 8);

    return in[0];
}

static void w25q128fv_writeRegister(flashDeviceIO_t *io, uint8_t command, uint8_t data)
{
    QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

   quadSpiTransmit1LINE(quadSpi, command, 0, &data, W25Q128FV_STATUS_REGISTER_BITS / 8);
}

static void w25q128fv_deviceReset(flashDevice_t *fdevice)
{
    flashDeviceIO_t *io = &fdevice->io;

    w25q128fv_waitForReady(fdevice);
    w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_ENABLE_RESET);
    w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_RESET_DEVICE);

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_RESET_MS);
    w25q128fv_waitForTimeout(fdevice);

    w25q128fv_waitForReady(fdevice);

#ifdef DISABLE_NONVOLATILE_QE_MODE // Use this if you encounter a chip with it's QE bit enabled when it shouldn't be.
    w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_WRITE_ENABLE);
    w25q128fv_writeRegister(io, W25Q128FV_INSTRUCTION_WRITE_STATUS2_REG, 0x00);
#endif


#if defined(USE_FLASH_WRITES_USING_4LINES) || defined(USE_FLASH_READS_USING_4LINES)
    uint8_t registerValue = w25q128fv_readRegister(io, W25Q128FV_INSTRUCTION_READ_STATUS2_REG);

    //
    // WARNING: DO NOT ENABLE QE bit if IO2/IO3 are connected to GND or VCC.
    //
    // See datasheet https://www.winbond.com/resource-files/w25q128fv%20rev.m%2005132016%20kms.pdf
    // W25Q128FV - Revision M - 7.1.10 Quad Enable
    //
    // There is no such warning for the W25Q128JV in the same documentation section
    // See datasheet https://www.winbond.com/resource-files/w25q128jv%20revg%2004082019%20plus.pdf
    // W25Q128JV - Revision G - 7.1.4 Quad Enable
    //

    if ((registerValue & W25Q128FV_SR2_BIT_QUAD_ENABLE) == 0) {
        // Enable QUADSPI mode.
        registerValue = w25q128fv_readRegister(io, W25Q128FV_INSTRUCTION_READ_STATUS2_REG);

        uint8_t newValue = registerValue;
        newValue |= W25Q128FV_SR2_BIT_QUAD_ENABLE;

        //w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_WRITE_ENABLE);
        w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_VOLATILE_WRITE_ENABLE);
        w25q128fv_writeRegister(io, W25Q128FV_INSTRUCTION_WRITE_STATUS2_REG, newValue);
    }
#endif
}

bool w25q128fv_isReady(flashDevice_t *fdevice)
{
    uint8_t status = w25q128fv_readRegister(&fdevice->io, W25Q128FV_INSTRUCTION_READ_STATUS1_REG);

    bool busy = (status & W25Q128FV_SR1_BIT_WRITE_IN_PROGRESS);

    return !busy;
}

static bool w25q128fv_isWritable(flashDevice_t *fdevice)
{
    uint8_t status = w25q128fv_readRegister(&fdevice->io, W25Q128FV_INSTRUCTION_READ_STATUS1_REG);

    bool writable = (status & W25Q128FV_SR1_BIT_WRITE_ENABLED);

    return writable;
}

bool w25q128fv_hasTimedOut(flashDevice_t *fdevice)
{
    uint32_t now = HAL_GetTick();
    if (cmp32(now, fdevice->timeoutAt) >= 0) {
        return true;
    }
    return false;
}

void w25q128fv_waitForTimeout(flashDevice_t *fdevice)
{
   while (!w25q128fv_hasTimedOut(fdevice)) { }

   fdevice->timeoutAt = 0;
}

bool w25q128fv_waitForReady(flashDevice_t *fdevice)
{
    bool ready = true;
    while (!w25q128fv_isReady(fdevice)) {
        if (w25q128fv_hasTimedOut(fdevice)) {
            ready = false;
            break;
        }
    }
    fdevice->timeoutAt = 0;

    return ready;
}

const flashVTable_t w25q128fv_vTable;

static void w25q128fv_deviceInit(flashDevice_t *flashdev);

bool w25q128fv_detect(flashDevice_t *fdevice, uint32_t chipID)
{
    switch (chipID) {
    case JEDEC_ID_WINBOND_W25Q128FV_SPI:
    case JEDEC_ID_WINBOND_W25Q128FV_QUADSPI:
    case JEDEC_ID_WINBOND_W25Q128JV_QUADSPI:
        fdevice->geometry.sectors           = 256;
        fdevice->geometry.pagesPerSector    = 256;
        fdevice->geometry.pageSize          = 256;
        break;

    default:
        // Unsupported chip
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    // use the chip id to determine the initial interface mode on cold-boot.
    switch (chipID) {
    case JEDEC_ID_WINBOND_W25Q128FV_SPI:
        w25q128fvState.initialMode = INITIAL_MODE_SPI;
        break;

    case JEDEC_ID_WINBOND_W25Q128JV_QUADSPI:
    case JEDEC_ID_WINBOND_W25Q128FV_QUADSPI:
        w25q128fvState.initialMode = INITIAL_MODE_QUADSPI;
        break;

    default:
        break;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    w25q128fv_deviceReset(fdevice);

    w25q128fv_deviceInit(fdevice);

    fdevice->vTable = &w25q128fv_vTable;

    return true;
}

static void w25q128fv_eraseSector(flashDevice_t *fdevice, uint32_t address)
{

    w25q128fv_waitForReady(fdevice);

    w25q128fv_writeEnable(fdevice);

    w25q128fv_performCommandWithAddress(&fdevice->io, W25Q128FV_INSTRUCTION_BLOCK_ERASE_64KB, address);
    
    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_BLOCK_ERASE_64KB_MS);
}

static void w25q128fv_eraseCompletely(flashDevice_t *fdevice)
{
    w25q128fv_waitForReady(fdevice);

    w25q128fv_writeEnable(fdevice);

    w25q128fv_performOneByteCommand(&fdevice->io, W25Q128FV_INSTRUCTION_CHIP_ERASE);

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_CHIP_ERASE_MS);
}


static void w25q128fv_loadProgramData(flashDevice_t *fdevice, const uint8_t *data, int length)
{
    w25q128fv_waitForReady(fdevice);

    QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

#ifdef USE_FLASH_WRITES_USING_4LINES
    quadSpiTransmitWithAddress4LINES(quadSpi, W25Q128FV_INSTRUCTION_QUAD_PAGE_PROGRAM, 0, w25q128fvState.currentWriteAddress, W25Q128FV_ADDRESS_BITS, data, length);
#else
    quadSpiTransmitWithAddress1LINE(quadSpi, W25Q128FV_INSTRUCTION_PAGE_PROGRAM, 0, w25q128fvState.currentWriteAddress, W25Q128FV_ADDRESS_BITS, data, length);
#endif

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_PAGE_PROGRAM_MS);

    w25q128fvState.currentWriteAddress += length;
}

static void w25q128fv_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length))
{
    fdevice->callback = callback;
    w25q128fvState.currentWriteAddress = address;
}

static uint32_t w25q128fv_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    for (uint32_t i = 0; i < bufferCount; i++) {
        w25q128fv_waitForReady(fdevice);

        w25q128fv_writeEnable(fdevice);

        // verify write enable is set.
        w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_WRITE_ENABLE_MS);
        bool writable = false;
        do {
            writable = w25q128fv_isWritable(fdevice);
        } while (!writable && w25q128fv_hasTimedOut(fdevice));

        if (!writable) {
            return 0; // TODO report failure somehow.
        }

        w25q128fv_loadProgramData(fdevice, buffers[i], bufferSizes[i]);
    }

    return fdevice->callbackArg;
}

static void w25q128fv_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

static void w25q128fv_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    w25q128fv_pageProgramBegin(fdevice, address, callback);
    w25q128fv_pageProgramContinue(fdevice, &data, &length, 1);
    w25q128fv_pageProgramFinish(fdevice);
}

void w25q128fv_flush(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

static int w25q128fv_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (!w25q128fv_waitForReady(fdevice)) {
        return 0;
    }

    QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;
#ifdef USE_FLASH_READS_USING_4LINES
    bool status = quadSpiReceiveWithAddress4LINES(quadSpi, W25Q128FV_INSTRUCTION_FAST_READ_QUAD_OUTPUT, 8, address, W25Q128FV_ADDRESS_BITS, buffer, length);
#else
    bool status = quadSpiReceiveWithAddress1LINE(quadSpi, W25Q128FV_INSTRUCTION_FAST_READ, 8, address, W25Q128FV_ADDRESS_BITS, buffer, length);
#endif
    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_PAGE_READ_MS);

    if (!status) {
        return 0;
    }

    return length;
}



const flashGeometry_t* w25q128fv_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t w25q128fv_vTable = {
    .isReady = w25q128fv_isReady,
    .waitForReady = w25q128fv_waitForReady,
    .eraseSector = w25q128fv_eraseSector,
    .eraseCompletely = w25q128fv_eraseCompletely,
    .pageProgramBegin = w25q128fv_pageProgramBegin,
    .pageProgramContinue = w25q128fv_pageProgramContinue,
    .pageProgramFinish = w25q128fv_pageProgramFinish,
    .pageProgram = w25q128fv_pageProgram,
    .flush = w25q128fv_flush,
    .readBytes = w25q128fv_readBytes,
    .getGeometry = w25q128fv_getGeometry,
};

static void w25q128fv_deviceInit(flashDevice_t *flashdev)
{
    UNUSED(flashdev);
}

#endif
