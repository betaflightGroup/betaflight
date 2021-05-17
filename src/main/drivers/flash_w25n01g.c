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
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_FLASH_W25N01G

#include "flash.h"
#include "flash_impl.h"
#include "flash_w25n01g.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/io.h"
#include "drivers/time.h"

// Device size parameters
#define W25N01G_PAGE_SIZE         2048
#define W25N01G_PAGES_PER_BLOCK   64
#define W25N01G_BLOCKS_PER_DIE 1024

// BB replacement area
#define W25N01G_BB_MARKER_BLOCKS           1
#define W25N01G_BB_REPLACEMENT_BLOCKS      20
#define W25N01G_BB_MANAGEMENT_BLOCKS       (W25N01G_BB_REPLACEMENT_BLOCKS + W25N01G_BB_MARKER_BLOCKS)
// blocks are zero-based index
#define W25N01G_BB_REPLACEMENT_START_BLOCK (W25N01G_BLOCKS_PER_DIE - W25N01G_BB_REPLACEMENT_BLOCKS)
#define W25N01G_BB_MANAGEMENT_START_BLOCK  (W25N01G_BLOCKS_PER_DIE - W25N01G_BB_MANAGEMENT_BLOCKS)
#define W25N01G_BB_MARKER_BLOCK            (W25N01G_BB_REPLACEMENT_START_BLOCK - W25N01G_BB_MARKER_BLOCKS)

// Instructions

#define W25N01G_INSTRUCTION_RDID             0x9F
#define W25N01G_INSTRUCTION_DEVICE_RESET     0xFF
#define W25N01G_INSTRUCTION_READ_STATUS_REG  0x05
#define W25N01G_INSTRUCTION_READ_STATUS_ALTERNATE_REG  0x0F
#define W25N01G_INSTRUCTION_WRITE_STATUS_REG 0x01
#define W25N01G_INSTRUCTION_WRITE_STATUS_ALTERNATE_REG 0x1F
#define W25N01G_INSTRUCTION_WRITE_ENABLE     0x06
#define W25N01G_INSTRUCTION_DIE_SELECT       0xC2
#define W25N01G_INSTRUCTION_BLOCK_ERASE      0xD8
#define W25N01G_INSTRUCTION_READ_BBM_LUT     0xA5
#define W25N01G_INSTRUCTION_BB_MANAGEMENT    0xA1
#define W25N01G_INSTRUCTION_PROGRAM_DATA_LOAD        0x02
#define W25N01G_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD 0x84
#define W25N01G_INSTRUCTION_PROGRAM_EXECUTE  0x10
#define W25N01G_INSTRUCTION_PAGE_DATA_READ   0x13
#define W25N01G_INSTRUCTION_READ_DATA        0x03
#define W25N01G_INSTRUCTION_FAST_READ        0x1B
#define W25N01G_INSTRUCTION_FAST_READ_QUAD_OUTPUT 0x6B

// Config/status register addresses
#define W25N01G_PROT_REG 0xA0
#define W25N01G_CONF_REG 0xB0
#define W25N01G_STAT_REG 0xC0

// Bits in config/status register 1 (W25N01G_PROT_REG)
#define W25N01G_PROT_CLEAR                (0)
#define W25N01G_PROT_SRP1_ENABLE          (1 << 0)
#define W25N01G_PROT_WP_E_ENABLE          (1 << 1)
#define W25N01G_PROT_TB_ENABLE            (1 << 2)
#define W25N01G_PROT_PB0_ENABLE           (1 << 3)
#define W25N01G_PROT_PB1_ENABLE           (1 << 4)
#define W25N01G_PROT_PB2_ENABLE           (1 << 5)
#define W25N01G_PROT_PB3_ENABLE           (1 << 6)
#define W25N01G_PROT_SRP2_ENABLE          (1 << 7)

// Bits in config/status register 2 (W25N01G_CONF_REG)
#define W25N01G_CONFIG_ECC_ENABLE         (1 << 4)
#define W25N01G_CONFIG_BUFFER_READ_MODE   (1 << 3)

// Bits in config/status register 3 (W25N01G_STATREG)
#define W25N01G_STATUS_BBM_LUT_FULL       (1 << 6)
#define W25N01G_STATUS_FLAG_ECC_POS       4
#define W25N01G_STATUS_FLAG_ECC_MASK      ((1 << 5)|(1 << 4))
#define W25N01G_STATUS_FLAG_ECC(status)   (((status) & W25N01G_STATUS_FLAG_ECC_MASK) >> 4)
#define W25N01G_STATUS_PROGRAM_FAIL       (1 << 3)
#define W25N01G_STATUS_ERASE_FAIL         (1 << 2)
#define W25N01G_STATUS_FLAG_WRITE_ENABLED (1 << 1)
#define W25N01G_STATUS_FLAG_BUSY          (1 << 0)

#define W25N01G_BBLUT_TABLE_ENTRY_COUNT     20
#define W25N01G_BBLUT_TABLE_ENTRY_SIZE      4  // in bytes

// Bits in LBA for BB LUT
#define W25N01G_BBLUT_STATUS_ENABLED (1 << 15)
#define W25N01G_BBLUT_STATUS_INVALID (1 << 14)
#define W25N01G_BBLUT_STATUS_MASK    (W25N01G_BBLUT_STATUS_ENABLED | W25N01G_BBLUT_STATUS_INVALID)

// Some useful defs and macros
#define W25N01G_LINEAR_TO_COLUMN(laddr) ((laddr) % W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_PAGE(laddr) ((laddr) / W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_BLOCK(laddr) (W25N01G_LINEAR_TO_PAGE(laddr) / W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_PAGE(block) ((block) * W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_LINEAR(block) (W25N01G_BLOCK_TO_PAGE(block) * W25N01G_PAGE_SIZE)

// IMPORTANT: Timeout values are currently required to be set to the highest value required by any of the supported flash chips by this driver

// The timeout values (2ms minimum to avoid 1 tick advance in consecutive calls to millis).
#define W25N01G_TIMEOUT_PAGE_READ_MS        2   // tREmax = 60us (ECC enabled)
#define W25N01G_TIMEOUT_PAGE_PROGRAM_MS     2   // tPPmax = 700us
#define W25N01G_TIMEOUT_BLOCK_ERASE_MS      15  // tBEmax = 10ms
#define W25N01G_TIMEOUT_RESET_MS            500 // tRSTmax = 500ms

// Sizes (in bits)
#define W28N01G_STATUS_REGISTER_SIZE        8
#define W28N01G_STATUS_PAGE_ADDRESS_SIZE    16
#define W28N01G_STATUS_COLUMN_ADDRESS_SIZE  16

typedef struct bblut_s {
    uint16_t pba;
    uint16_t lba;
} bblut_t;

static bool w25n01g_waitForReady(flashDevice_t *fdevice);

static void w25n01g_setTimeout(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t now = millis();
    fdevice->timeoutAt = now + timeoutMillis;
}

/**
 * Send the given command byte to the device.
 */
static void w25n01g_performOneByteCommand(flashDeviceIO_t *io, uint8_t command)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        busSegment_t segments[] = {
                {&command, NULL, sizeof (command), true, NULL},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWaitClaim(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;
        quadSpiTransmit1LINE(quadSpi, command, 0, NULL, 0);
    }
#endif
}

static void w25n01g_performCommandWithPageAddress(flashDeviceIO_t *io, uint8_t command, uint32_t pageAddress)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        uint8_t cmd[] = { command, 0, (pageAddress >> 8) & 0xff, (pageAddress >> 0) & 0xff};

        busSegment_t segments[] = {
                {cmd, NULL, sizeof (cmd), true, NULL},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWaitClaim(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

        quadSpiInstructionWithAddress1LINE(quadSpi, command, 0, pageAddress & 0xffff, W28N01G_STATUS_PAGE_ADDRESS_SIZE + 8);
    }
#endif
}

static uint8_t w25n01g_readRegister(flashDeviceIO_t *io, uint8_t reg)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        uint8_t cmd[3] = { W25N01G_INSTRUCTION_READ_STATUS_REG, reg, 0 };
        uint8_t in[3];

        busSegment_t segments[] = {
                {cmd, in, sizeof (cmd), true, NULL},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);

        return in[2];
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {

        QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

        uint8_t in[1];
        quadSpiReceiveWithAddress1LINE(quadSpi, W25N01G_INSTRUCTION_READ_STATUS_REG, 0, reg, W28N01G_STATUS_REGISTER_SIZE, in, sizeof(in));

        return in[0];
    }
#endif
    return 0;
}

static void w25n01g_writeRegister(flashDeviceIO_t *io, uint8_t reg, uint8_t data)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;
        uint8_t cmd[3] = { W25N01G_INSTRUCTION_WRITE_STATUS_REG, reg, data };

        busSegment_t segments[] = {
                {cmd, NULL, sizeof (cmd), true, NULL},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
   }
#ifdef USE_QUADSPI
   else if (io->mode == FLASHIO_QUADSPI) {
       QUADSPI_TypeDef *quadSpi = io->handle.quadSpi;

       quadSpiTransmitWithAddress1LINE(quadSpi, W25N01G_INSTRUCTION_WRITE_STATUS_REG, 0, reg, W28N01G_STATUS_REGISTER_SIZE, &data, 1);
   }
#endif
}


static void w25n01g_deviceReset(flashDevice_t *fdevice)
{
    flashDeviceIO_t *io = &fdevice->io;

    w25n01g_performOneByteCommand(io, W25N01G_INSTRUCTION_DEVICE_RESET);

    w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_RESET_MS);
    w25n01g_waitForReady(fdevice);

    // Protection for upper 1/32 (BP[3:0] = 0101, TB=0), WP-E on; to protect bad block replacement area
    // DON'T DO THIS. This will prevent writes through the bblut as well.
    // w25n01g_writeRegister(dev, W25N01G_PROT_REG, W25N01G_PROT_PB0_ENABLE|W25N01G_PROT_PB2_ENABLE|W25N01G_PROT_WP_E_ENABLE);

    // No protection, WP-E off, WP-E prevents use of IO2
    w25n01g_writeRegister(io, W25N01G_PROT_REG, W25N01G_PROT_CLEAR);

    // Buffered read mode (BUF = 1), ECC enabled (ECC = 1)
    w25n01g_writeRegister(io, W25N01G_CONF_REG, W25N01G_CONFIG_ECC_ENABLE|W25N01G_CONFIG_BUFFER_READ_MODE);
}

bool w25n01g_isReady(flashDevice_t *fdevice)
{
    uint8_t status = w25n01g_readRegister(&fdevice->io, W25N01G_STAT_REG);

    return ((status & W25N01G_STATUS_FLAG_BUSY) == 0);
}

static bool w25n01g_waitForReady(flashDevice_t *fdevice)
{
    while (!w25n01g_isReady(fdevice)) {
        uint32_t now = millis();
        if (cmp32(now, fdevice->timeoutAt) >= 0) {
            return false;
        }
    }
    fdevice->timeoutAt = 0;

    return true;
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
static void w25n01g_writeEnable(flashDevice_t *fdevice)
{
    w25n01g_performOneByteCommand(&fdevice->io, W25N01G_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    fdevice->couldBeBusy = true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */
const flashVTable_t w25n01g_vTable;

static void w25n01g_deviceInit(flashDevice_t *flashdev);

bool w25n01g_detect(flashDevice_t *fdevice, uint32_t chipID)
{
    switch (chipID) {
    case JEDEC_ID_WINBOND_W25N01GV:
        fdevice->geometry.sectors = 1024;      // Blocks
        fdevice->geometry.pagesPerSector = 64; // Pages/Blocks
        fdevice->geometry.pageSize = 2048;
        break;

    default:
        // Unsupported chip
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;

        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NAND;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    flashPartitionSet(FLASH_PARTITION_TYPE_BADBLOCK_MANAGEMENT,
            W25N01G_BB_MANAGEMENT_START_BLOCK,
            W25N01G_BB_MANAGEMENT_START_BLOCK + W25N01G_BB_MANAGEMENT_BLOCKS - 1);

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be

    w25n01g_deviceReset(fdevice);

    // Upper 4MB (32 blocks * 128KB/block) will be used for bad block replacement area.

    // Blocks in this area are only written through bad block LUT,
    // and factory written bad block marker in unused blocks are retained.

    // When a replacement block is required,
    // (1) "Read BB LUT" command is used to obtain the last block mapped,
    // (2) blocks after the last block is scanned for a good block,
    // (3) the first good block is used for replacement, and the BB LUT is updated.

    // There are only 20 BB LUT entries, and there are 32 replacement blocks.
    // There will be a least chance of running out of replacement blocks.
    // If it ever run out, the device becomes unusable.

    w25n01g_deviceInit(fdevice);

    fdevice->vTable = &w25n01g_vTable;

    return true;
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
void w25n01g_eraseSector(flashDevice_t *fdevice, uint32_t address)
{

    w25n01g_waitForReady(fdevice);

    w25n01g_writeEnable(fdevice);

    w25n01g_performCommandWithPageAddress(&fdevice->io, W25N01G_INSTRUCTION_BLOCK_ERASE, W25N01G_LINEAR_TO_PAGE(address));

    w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_BLOCK_ERASE_MS);
}

//
// W25N01G does not support full chip erase.
// Call eraseSector repeatedly.

void w25n01g_eraseCompletely(flashDevice_t *fdevice)
{
    for (uint32_t block = 0; block < fdevice->geometry.sectors; block++) {
        w25n01g_eraseSector(fdevice, W25N01G_BLOCK_TO_LINEAR(block));
    }
}

static void w25n01g_programDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{

    w25n01g_waitForReady(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;
        uint8_t cmd[] = { W25N01G_INSTRUCTION_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress & 0xff };

         busSegment_t segments[] = {
                 {cmd, NULL, sizeof (cmd), true, NULL},
                 {(uint8_t *)data, NULL, length, true, NULL},
                 {NULL, NULL, 0, true, NULL},
         };

         // Ensure any prior DMA has completed before continuing
         spiWaitClaim(dev);

         spiSequence(dev, &segments[0]);

         // Block pending completion of SPI access
         spiWait(dev);
   }
#ifdef USE_QUADSPI
   else if (fdevice->io.mode == FLASHIO_QUADSPI) {
       QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

       quadSpiTransmitWithAddress1LINE(quadSpi, W25N01G_INSTRUCTION_PROGRAM_DATA_LOAD, 0, columnAddress, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, data, length);
    }
#endif

    w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);
}

static void w25n01g_randomProgramDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{
    uint8_t cmd[] = { W25N01G_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress & 0xff };

    w25n01g_waitForReady(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        busSegment_t segments[] = {
                {cmd, NULL, sizeof (cmd), true, NULL},
                {(uint8_t *)data, NULL, length, true, NULL},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWaitClaim(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        quadSpiTransmitWithAddress1LINE(quadSpi, W25N01G_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, 0, columnAddress, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, data, length);
     }
#endif

    w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

}

static void w25n01g_programExecute(flashDevice_t *fdevice, uint32_t pageAddress)
{
    w25n01g_waitForReady(fdevice);

    w25n01g_performCommandWithPageAddress(&fdevice->io, W25N01G_INSTRUCTION_PROGRAM_EXECUTE, pageAddress);

    w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);
}

//
// Writes are done in three steps:
// (1) Load internal data buffer with data to write
//     - We use "Random Load Program Data", as "Load Program Data" resets unused data bytes in the buffer to 0xff.
//     - Each "Random Load Program Data" instruction must be accompanied by at least a single data.
//     - Each "Random Load Program Data" instruction terminates at the rising of CS.
// (2) Enable write
// (3) Issue "Execute Program"
//

/*
flashfs page program behavior
- Single program never crosses page boundary.
- Except for this characteristic, it program arbitral size.
- Write address is, naturally, not a page boundary.

To cope with this behavior.

pageProgramBegin:
If buffer is dirty and programLoadAddress != address, then the last page is a partial write;
issue PAGE_PROGRAM_EXECUTE to flash buffer contents, clear dirty and record the address as programLoadAddress and programStartAddress.
Else do nothing.

pageProgramContinue:
Mark buffer as dirty.
If programLoadAddress is on page boundary, then issue PROGRAM_LOAD_DATA, else issue RANDOM_PROGRAM_LOAD_DATA.
Update programLoadAddress.
Optionally observe the programLoadAddress, and if it's on page boundary, issue PAGE_PROGRAM_EXECUTE.

pageProgramFinish:
Observe programLoadAddress. If it's on page boundary, issue PAGE_PROGRAM_EXECUTE and clear dirty, else just return.
If pageProgramContinue observes the page boundary, then do nothing(?).
*/

static uint32_t programStartAddress;
static uint32_t programLoadAddress;
bool bufferDirty = false;
bool isProgramming = false;

void w25n01g_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length))
{
    fdevice->callback = callback;

    if (bufferDirty) {
        if (address != programLoadAddress) {
            w25n01g_waitForReady(fdevice);

            isProgramming = false;

            w25n01g_writeEnable(fdevice);

            w25n01g_programExecute(fdevice, W25N01G_LINEAR_TO_PAGE(programStartAddress));

            bufferDirty = false;
            isProgramming = true;
        }
    } else {
        programStartAddress = programLoadAddress = address;
    }
}

uint32_t w25n01g_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (bufferCount < 1) {
        fdevice->callback(0);
        return 0;
    }

    w25n01g_waitForReady(fdevice);

    w25n01g_writeEnable(fdevice);

    isProgramming = false;

    if (!bufferDirty) {
        w25n01g_programDataLoad(fdevice, W25N01G_LINEAR_TO_COLUMN(programLoadAddress), buffers[0], bufferSizes[0]);
    } else {
        w25n01g_randomProgramDataLoad(fdevice, W25N01G_LINEAR_TO_COLUMN(programLoadAddress), buffers[0], bufferSizes[0]);
    }

    // XXX Test if write enable is reset after each data loading.

    bufferDirty = true;
    programLoadAddress += bufferSizes[0];

    if (fdevice->callback) {
        fdevice->callback(bufferSizes[0]);
    }

    return bufferSizes[0];
}

static uint32_t currentPage = UINT32_MAX;

void w25n01g_pageProgramFinish(flashDevice_t *fdevice)
{
    if (bufferDirty && W25N01G_LINEAR_TO_COLUMN(programLoadAddress) == 0) {

        currentPage = W25N01G_LINEAR_TO_PAGE(programStartAddress); // reset page to the page being written

        w25n01g_programExecute(fdevice, W25N01G_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;

        programStartAddress = programLoadAddress;
    }
}

/**
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */

void w25n01g_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    w25n01g_pageProgramBegin(fdevice, address, callback);
    w25n01g_pageProgramContinue(fdevice, &data, &length, 1);
    w25n01g_pageProgramFinish(fdevice);
}

void w25n01g_flush(flashDevice_t *fdevice)
{
    if (bufferDirty) {
        currentPage = W25N01G_LINEAR_TO_PAGE(programStartAddress); // reset page to the page being written

        w25n01g_programExecute(fdevice, W25N01G_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;
    } else {
        isProgramming = false;
    }
}

void w25n01g_addError(uint32_t address, uint8_t code)
{
    UNUSED(address);
    UNUSED(code);
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to W25N01G_TIMEOUT_PAGE_READ_MS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */

// Continuous read mode (BUF = 0):
// (1) "Page Data Read" command is executed for the page pointed by address
// (2) "Read Data" command is executed for bytes not requested and data are discarded
// (3) "Read Data" command is executed and data are stored directly into caller's buffer
//
// Buffered read mode (BUF = 1), non-read ahead
// (1) If currentBufferPage != requested page, then issue PAGE_DATA_READ on requested page.
// (2) Compute transferLength as smaller of remaining length and requested length.
// (3) Issue READ_DATA on column address.
// (4) Return transferLength.

int w25n01g_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    uint32_t targetPage = W25N01G_LINEAR_TO_PAGE(address);

    if (currentPage != targetPage) {
        if (!w25n01g_waitForReady(fdevice)) {
            return 0;
        }

        currentPage = UINT32_MAX;

        w25n01g_performCommandWithPageAddress(&fdevice->io, W25N01G_INSTRUCTION_PAGE_DATA_READ, targetPage);

        w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_PAGE_READ_MS);
        if (!w25n01g_waitForReady(fdevice)) {
            return 0;
        }

        currentPage = targetPage;
    }

    uint32_t column = W25N01G_LINEAR_TO_COLUMN(address);
    uint16_t transferLength;

    if (length > W25N01G_PAGE_SIZE - column) {
        transferLength = W25N01G_PAGE_SIZE - column;
    } else {
        transferLength = length;
    }

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[4];
        cmd[0] = W25N01G_INSTRUCTION_READ_DATA;
        cmd[1] = (column >> 8) & 0xff;
        cmd[2] = (column >> 0) & 0xff;
        cmd[3] = 0;

        busSegment_t segments[] = {
                {cmd, NULL, sizeof (cmd), false, NULL},
                {NULL, buffer, length, true, NULL},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWaitClaim(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        //quadSpiReceiveWithAddress1LINE(quadSpi, W25N01G_INSTRUCTION_READ_DATA, 8, column, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, buffer, length);
        quadSpiReceiveWithAddress4LINES(quadSpi, W25N01G_INSTRUCTION_FAST_READ_QUAD_OUTPUT, 8, column, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, buffer, length);
    }
#endif

    // XXX Don't need this?
    w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_PAGE_READ_MS);
    if (!w25n01g_waitForReady(fdevice)) {
        return 0;
    }

    // Check ECC

    uint8_t statReg = w25n01g_readRegister(&fdevice->io, W25N01G_STAT_REG);
    uint8_t eccCode = W25N01G_STATUS_FLAG_ECC(statReg);

    switch (eccCode) {
    case 0: // Successful read, no ECC correction
        break;
    case 1: // Successful read with ECC correction
    case 2: // Uncorrectable ECC in a single page
    case 3: // Uncorrectable ECC in multiple pages
        w25n01g_addError(address, eccCode);
        w25n01g_deviceReset(fdevice);
        break;
    }

    return transferLength;
}

int w25n01g_readExtensionBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{

    if (!w25n01g_waitForReady(fdevice)) {
        return 0;
    }

    w25n01g_performCommandWithPageAddress(&fdevice->io, W25N01G_INSTRUCTION_PAGE_DATA_READ, W25N01G_LINEAR_TO_PAGE(address));

    uint32_t column = 2048;

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[4];
        cmd[0] = W25N01G_INSTRUCTION_READ_DATA;
        cmd[1] = (column >> 8) & 0xff;
        cmd[2] = (column >> 0) & 0xff;
        cmd[3] = 0;

        busSegment_t segments[] = {
                {cmd, NULL, sizeof (cmd), false, NULL},
                {NULL, buffer, length, true, NULL},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);

    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        quadSpiReceiveWithAddress1LINE(quadSpi, W25N01G_INSTRUCTION_READ_DATA, 8, column, W28N01G_STATUS_COLUMN_ADDRESS_SIZE, buffer, length);
    }
#endif

    w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_PAGE_READ_MS);

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling w25n01g_init() (the result would have totalSize = 0).
 */
const flashGeometry_t* w25n01g_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t w25n01g_vTable = {
    .isReady = w25n01g_isReady,
    .waitForReady = w25n01g_waitForReady,
    .eraseSector = w25n01g_eraseSector,
    .eraseCompletely = w25n01g_eraseCompletely,
    .pageProgramBegin = w25n01g_pageProgramBegin,
    .pageProgramContinue = w25n01g_pageProgramContinue,
    .pageProgramFinish = w25n01g_pageProgramFinish,
    .pageProgram = w25n01g_pageProgram,
    .flush = w25n01g_flush,
    .readBytes = w25n01g_readBytes,
    .getGeometry = w25n01g_getGeometry,
};

typedef volatile struct cb_context_s {
    flashDevice_t *fdevice;
    bblut_t *bblut;
    int lutsize;
    int lutindex;
} cb_context_t;

// Called in ISR context
// Read of BBLUT entry has just completed
busStatus_e w25n01g_readBBLUTCallback(uint32_t arg)
{
    cb_context_t *cb_context = (cb_context_t *)arg;
    flashDevice_t *fdevice = cb_context->fdevice;
    uint8_t *rxData = fdevice->io.handle.dev->bus->curSegment->rxData;


    cb_context->bblut->pba = (rxData[0] << 16)|rxData[1];
    cb_context->bblut->lba = (rxData[2] << 16)|rxData[3];

    if (++cb_context->lutindex < cb_context->lutsize) {
        cb_context->bblut++;
        return BUS_BUSY; // Repeat the operation
    }

    return BUS_READY; // All done
}


void w25n01g_readBBLUT(flashDevice_t *fdevice, bblut_t *bblut, int lutsize)
{
    cb_context_t cb_context;
    uint8_t in[4];

    cb_context.fdevice = fdevice;
    fdevice->callbackArg = (uint32_t)&cb_context;

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[4];

        cmd[0] = W25N01G_INSTRUCTION_READ_BBM_LUT;
        cmd[1] = 0;

        cb_context.bblut = &bblut[0];
        cb_context.lutsize = lutsize;
        cb_context.lutindex = 0;

        busSegment_t segments[] = {
                {cmd, NULL, sizeof (cmd), false, NULL},
                {NULL, in, sizeof (in), true, w25n01g_readBBLUTCallback},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWaitClaim(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        // Note: Using HAL QuadSPI there doesn't appear to be a way to send 2 bytes, then blocks of 4 bytes, while keeping the CS line LOW
        // thus, we have to read the entire BBLUT in one go and process the result.

        uint8_t bblutBuffer[W25N01G_BBLUT_TABLE_ENTRY_COUNT * W25N01G_BBLUT_TABLE_ENTRY_SIZE];
        quadSpiReceive1LINE(quadSpi, W25N01G_INSTRUCTION_READ_BBM_LUT, 8, bblutBuffer, sizeof(bblutBuffer));

        for (int i = 0, offset = 0 ; i < lutsize ; i++, offset += 4) {
            if (i < W25N01G_BBLUT_TABLE_ENTRY_COUNT) {
                bblut[i].pba = (in[offset + 0] << 16)|in[offset + 1];
                bblut[i].lba = (in[offset + 2] << 16)|in[offset + 3];
            }
        }
    }
#endif
}

void w25n01g_writeBBLUT(flashDevice_t *fdevice, uint16_t lba, uint16_t pba)
{
    w25n01g_waitForReady(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[5] = { W25N01G_INSTRUCTION_BB_MANAGEMENT, lba >> 8, lba, pba >> 8, pba };

        busSegment_t segments[] = {
                {cmd, NULL, sizeof (cmd), true, NULL},
                {NULL, NULL, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        QUADSPI_TypeDef *quadSpi = fdevice->io.handle.quadSpi;

        uint8_t data[4] = { lba >> 8, lba, pba >> 8, pba };
        quadSpiInstructionWithData1LINE(quadSpi, W25N01G_INSTRUCTION_BB_MANAGEMENT, 0, data, sizeof(data));
    }
#endif

    w25n01g_setTimeout(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);
}

static void w25n01g_deviceInit(flashDevice_t *flashdev)
{
    UNUSED(flashdev);
}
#endif
