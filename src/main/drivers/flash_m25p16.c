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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_M25P16

#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/flash.h"

#include "flash_m25p16.h"

#define M25P16_INSTRUCTION_RDID             SPIFLASH_INSTRUCTION_RDID
#define M25P16_INSTRUCTION_READ_BYTES       0x03
#define M25P16_INSTRUCTION_READ_STATUS_REG  0x05
#define M25P16_INSTRUCTION_WRITE_STATUS_REG 0x01
#define M25P16_INSTRUCTION_WRITE_ENABLE     0x06
#define M25P16_INSTRUCTION_WRITE_DISABLE    0x04
#define M25P16_INSTRUCTION_PAGE_PROGRAM     0x02
#define M25P16_INSTRUCTION_SECTOR_ERASE     0xD8
#define M25P16_INSTRUCTION_BULK_ERASE       0xC7

#define M25P16_STATUS_FLAG_WRITE_IN_PROGRESS 0x01
#define M25P16_STATUS_FLAG_WRITE_ENABLED     0x02

#define W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE 0xB7

static uint32_t maxClkSPIHz;
static uint32_t maxReadClkSPIHz;

// Table of recognised FLASH devices
struct {
    uint32_t        jedecID;
    uint16_t        maxClkSPIMHz;
    uint16_t        maxReadClkSPIMHz;
    flashSector_t   sectors;
    uint16_t        pagesPerSector;
} m25p16FlashConfig[] = {
    // Macronix MX25L3206E
    // Datasheet: https://docs.rs-online.com/5c85/0900766b814ac6f9.pdf
    { 0xC22016, 86, 33, 64, 256 },
    // Macronix MX25L6406E
    // Datasheet: https://www.macronix.com/Lists/Datasheet/Attachments/7370/MX25L6406E,%203V,%2064Mb,%20v1.9.pdf
    { 0xC22017, 86, 33, 128, 256 },
    // Macronix MX25L25635E
    // Datasheet: https://www.macronix.com/Lists/Datasheet/Attachments/7331/MX25L25635E,%203V,%20256Mb,%20v1.3.pdf
    { 0xC22019, 80, 50, 512, 256 },
    // Micron M25P16
    // Datasheet: https://www.micron.com/-/media/client/global/documents/products/data-sheet/nor-flash/serial-nor/m25p/m25p16.pdf
    { 0x202015, 25, 20, 32, 256 },
    // Micron N25Q064
    // Datasheet: https://www.micron.com/-/media/client/global/documents/products/data-sheet/nor-flash/serial-nor/n25q/n25q_64a_3v_65nm.pdf
    { 0x20BA17, 108, 54, 128, 256 },
    // Micron N25Q128
    // Datasheet: https://www.micron.com/-/media/client/global/documents/products/data-sheet/nor-flash/serial-nor/n25q/n25q_128mb_1_8v_65nm.pdf
    { 0x20ba18, 108, 54, 256, 256 },
    // Winbond W25Q16
    // Datasheet: https://www.winbond.com/resource-files/w25q16dv_revi_nov1714_web.pdf
    { 0xEF4015, 104, 50, 32, 256 },
    // Winbond W25Q32
    // Datasheet: https://www.winbond.com/resource-files/w25q32jv%20dtr%20revf%2002242017.pdf?__locale=zh_TW
    { 0xEF4016, 133, 50, 64, 256 },
    // Winbond W25Q64
    // Datasheet: https://www.winbond.com/resource-files/w25q64jv%20spi%20%20%20revc%2006032016%20kms.pdf
    { 0xEF4017, 133, 50, 128, 256 }, // W25Q64JV-IQ/JQ 
    { 0xEF7017, 133, 50, 128, 256 }, // W25Q64JV-IM/JM*
    // Winbond W25Q128
    // Datasheet: https://www.winbond.com/resource-files/w25q128fv%20rev.l%2008242015.pdf
    { 0xEF4018, 104, 50, 256, 256 },
    // Zbit ZB25VQ128
    // Datasheet: http://zbitsemi.com/upload/file/20201010/20201010174048_82182.pdf
    { 0x5E4018, 104, 50, 256, 256 },
    // Winbond W25Q128_DTR
    // Datasheet: https://www.winbond.com/resource-files/w25q128jv%20dtr%20revb%2011042016.pdf
    { 0xEF7018, 66, 50, 256, 256 },
    // Winbond W25Q256
    // Datasheet: https://www.winbond.com/resource-files/w25q256jv%20spi%20revb%2009202016.pdf
    { 0xEF4019, 133, 50, 512, 256 },
    // Cypress S25FL064L
    // Datasheet: https://www.cypress.com/file/316661/download
    { 0x016017, 133, 50, 128, 256 },
    // Cypress S25FL128L
    // Datasheet: https://www.cypress.com/file/316171/download
    { 0x016018, 133, 50, 256, 256 },
    // BergMicro W25Q32
    // Datasheet: https://www.winbond.com/resource-files/w25q32jv%20dtr%20revf%2002242017.pdf?__locale=zh_TW
    { 0xE04016, 133, 50, 1024, 16 },
    // End of list
    { 0x000000, 0, 0, 0, 0 }
};

// IMPORTANT: Timeout values are currently required to be set to the highest value required by any of the supported flash chips by this driver.

// The timeout we expect between being able to issue page program instructions
#define DEFAULT_TIMEOUT_MILLIS       6
#define SECTOR_ERASE_TIMEOUT_MILLIS  5000

// etracer65 notes: For bulk erase The 25Q16 takes about 3 seconds and the 25Q128 takes about 49
#define BULK_ERASE_TIMEOUT_MILLIS    50000

#define M25P16_PAGESIZE 256

STATIC_ASSERT(M25P16_PAGESIZE < FLASH_MAX_PAGE_SIZE, M25P16_PAGESIZE_too_small);

const flashVTable_t m25p16_vTable;

#ifndef USE_SPI_TRANSACTION
static void m25p16_disable(busDevice_t *bus)
{
    IOHi(bus->busdev_u.spi.csnPin);
    __NOP();
}

static void m25p16_enable(busDevice_t *bus)
{
    __NOP();
    IOLo(bus->busdev_u.spi.csnPin);
}
#endif

static void m25p16_transfer(busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int len)
{
#ifdef USE_SPI_TRANSACTION
    spiBusTransactionTransfer(bus, txData, rxData, len);
#else
    m25p16_enable(bus);
    spiTransfer(bus->busdev_u.spi.instance, txData, rxData, len);
    m25p16_disable(bus);
#endif
}

/**
 * Send the given command byte to the device.
 */
static void m25p16_performOneByteCommand(busDevice_t *bus, uint8_t command)
{
#ifdef USE_SPI_TRANSACTION
    m25p16_transfer(bus, &command, NULL, 1);
#else
    m25p16_enable(bus);
    spiTransferByte(bus->busdev_u.spi.instance, command);
    m25p16_disable(bus);
#endif
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
static void m25p16_writeEnable(flashDevice_t *fdevice)
{
    m25p16_performOneByteCommand(fdevice->io.handle.busdev, M25P16_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    fdevice->couldBeBusy = true;
}

static uint8_t m25p16_readStatus(busDevice_t *bus)
{
    const uint8_t command[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    uint8_t in[2];

    m25p16_transfer(bus, command, in, sizeof(command));

    return in[1];
}

static bool m25p16_isReady(flashDevice_t *fdevice)
{
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    fdevice->couldBeBusy = fdevice->couldBeBusy && ((m25p16_readStatus(fdevice->io.handle.busdev) & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) != 0);

    return !fdevice->couldBeBusy;
}

static void m25p16_setTimeout(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t now = millis();
    fdevice->timeoutAt = now + timeoutMillis;
}

static bool m25p16_waitForReady(flashDevice_t *fdevice)
{
    while (!m25p16_isReady(fdevice)) {
        uint32_t now = millis();
        if (cmp32(now, fdevice->timeoutAt) >= 0) {
            return false;
        }
    }

    fdevice->timeoutAt = 0;
    return true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */

bool m25p16_detect(flashDevice_t *fdevice, uint32_t chipID)
{
    flashGeometry_t *geometry = &fdevice->geometry;
    uint8_t index;

    for (index = 0; m25p16FlashConfig[index].jedecID; index++) {
        if (m25p16FlashConfig[index].jedecID == chipID) {
            maxClkSPIHz = m25p16FlashConfig[index].maxClkSPIMHz * 1000000;
            maxReadClkSPIHz = m25p16FlashConfig[index].maxReadClkSPIMHz * 1000000;
            geometry->sectors = m25p16FlashConfig[index].sectors;
            geometry->pagesPerSector = m25p16FlashConfig[index].pagesPerSector;
            break;
        }
    }

    if (m25p16FlashConfig[index].jedecID == 0) {
        // Unsupported chip or not an SPI NOR flash
        geometry->sectors = 0;
        geometry->pagesPerSector = 0;
        geometry->sectorSize = 0;
        geometry->totalSize = 0;
        return false;
    }

    geometry->flashType = FLASH_TYPE_NOR;
    geometry->pageSize = M25P16_PAGESIZE;
    geometry->sectorSize = geometry->pagesPerSector * geometry->pageSize;
    geometry->totalSize = geometry->sectorSize * geometry->sectors;

    // Adjust the SPI bus clock frequency
#ifndef FLASH_SPI_SHARED
    spiSetDivisor(fdevice->io.handle.busdev->busdev_u.spi.instance, spiCalculateDivider(maxReadClkSPIHz));
#endif

    if (fdevice->geometry.totalSize > 16 * 1024 * 1024) {
        fdevice->isLargeFlash = true;
        m25p16_performOneByteCommand(fdevice->io.handle.busdev, W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE);
    }

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
    fdevice->vTable = &m25p16_vTable;
    return true;
}

static void m25p16_setCommandAddress(uint8_t *buf, uint32_t address, bool useLongAddress)
{
    if (useLongAddress) {
        *buf++ = (address >> 24) & 0xff;
    }
    *buf++ = (address >> 16) & 0xff;
    *buf++ = (address >> 8) & 0xff;
    *buf = address & 0xff;
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
static void m25p16_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    uint8_t out[5] = { M25P16_INSTRUCTION_SECTOR_ERASE };

    m25p16_setCommandAddress(&out[1], address, fdevice->isLargeFlash);

    m25p16_waitForReady(fdevice);

    m25p16_writeEnable(fdevice);

    m25p16_transfer(fdevice->io.handle.busdev, out, NULL, fdevice->isLargeFlash ? 5 : 4);

    m25p16_setTimeout(fdevice, SECTOR_ERASE_TIMEOUT_MILLIS);
}

static void m25p16_eraseCompletely(flashDevice_t *fdevice)
{
    m25p16_waitForReady(fdevice);

    m25p16_writeEnable(fdevice);

    m25p16_performOneByteCommand(fdevice->io.handle.busdev, M25P16_INSTRUCTION_BULK_ERASE);

    m25p16_setTimeout(fdevice, BULK_ERASE_TIMEOUT_MILLIS);
}

static void m25p16_pageProgramBegin(flashDevice_t *fdevice, uint32_t address)
{
    UNUSED(fdevice);

    fdevice->currentWriteAddress = address;
}

static void m25p16_pageProgramContinue(flashDevice_t *fdevice, const uint8_t *data, int length)
{
    uint8_t command[5] = { M25P16_INSTRUCTION_PAGE_PROGRAM };

    m25p16_setCommandAddress(&command[1], fdevice->currentWriteAddress, fdevice->isLargeFlash);

    m25p16_waitForReady(fdevice);

    m25p16_writeEnable(fdevice);

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionBegin(fdevice->io.handle.busdev);
#else
    m25p16_enable(fdevice->io.handle.busdev);
#endif

    spiTransfer(fdevice->io.handle.busdev->busdev_u.spi.instance, command, NULL, fdevice->isLargeFlash ? 5 : 4);
    spiTransfer(fdevice->io.handle.busdev->busdev_u.spi.instance, data, NULL, length);

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionEnd(fdevice->io.handle.busdev);
#else
    m25p16_disable(fdevice->io.handle.busdev);
#endif

    fdevice->currentWriteAddress += length;

    m25p16_setTimeout(fdevice, DEFAULT_TIMEOUT_MILLIS);
}

static void m25p16_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
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
static void m25p16_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, int length)
{
    m25p16_pageProgramBegin(fdevice, address);

    m25p16_pageProgramContinue(fdevice, data, length);

    m25p16_pageProgramFinish(fdevice);
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
static int m25p16_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{
    uint8_t command[5] = { M25P16_INSTRUCTION_READ_BYTES };

    m25p16_setCommandAddress(&command[1], address, fdevice->isLargeFlash);

    if (!m25p16_waitForReady(fdevice)) {
        return 0;
    }

#ifndef FLASH_SPI_SHARED
    spiSetDivisor(fdevice->io.handle.busdev->busdev_u.spi.instance, spiCalculateDivider(maxReadClkSPIHz));
#endif

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionBegin(fdevice->io.handle.busdev);
#else
    m25p16_enable(fdevice->io.handle.busdev);
#endif

    spiTransfer(fdevice->io.handle.busdev->busdev_u.spi.instance, command, NULL, fdevice->isLargeFlash ? 5 : 4);
    spiTransfer(fdevice->io.handle.busdev->busdev_u.spi.instance, NULL, buffer, length);

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionEnd(fdevice->io.handle.busdev);
#else
    m25p16_disable(fdevice->io.handle.busdev);
#endif

#ifndef FLASH_SPI_SHARED
    spiSetDivisor(fdevice->io.handle.busdev->busdev_u.spi.instance, spiCalculateDivider(maxClkSPIHz));
#endif

    m25p16_setTimeout(fdevice, DEFAULT_TIMEOUT_MILLIS);

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling m25p16_init() (the result would have totalSize = 0).
 */
static const flashGeometry_t* m25p16_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t m25p16_vTable = {
    .isReady = m25p16_isReady,
    .waitForReady = m25p16_waitForReady,
    .eraseSector = m25p16_eraseSector,
    .eraseCompletely = m25p16_eraseCompletely,
    .pageProgramBegin = m25p16_pageProgramBegin,
    .pageProgramContinue = m25p16_pageProgramContinue,
    .pageProgramFinish = m25p16_pageProgramFinish,
    .pageProgram = m25p16_pageProgram,
    .readBytes = m25p16_readBytes,
    .getGeometry = m25p16_getGeometry,
};
#endif
