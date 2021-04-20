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
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_CHIP

#include "flash.h"
#include "flash_impl.h"
#include "flash_m25p16.h"
#include "flash_w25n01g.h"
#include "flash_w25q128fv.h"
#include "flash_w25m.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/io.h"
#include "drivers/time.h"

// 20 MHz max SPI frequency
#define FLASH_MAX_SPI_CLK_HZ 20000000
// 5 MHz max SPI init frequency
#define FLASH_MAX_SPI_INIT_CLK 5000000

static extDevice_t devInstance;
static extDevice_t *dev;

static flashDevice_t flashDevice;
static flashPartitionTable_t flashPartitionTable;
static int flashPartitions = 0;

#define FLASH_INSTRUCTION_RDID 0x9F

#ifdef USE_QUADSPI
static bool flashQuadSpiInit(const flashConfig_t *flashConfig)
{
    bool detected = false;

    enum { TRY_1LINE = 0, TRY_4LINE, BAIL};
    int phase = TRY_1LINE;

    QUADSPI_TypeDef *hqspi = quadSpiInstanceByDevice(QUADSPI_CFG_TO_DEV(flashConfig->quadSpiDevice));

    do {
        quadSpiSetDivisor(hqspi, QUADSPI_CLOCK_INITIALISATION);

        // 3 bytes for what we need, but some IC's need 8 dummy cycles after the instruction, so read 4 and make two attempts to
        // assemble the chip id from the response.
        uint8_t readIdResponse[4];

        bool status = false;
        switch (phase) {
        case TRY_1LINE:
            status = quadSpiReceive1LINE(hqspi, FLASH_INSTRUCTION_RDID, 0, readIdResponse, 4);
            break;
        case TRY_4LINE:
            status = quadSpiReceive4LINES(hqspi, FLASH_INSTRUCTION_RDID, 2, readIdResponse, 3);
            break;
        default:
            break;
        }

        if (!status) {
            phase++;
            continue;
        }

        flashDevice.io.handle.quadSpi = hqspi;
        flashDevice.io.mode = FLASHIO_QUADSPI;

        quadSpiSetDivisor(hqspi, QUADSPI_CLOCK_ULTRAFAST);


        for (uint8_t offset = 0; offset <= 1 && !detected; offset++) {

            uint32_t chipID = (readIdResponse[offset + 0] << 16) | (readIdResponse[offset + 1] << 8) | (readIdResponse[offset + 2]);

            if (offset == 0) {
#ifdef USE_FLASH_W25Q128FV
                if (!detected && w25q128fv_detect(&flashDevice, chipID)) {
                    detected = true;
                }
#endif
            }

            if (offset == 1) {
#ifdef USE_FLASH_W25N01G
                if (!detected && w25n01g_detect(&flashDevice, chipID)) {
                    detected = true;
                }
#endif
#if defined(USE_FLASH_W25M02G)
                if (!detected && w25m_detect(&flashDevice, chipID)) {
                    detected = true;
                }
#endif
            }
        }
        phase++;
    } while (phase != BAIL && !detected);

    return detected;
}
#endif  // USE_QUADSPI

#ifdef USE_SPI

void flashPreInit(const flashConfig_t *flashConfig)
{
    spiPreinitRegister(flashConfig->csTag, IOCFG_IPU, 1);
}

static bool flashSpiInit(const flashConfig_t *flashConfig)
{
    // Read chip identification and send it to device detect
    dev = &devInstance;

    if (flashConfig->csTag) {
        dev->busType_u.spi.csnPin = IOGetByTag(flashConfig->csTag);
    } else {
        return false;
    }

    if (!IOIsFreeOrPreinit(dev->busType_u.spi.csnPin)) {
        return false;
    }

    if (!spiSetBusInstance(dev, flashConfig->spiDevice, OWNER_FLASH_CS)) {
        return false;
    }

    // Set the callback argument when calling back to this driver for DMA completion
    dev->callbackArg = (uint32_t)&flashDevice;

    IOInit(dev->busType_u.spi.csnPin, OWNER_FLASH_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);

    //Maximum speed for standard READ command is 20mHz, other commands tolerate 25mHz
    spiSetClkDivisor(dev, spiCalculateDivider(FLASH_MAX_SPI_INIT_CLK));

    flashDevice.io.mode = FLASHIO_SPI;
    flashDevice.io.handle.dev = dev;

    delay(50); // short delay required after initialisation of SPI device instance.

    /*
     * Some newer chips require one dummy byte to be read; we can read
     * 4 bytes for these chips while retaining backward compatibility.
     */
    uint8_t readIdResponse[4] = { 0 };

    spiReadRegBuf(dev, FLASH_INSTRUCTION_RDID, readIdResponse, sizeof (readIdResponse));

    // Manufacturer, memory type, and capacity
    uint32_t chipID = (readIdResponse[0] << 16) | (readIdResponse[1] << 8) | (readIdResponse[2]);

#ifdef USE_FLASH_M25P16
    if (m25p16_detect(&flashDevice, chipID)) {
        return true;
    }
#endif

#ifdef USE_FLASH_W25M512
    if (w25m_detect(&flashDevice, chipID)) {
        return true;
    }
#endif

    // Newer chips
    chipID = (readIdResponse[2] << 16) | (readIdResponse[3] << 8) | (readIdResponse[4]);

#ifdef USE_FLASH_W25N01G
    if (w25n01g_detect(&flashDevice, chipID)) {
        return true;
    }
#endif

#ifdef USE_FLASH_W25M02G
    if (w25m_detect(&flashDevice, chipID)) {
        return true;
    }
#endif

    spiPreinitByTag(flashConfig->csTag);

    return false;
}
#endif // USE_SPI

bool flashDeviceInit(const flashConfig_t *flashConfig)
{
#ifdef USE_SPI
    bool useSpi = (SPI_CFG_TO_DEV(flashConfig->spiDevice) != SPIINVALID);

    if (useSpi) {
        return flashSpiInit(flashConfig);
    }
#endif

#ifdef USE_QUADSPI
    bool useQuadSpi = (QUADSPI_CFG_TO_DEV(flashConfig->quadSpiDevice) != QUADSPIINVALID);
    if (useQuadSpi) {
        return flashQuadSpiInit(flashConfig);
    }
#endif

    return false;
}

bool flashIsReady(void)
{
    return flashDevice.vTable->isReady(&flashDevice);
}

bool flashWaitForReady(void)
{
    return flashDevice.vTable->waitForReady(&flashDevice);
}

void flashEraseSector(uint32_t address)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->eraseSector(&flashDevice, address);
}

void flashEraseCompletely(void)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->eraseCompletely(&flashDevice);
}

void flashPageProgramBegin(uint32_t address)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->pageProgramBegin(&flashDevice, address);
}

void flashPageProgramContinue(const uint8_t *data, int length)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->pageProgramContinue(&flashDevice, data, length);
}

void flashPageProgramFinish(void)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->pageProgramFinish(&flashDevice);
}

void flashPageProgram(uint32_t address, const uint8_t *data, int length)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->pageProgram(&flashDevice, address, data, length);
}

int flashReadBytes(uint32_t address, uint8_t *buffer, int length)
{
    flashDevice.callback = NULL;
    return flashDevice.vTable->readBytes(&flashDevice, address, buffer, length);
}

void flashFlush(void)
{
    if (flashDevice.vTable->flush) {
        flashDevice.vTable->flush(&flashDevice);
    }
}

static const flashGeometry_t noFlashGeometry = {
    .totalSize = 0,
};

const flashGeometry_t *flashGetGeometry(void)
{
    if (flashDevice.vTable && flashDevice.vTable->getGeometry) {
        return flashDevice.vTable->getGeometry(&flashDevice);
    }

    return &noFlashGeometry;
}

/*
 * Flash partitioning
 *
 * Partition table is not currently stored on the flash, in-memory only.
 *
 * Partitions are required so that Badblock management (inc spare blocks), FlashFS (Blackbox Logging), Configuration and Firmware can be kept separate and tracked.
 *
 * XXX FIXME
 * XXX Note that Flash FS must start at sector 0.
 * XXX There is existing blackbox/flash FS code the relies on this!!!
 * XXX This restriction can and will be fixed by creating a set of flash operation functions that take partition as an additional parameter.
 */

static void flashConfigurePartitions(void)
{

    const flashGeometry_t *flashGeometry = flashGetGeometry();
    if (flashGeometry->totalSize == 0) {
        return;
    }

    flashSector_t startSector = 0;
    flashSector_t endSector = flashGeometry->sectors - 1; // 0 based index

    const flashPartition_t *badBlockPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_BADBLOCK_MANAGEMENT);
    if (badBlockPartition) {
        endSector = badBlockPartition->startSector - 1;
    }

#if defined(FIRMWARE_SIZE)
    const uint32_t firmwareSize = (FIRMWARE_SIZE * 1024);
    flashSector_t firmwareSectors = (firmwareSize / flashGeometry->sectorSize);

    if (firmwareSize % flashGeometry->sectorSize > 0) {
        firmwareSectors++; // needs a portion of a sector.
    }

    startSector = (endSector + 1) - firmwareSectors; // + 1 for inclusive

    flashPartitionSet(FLASH_PARTITION_TYPE_FIRMWARE, startSector, endSector);

    endSector = startSector - 1;
    startSector = 0;
#endif

#if defined(CONFIG_IN_EXTERNAL_FLASH)
    const uint32_t configSize = EEPROM_SIZE;
    flashSector_t configSectors = (configSize / flashGeometry->sectorSize);

    if (configSize % flashGeometry->sectorSize > 0) {
        configSectors++; // needs a portion of a sector.
    }

    startSector = (endSector + 1) - configSectors; // + 1 for inclusive

    flashPartitionSet(FLASH_PARTITION_TYPE_CONFIG, startSector, endSector);

    endSector = startSector - 1;
    startSector = 0;
#endif

#ifdef USE_FLASHFS
    flashPartitionSet(FLASH_PARTITION_TYPE_FLASHFS, startSector, endSector);
#endif
}

flashPartition_t *flashPartitionFindByType(uint8_t type)
{
    for (int index = 0; index < FLASH_MAX_PARTITIONS; index++) {
        flashPartition_t *candidate = &flashPartitionTable.partitions[index];
        if (candidate->type == type) {
            return candidate;
        }
    }

    return NULL;
}

const flashPartition_t *flashPartitionFindByIndex(uint8_t index)
{
    if (index >= flashPartitions) {
        return NULL;
    }

    return &flashPartitionTable.partitions[index];
}

void flashPartitionSet(uint8_t type, uint32_t startSector, uint32_t endSector)
{
    flashPartition_t *entry = flashPartitionFindByType(type);

    if (!entry) {
        if (flashPartitions == FLASH_MAX_PARTITIONS - 1) {
            return;
        }
        entry = &flashPartitionTable.partitions[flashPartitions++];
    }

    entry->type = type;
    entry->startSector = startSector;
    entry->endSector = endSector;
}

// Must be in sync with FLASH_PARTITION_TYPE
static const char *flashPartitionNames[] = {
    "UNKNOWN  ",
    "PARTITION",
    "FLASHFS  ",
    "BBMGMT   ",
    "FIRMWARE ",
    "CONFIG   ",
};

const char *flashPartitionGetTypeName(flashPartitionType_e type)
{
    if (type < ARRAYLEN(flashPartitionNames)) {
        return flashPartitionNames[type];
    }

    return NULL;
}

bool flashInit(const flashConfig_t *flashConfig)
{
    memset(&flashPartitionTable, 0x00, sizeof(flashPartitionTable));
    flashPartitions = 0;

    bool haveFlash = flashDeviceInit(flashConfig);

    flashConfigurePartitions();

    return haveFlash;
}

int flashPartitionCount(void)
{
    return flashPartitions;
}
#endif // USE_FLASH_CHIP
