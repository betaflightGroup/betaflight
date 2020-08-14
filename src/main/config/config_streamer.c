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

#include <string.h>

#include "platform.h"

#include "drivers/system.h"
#include "drivers/flash.h"

#include "config/config_streamer.h"

#if !defined(CONFIG_IN_FLASH)
#if defined(CONFIG_IN_RAM) && defined(PERSISTENT)
PERSISTENT uint8_t eepromData[EEPROM_SIZE];
#else
uint8_t eepromData[EEPROM_SIZE];
#endif
#endif


#if defined(STM32H750xx) && !(defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD))
#error "STM32750xx only has one flash page which contains the bootloader, no spare flash pages available, use external storage for persistent config or ram for target testing"
#endif
// @todo this is not strictly correct for F4/F7, where sector sizes are variable
#if !defined(FLASH_PAGE_SIZE)
// F1
# if defined(STM32F10X_MD)
#  define FLASH_PAGE_SIZE                 (0x400)
# elif defined(STM32F10X_HD)
#  define FLASH_PAGE_SIZE                 (0x800)
// F3
# elif defined(STM32F303xC)
#  define FLASH_PAGE_SIZE                 (0x800)
// F4
# elif defined(STM32F40_41xxx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors
# elif defined (STM32F411xE)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
# elif defined(STM32F427_437xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
# elif defined (STM32F446xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
// F7
#elif defined(STM32F722xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors
# elif defined(STM32F745xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000) // 32K sectors
# elif defined(STM32F746xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000)
# elif defined(STM32F765xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000)
# elif defined(UNIT_TEST)
#  define FLASH_PAGE_SIZE                 (0x400)
// H7
# elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x20000) // 128K sectors
# elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x2000) // 8K sectors
// G4
# elif defined(STM32G4)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x800) // 2K page
// SIMULATOR
# elif defined(SIMULATOR_BUILD)
#  define FLASH_PAGE_SIZE                 (0x400)
# else
#  error "Flash page size not defined for target."
# endif
#endif

void config_streamer_init(config_streamer_t *c)
{
    memset(c, 0, sizeof(*c));
}

void config_streamer_start(config_streamer_t *c, uintptr_t base, int size)
{
    // base must start at FLASH_PAGE_SIZE boundary when using embedded flash.
    c->address = base;
    c->size = size;
    if (!c->unlocked) {
#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_SDCARD)
        // NOP
#elif defined(CONFIG_IN_FLASH) || defined(CONFIG_IN_FILE)
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        HAL_FLASH_Unlock();
#else
        FLASH_Unlock();
#endif
#endif
        c->unlocked = true;
    }

#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_FILE) || defined(CONFIG_IN_EXTERNAL_FLASH)
    // NOP
#elif defined(CONFIG_IN_FLASH)
#if defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#elif defined(STM32F303)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F4)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32F7)
    // NOP
#elif defined(STM32H7)
    // NOP
#elif defined(STM32G4)
    // NOP
#elif defined(UNIT_TEST) || defined(SIMULATOR_BUILD)
    // NOP
#else
# error "Unsupported CPU"
#endif
#endif
    c->err = 0;
}

#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_SDCARD)
// No flash sector method required.
#elif defined(CONFIG_IN_FLASH)
#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
/*
Sector 0    0x08000000 - 0x08007FFF 32 Kbytes
Sector 1    0x08008000 - 0x0800FFFF 32 Kbytes
Sector 2    0x08010000 - 0x08017FFF 32 Kbytes
Sector 3    0x08018000 - 0x0801FFFF 32 Kbytes
Sector 4    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 5    0x08040000 - 0x0807FFFF 256 Kbytes
Sector 6    0x08080000 - 0x080BFFFF 256 Kbytes
Sector 7    0x080C0000 - 0x080FFFFF 256 Kbytes

F7X5XI device with 2M flash
Sector 8    0x08100000 - 0x0813FFFF 256 Kbytes
Sector 9    0x08140000 - 0x0817FFFF 256 Kbytes
Sector 10   0x08180000 - 0x081BFFFF 256 Kbytes
Sector 11   0x081C0000 - 0x081FFFFF 256 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&__config_start <= 0x08017FFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&__config_start <= 0x080BFFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&__config_start <= 0x080FFFFF)
        return FLASH_SECTOR_7;
#if defined(STM32F765xx)
    if ((uint32_t)&__config_start <= 0x0813FFFF)
        return FLASH_SECTOR_8;
    if ((uint32_t)&__config_start <= 0x0817FFFF)
        return FLASH_SECTOR_9;
    if ((uint32_t)&__config_start <= 0x081BFFFF)
        return FLASH_SECTOR_10;
    if ((uint32_t)&__config_start <= 0x081FFFFF)
        return FLASH_SECTOR_11;
#endif

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32F722xx)
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08003FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&__config_start <= 0x0800BFFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&__config_start <= 0x0805FFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_SECTOR_7;

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32F4)
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08003FFF)
        return FLASH_Sector_0;
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_Sector_1;
    if ((uint32_t)&__config_start <= 0x0800BFFF)
        return FLASH_Sector_2;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_Sector_3;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_Sector_4;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_Sector_5;
    if ((uint32_t)&__config_start <= 0x0805FFFF)
        return FLASH_Sector_6;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_Sector_7;
    if ((uint32_t)&__config_start <= 0x0809FFFF)
        return FLASH_Sector_8;
    if ((uint32_t)&__config_start <= 0x080DFFFF)
        return FLASH_Sector_9;
    if ((uint32_t)&__config_start <= 0x080BFFFF)
        return FLASH_Sector_10;
    if ((uint32_t)&__config_start <= 0x080FFFFF)
        return FLASH_Sector_11;

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32H743xx) || defined(STM32G4) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ) || defined(STM32H723xx) || defined(STM32H725xx)
/*
MCUs with uniform array of equal size sectors, handled in two banks having contiguous address.
(Devices with non-contiguous flash layout is not currently useful anyways.)

H743
2 bank * 8 sector/bank * 128K/sector (2MB)
Bank 1 0x08000000 - 0x080FFFFF 128KB * 8
Bank 2 0x08100000 - 0x081FFFFF 128KB * 8

H743
1 bank * 8 sector/bank * 128K/sector (1MB)
Bank 1 0x08000000 - 0x080FFFFF 128KB * 8

H7A3
2 bank * 128 sector/bank * 8KB/sector (2MB)
Bank 1 0x08000000 - 0x080FFFFF 8KB * 128
Bank 2 0x08100000 - 0x081FFFFF 8KB * 128

G473/474 in dual bank mode
2 bank * 128 sector/bank * 2KB/sector (512KB)
Bank 1 0x08000000 - 0x0803FFFF 2KB * 128
Bank 2 0x08040000 - 0x0807FFFF 2KB * 128

Note that FLASH_BANK_SIZE constant used in the following code changes depending on
bank operation mode. The code assumes dual bank operation, in which case the
FLASH_BANK_SIZE constant is set to one half of the available flash size in HAL.
*/

#if defined(STM32H743xx) || defined(STM32H723xx) || defined(STM32H725xx)
#define FLASH_PAGE_PER_BANK 8
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define FLASH_PAGE_PER_BANK 128
#elif defined(STM32G4)
#define FLASH_PAGE_PER_BANK 128
// These are not defined in CMSIS like H7
#define FLASH_BANK1_BASE FLASH_BASE
#define FLASH_BANK2_BASE (FLASH_BANK1_BASE + FLASH_BANK_SIZE)
#endif

static void getFLASHSectorForEEPROM(uint32_t address, uint32_t *bank, uint32_t *sector)
{
#if defined(FLASH_BANK2_BASE)
    if (address >= FLASH_BANK1_BASE && address < FLASH_BANK2_BASE) {
        *bank = FLASH_BANK_1;
    } else if (address >= FLASH_BANK2_BASE && address < FLASH_BANK2_BASE + FLASH_BANK_SIZE) {
        *bank = FLASH_BANK_2;
        address -= FLASH_BANK_SIZE;
    }
#else
    if (address >= FLASH_BANK1_BASE && address < FLASH_BANK1_BASE + FLASH_BANK_SIZE) {
        *bank = FLASH_BANK_1;
    }
#endif
    else {
        // Not good
        while (1) {
            failureMode(FAILURE_CONFIG_STORE_FAILURE);
        }
    }

    address -= FLASH_BANK1_BASE;
    *sector = address / FLASH_PAGE_SIZE;
}
#elif defined(STM32H750xx)
/*
The memory map supports 2 banks of 8 128k sectors like the H743xx, but there is only one 128K sector so we save some code
space by using a smaller function.

Bank 1
Sector 0    0x08000000 - 0x0801FFFF 128 Kbytes

*/

static void getFLASHSectorForEEPROM(uint32_t *bank, uint32_t *sector)
{

    uint32_t start = (uint32_t)&__config_start;

    if (start == FLASH_BANK1_BASE) {
        *sector = FLASH_SECTOR_0;
        *bank = FLASH_BANK_1;
    } else {
        // Not good
        while (1) {
            failureMode(FAILURE_CONFIG_STORE_FAILURE);
        }
    }
}
#endif
#endif // CONFIG_IN_FLASH

// FIXME the return values are currently magic numbers
static int write_word(config_streamer_t *c, config_streamer_buffer_align_type_t *buffer)
{
    if (c->err != 0) {
        return c->err;
    }
#if defined(CONFIG_IN_EXTERNAL_FLASH)

    uint32_t dataOffset = (uint32_t)(c->address - (uintptr_t)&eepromData[0]);

    const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_CONFIG);
    const flashGeometry_t *flashGeometry = flashGetGeometry();

    uint32_t flashStartAddress = flashPartition->startSector * flashGeometry->sectorSize;
    uint32_t flashOverflowAddress = ((flashPartition->endSector + 1) * flashGeometry->sectorSize); // +1 to sector for inclusive

    uint32_t flashAddress = flashStartAddress + dataOffset;
    if (flashAddress + CONFIG_STREAMER_BUFFER_SIZE > flashOverflowAddress) {
        return -3; // address is past end of partition
    }

    uint32_t flashSectorSize = flashGeometry->sectorSize;
    uint32_t flashPageSize = flashGeometry->pageSize;
    const uint8_t *buffers[1];
    uint32_t bufferSizes[1];

    bool onPageBoundary = (flashAddress % flashPageSize == 0);
    if (onPageBoundary) {

        bool firstPage = (flashAddress == flashStartAddress);
        if (!firstPage) {
            flashPageProgramFinish();
        }

        if (flashAddress % flashSectorSize == 0) {
            flashEraseSector(flashAddress);
        }

        flashPageProgramBegin(flashAddress, NULL);
    }

    buffers[0] = (uint8_t *)buffer;
    bufferSizes[0] = CONFIG_STREAMER_BUFFER_SIZE;

    flashPageProgramContinue(buffers, bufferSizes, 1);

#elif defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD)
    if (c->address == (uintptr_t)&eepromData[0]) {
        memset(eepromData, 0, sizeof(eepromData));
    }

    uint64_t *dest_addr = (uint64_t *)c->address;
    uint64_t *src_addr = (uint64_t*)buffer;
    uint8_t row_index = 4;
    /* copy the 256 bits flash word */
    do
    {
      *dest_addr++ = *src_addr++;
    } while (--row_index != 0);

#elif defined(CONFIG_IN_FILE)

    if (c->address % FLASH_PAGE_SIZE == 0) {
        const FLASH_Status status = FLASH_ErasePage(c->address);
        if (status != FLASH_COMPLETE) {
            return -1;
        }
    }
    const FLASH_Status status = FLASH_ProgramWord(c->address, *buffer);
    if (status != FLASH_COMPLETE) {
        return -2;
    }

#elif defined(CONFIG_IN_FLASH)

#if defined(STM32H7)
    if (c->address % FLASH_PAGE_SIZE == 0) {
        FLASH_EraseInitTypeDef EraseInitStruct = {
            .TypeErase     = FLASH_TYPEERASE_SECTORS,
#if !(defined(STM32H7A3xx) || defined(STM32H7A3xxQ))
            .VoltageRange  = FLASH_VOLTAGE_RANGE_3, // 2.7-3.6V
#endif
            .NbSectors     = 1
        };
        getFLASHSectorForEEPROM(c->address, &EraseInitStruct.Banks, &EraseInitStruct.Sector);
        uint32_t SECTORError;
        const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
        if (status != HAL_OK) {
            return -1;
        }
    }

    // For H7
    // HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t DataAddress);
    const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, c->address, (uint64_t)(uint32_t)buffer);
    if (status != HAL_OK) {
        return -2;
    }
#elif defined(STM32F7)
    if (c->address % FLASH_PAGE_SIZE == 0) {
        FLASH_EraseInitTypeDef EraseInitStruct = {
            .TypeErase     = FLASH_TYPEERASE_SECTORS,
            .VoltageRange  = FLASH_VOLTAGE_RANGE_3, // 2.7-3.6V
            .NbSectors     = 1
        };
        EraseInitStruct.Sector = getFLASHSectorForEEPROM();
        uint32_t SECTORError;
        const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
        if (status != HAL_OK) {
            return -1;
        }
    }

    // For F7
    // HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
    const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, c->address, (uint64_t)*buffer);
    if (status != HAL_OK) {
        return -2;
    }
#elif defined(STM32G4)
    if (c->address % FLASH_PAGE_SIZE == 0) {

        FLASH_EraseInitTypeDef EraseInitStruct = {
            .TypeErase     = FLASH_TYPEERASE_PAGES,
            .NbPages       = 1
        };
        getFLASHSectorForEEPROM(c->address, &EraseInitStruct.Banks, &EraseInitStruct.Page);
        uint32_t SECTORError;
        const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
        if (status != HAL_OK) {
            return -1;
        }
    }

    const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, c->address, (uint64_t)*buffer);
    if (status != HAL_OK) {
        return -2;
    }
#else // !STM32H7 && !STM32F7 && !STM32G4
    if (c->address % FLASH_PAGE_SIZE == 0) {
#if defined(STM32F4)
        const FLASH_Status status = FLASH_EraseSector(getFLASHSectorForEEPROM(), VoltageRange_3); //0x08080000 to 0x080A0000
#else // STM32F3, STM32F1
        const FLASH_Status status = FLASH_ErasePage(c->address);
#endif
        if (status != FLASH_COMPLETE) {
            return -1;
        }
    }
    const FLASH_Status status = FLASH_ProgramWord(c->address, *buffer);
    if (status != FLASH_COMPLETE) {
        return -2;
    }
#endif
#endif
    c->address += CONFIG_STREAMER_BUFFER_SIZE;
    return 0;
}

int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size)
{
    for (const uint8_t *pat = p; pat != (uint8_t*)p + size; pat++) {
        c->buffer.b[c->at++] = *pat;

        if (c->at == sizeof(c->buffer)) {
            c->err = write_word(c, &c->buffer.w);
            c->at = 0;
        }
    }
    return c->err;
}

int config_streamer_status(config_streamer_t *c)
{
    return c->err;
}

int config_streamer_flush(config_streamer_t *c)
{
    if (c->at != 0) {
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
        c->err = write_word(c, &c->buffer.w);
        c->at = 0;
    }
    return c-> err;
}

int config_streamer_finish(config_streamer_t *c)
{
    if (c->unlocked) {
#if defined(CONFIG_IN_SDCARD)
        bool saveEEPROMToSDCard(void); // XXX forward declaration to avoid circular dependency between config_streamer / config_eeprom
        saveEEPROMToSDCard();
        // TODO overwrite the data in the file on the SD card.
#elif defined(CONFIG_IN_EXTERNAL_FLASH)
        flashFlush();
#elif defined(CONFIG_IN_RAM)
        // NOP
#elif defined(CONFIG_IN_FILE)
        FLASH_Lock();
#elif defined(CONFIG_IN_FLASH)
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        HAL_FLASH_Lock();
#else
        FLASH_Lock();
#endif
#endif
        c->unlocked = false;
    }
    return c->err;
}
