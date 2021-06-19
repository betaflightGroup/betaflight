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

#include <stdint.h>
#include <stdbool.h>

void systemInit(void);

typedef enum {
    FAILURE_DEVELOPER = 0,
    FAILURE_MISSING_ACC,
    FAILURE_ACC_INIT,
    FAILURE_ACC_INCOMPATIBLE,
    FAILURE_INVALID_EEPROM_CONTENTS,
    FAILURE_CONFIG_STORE_FAILURE,
    FAILURE_GYRO_INIT_FAILED,
    FAILURE_FLASH_READ_FAILED,
    FAILURE_FLASH_WRITE_FAILED,
    FAILURE_FLASH_INIT_FAILED, // RESERVED
    FAILURE_EXTERNAL_FLASH_READ_FAILED,  // RESERVED
    FAILURE_EXTERNAL_FLASH_WRITE_FAILED, // RESERVED
    FAILURE_EXTERNAL_FLASH_INIT_FAILED,
    FAILURE_SDCARD_READ_FAILED,
    FAILURE_SDCARD_WRITE_FAILED,
    FAILURE_SDCARD_INITIALISATION_FAILED,
    FAILURE_SDCARD_REQUIRED,
} failureMode_e;

#define WARNING_FLASH_DURATION_MS 50
#define WARNING_FLASH_COUNT 5
#define WARNING_PAUSE_DURATION_MS 500
#define WARNING_CODE_DURATION_LONG_MS 250
#define WARNING_CODE_DURATION_SHORT_MS 50

typedef enum {
    BOOTLOADER_REQUEST_ROM,
    BOOTLOADER_REQUEST_FLASH,
} bootloaderRequestType_e;

// failure
void indicateFailure(failureMode_e mode, int repeatCount);
void failureMode(failureMode_e mode);

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(bootloaderRequestType_e requestType);
bool isMPUSoftReset(void);
void cycleCounterInit(void);
uint32_t clockCyclesToMicros(uint32_t clockCycles);
uint32_t getCycleCounter(void);
#if defined(STM32H7) || defined(STM32G4)
void systemCheckResetReason(void);
#endif

void initialiseMemorySections(void);

void enableGPIOPowerUsageAndNoiseReductions(void);
// current crystal frequency - 8 or 12MHz

extern uint32_t hse_value;
extern uint32_t cachedRccCsrValue;

typedef void extiCallbackHandlerFunc(void);

void registerExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);void unregisterExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);

void unusedPinsInit(void);
