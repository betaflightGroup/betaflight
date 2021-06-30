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

#include "common/time.h"

#include "drivers/exti.h"

#define RX_SPI_MAX_PAYLOAD_SIZE 35

struct rxSpiConfig_s;

void rxSpiDevicePreInit(const struct rxSpiConfig_s *rxSpiConfig);
bool rxSpiDeviceInit(const struct rxSpiConfig_s *rxSpiConfig);
bool rxSpiInhibitDma();
void rxSpiRestoreDma(bool useDMA);
uint8_t rxSpiTransferByte(uint8_t data);
void rxSpiWriteByte(uint8_t data);
void rxSpiWriteCommand(uint8_t command, uint8_t data);
void rxSpiWriteCommandMulti(uint8_t command, const uint8_t *data, uint8_t length);
uint8_t rxSpiReadCommand(uint8_t command, uint8_t commandData);
void rxSpiReadCommandMulti(uint8_t command, uint8_t commandData, uint8_t *retData, uint8_t length);
void rxSpiExtiInit(ioConfig_t rxSpiExtiPinConfig, extiTrigger_t rxSpiExtiPinTrigger);
bool rxSpiExtiConfigured(void);
bool rxSpiGetExtiState(void);
bool rxSpiPollExti(void);
void rxSpiResetExti(void);
timeUs_t rxSpiGetLastExtiTimeUs(void);
