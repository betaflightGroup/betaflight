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

#include "drivers/io_types.h"

#define BMP085_I2C_ADDR 0x77

typedef struct bmp085Config_s {
    ioTag_t xclrIO;
    ioTag_t eocIO;
} bmp085Config_t;

bool bmp085Detect(const bmp085Config_t *config, baroDev_t *baro);
void bmp085Disable(const bmp085Config_t *config);

#if defined(BARO_EOC_GPIO)
bool bmp085TestEOCConnected(const bmp085Config_t *config);
#endif

#ifdef UNIT_TEST
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
#endif
