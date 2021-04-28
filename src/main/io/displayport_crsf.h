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

#include "drivers/display.h"

#define CRSF_DISPLAY_PORT_ROWS_MAX          9
#define CRSF_DISPLAY_PORT_COLS_MAX          32
#define CRSF_DISPLAY_PORT_MAX_BUFFER_SIZE   (CRSF_DISPLAY_PORT_ROWS_MAX * CRSF_DISPLAY_PORT_COLS_MAX)

typedef struct crsfDisplayPortScreen_s {
    char buffer[CRSF_DISPLAY_PORT_MAX_BUFFER_SIZE];
    bool updated;
    uint8_t rows;
    uint8_t cols;
    bool reset;
} crsfDisplayPortScreen_t;

void crsfDisplayportRegister(void);
crsfDisplayPortScreen_t *crsfDisplayPortScreen(void);
void crsfDisplayPortMenuOpen(void);
void crsfDisplayPortMenuExit(void);
void crsfDisplayPortRefresh(void);
bool crsfDisplayPortIsReady(void);
void crsfDisplayPortSetDimensions(uint8_t rows, uint8_t cols);
