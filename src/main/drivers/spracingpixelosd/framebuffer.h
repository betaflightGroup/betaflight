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
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working Pixel OSD system.
 */

#pragma once

//
// Frame Buffer
//

#include "configuration.h"

void frameBuffer_eraseInit(void);
void frameBuffer_erase(uint8_t *frameBuffer);

uint8_t *frameBuffer_getBuffer(uint8_t index);
uint8_t frameBuffer_getBufferIndex(uint8_t *frameBuffer);

// Use FRAME_PIXEL_* for 'mode` arguments below.

void frameBuffer_slowWriteString(uint8_t *frameBuffer, uint16_t x, uint16_t y, const uint8_t *message, uint8_t messageLength);
void frameBuffer_slowWriteCharacter(uint8_t *frameBuffer, uint16_t x, uint16_t y, uint8_t characterIndex);
void framebuffer_drawVerticalLine(uint8_t *frameBuffer, uint16_t x, uint16_t y0, uint16_t y1, uint8_t mode);
void framebuffer_drawLine(uint8_t *frameBuffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t mode);
void framebuffer_drawRectangle(uint8_t *frameBuffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t mode);
