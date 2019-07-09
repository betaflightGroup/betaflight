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

/** PAL or NTSC, value is number of chars total */
#define VIDEO_BUFFER_CHARS_NTSC   390
#define VIDEO_BUFFER_CHARS_PAL    480
#define VIDEO_LINES_NTSC          13
#define VIDEO_LINES_PAL           16

extern uint16_t maxScreenSize;

struct vcdProfile_s;
void    max7456HardwareReset(void);
struct max7456Config_s;
void    max7456PreInit(const struct max7456Config_s *max7456Config);
bool    max7456Init(const struct max7456Config_s *max7456Config, const struct vcdProfile_s *vcdProfile, bool cpuOverclock);
void    max7456Invert(bool invert);
void    max7456Brightness(uint8_t black, uint8_t white);
void    max7456DrawScreen(void);
void    max7456WriteNvm(uint16_t char_address, const uint8_t *font_data);
void    max7456ReadNvm(uint16_t char_address, uint8_t *font_data);
uint8_t max7456GetRowsCount(void);
void    max7456Write(uint8_t x, uint8_t y, const char *buff);
void    max7456WriteChar(uint8_t x, uint8_t y, uint8_t c);
#ifdef USE_MAX7456_EXTENDED
void    max7456ReadShadow(uint8_t *font_data);
void    max7456WriteExtended(uint8_t x, uint8_t y, const char *buff);
void    max7456WriteCharExtended(uint8_t x, uint8_t y,uint8_t extendedChar);
#endif
void    max7456ClearScreen(void);
void    max7456RefreshAll(void);
bool    max7456DmaInProgress(void);
bool    max7456BuffersSynced(void);
bool    max7456UseExtended(void);
bool    max7456IsExtended(void);
bool    max7456CheckIsChipAlive(void);
bool    max7456WasDeadAtInit(void);
