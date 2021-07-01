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

#include "platform.h"

#include "drivers/io_types.h"

#define WS2811_LED_STRIP_LENGTH    32

#define WS2811_BITS_PER_LED_MAX    32

#if defined(USE_WS2811_SINGLE_COLOUR)
#define WS2811_DATA_BUFFER_SIZE    1
#define WS2811_DMA_BUFFER_SIZE     (WS2811_DATA_BUFFER_SIZE * WS2811_BITS_PER_LED_MAX)
// Do 2 extra iterations of the DMA transfer with the output set to low to generate the > 50us delay.
#define WS2811_DELAY_ITERATIONS    2
#else
#define WS2811_DATA_BUFFER_SIZE    WS2811_LED_STRIP_LENGTH
// for 50us delay
#define WS2811_DELAY_BUFFER_LENGTH 42
// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes)
#define WS2811_DMA_BUFFER_SIZE     (WS2811_DATA_BUFFER_SIZE * WS2811_BITS_PER_LED_MAX + WS2811_DELAY_BUFFER_LENGTH)
#endif

#ifdef USE_LEDSTRIP_CACHE_MGMT
// WS2811_DMA_BUFFER_SIZE is multiples of uint32_t
// Number of bytes required for buffer
#define WS2811_DMA_BUF_BYTES              (WS2811_DMA_BUFFER_SIZE * sizeof (uint32_t))
// Number of bytes required to cache align buffer
#define WS2811_DMA_BUF_CACHE_ALIGN_BYTES  ((WS2811_DMA_BUF_BYTES + 0x20) & ~0x1f)
// Size of array to create a cache aligned buffer
#define WS2811_DMA_BUF_CACHE_ALIGN_LENGTH (WS2811_DMA_BUF_CACHE_ALIGN_BYTES / sizeof (uint32_t))
extern uint32_t ledStripDMABuffer[WS2811_DMA_BUF_CACHE_ALIGN_LENGTH];
#else
#if defined(STM32F1) || defined(STM32F3)
extern uint8_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#else
extern uint32_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#endif
#endif

#define WS2811_TIMER_MHZ           48
#define WS2811_CARRIER_HZ          800000

// Enumeration to match the string options defined in lookupLedStripFormatRGB in settings.c
typedef enum {
    LED_GRB,
    LED_RGB,
    LED_GRBW
} ledStripFormatRGB_e;

void ws2811LedStripInit(ioTag_t ioTag);
void ws2811LedStripEnable(void);

bool ws2811LedStripHardwareInit(ioTag_t ioTag);
void ws2811LedStripDMAEnable(void);

void ws2811UpdateStrip(ledStripFormatRGB_e ledFormat);

void setLedHsv(uint16_t index, const hsvColor_t *color);
void getLedHsv(uint16_t index, hsvColor_t *color);

void scaleLedValue(uint16_t index, const uint8_t scalePercent);
void setLedValue(uint16_t index, const uint8_t value);

void setStripColor(const hsvColor_t *color);
void setStripColors(const hsvColor_t *colors);

void setUsedLedCount(unsigned ledCount);

bool isWS2811LedStripReady(void);

extern volatile bool ws2811LedDataTransferInProgress;

extern uint16_t BIT_COMPARE_1;
extern uint16_t BIT_COMPARE_0;
