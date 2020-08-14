/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <stdlib.h>

#include <limits.h>

extern "C" {
    #include "build/build_config.h"

    #include "common/color.h"

    #include "drivers/light_ws2811strip.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    void updateLEDDMABuffer(ledStripFormatRGB_e ledFormat, rgbColor24bpp_t *color, unsigned ledIndex);
    void ignoreTaskShortExecTime(void) {}
    void ignoreTaskStateTime(void) {}
}

TEST(WS2812, updateDMABuffer) {
    // given
    rgbColor24bpp_t color1 = { .raw = {0xFF,0xAA,0x55} };

    // when
    updateLEDDMABuffer(LED_GRB, &color1, 0);

    // and
    uint8_t byteIndex = 0;

    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;

    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;

    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;
}

extern "C" {
rgbColor24bpp_t* hsvToRgb24(const hsvColor_t *c) {
    UNUSED(c);
    return NULL;
}

bool ws2811LedStripHardwareInit(ioTag_t ioTag) {
    UNUSED(ioTag);

    return true;
}

void ws2811LedStripDMAEnable(void) {}
}

