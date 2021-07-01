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

#pragma once

#include <string.h>

extern "C" {
    #include "drivers/display.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

void displayPortTestBufferSubstring(int x, int y, const char * expectedFormat, ...) __attribute__ ((format (printf, 3, 4)));

#define UNITTEST_DISPLAYPORT_ROWS 16
#define UNITTEST_DISPLAYPORT_COLS 30
#define UNITTEST_DISPLAYPORT_BUFFER_LEN (UNITTEST_DISPLAYPORT_ROWS * UNITTEST_DISPLAYPORT_COLS)

char testDisplayPortBuffer[UNITTEST_DISPLAYPORT_BUFFER_LEN];

static displayPort_t testDisplayPort;

static int displayPortTestGrab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int displayPortTestRelease(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int displayPortTestClearScreen(displayPort_t *displayPort, displayClearOption_e options)
{
    UNUSED(displayPort);
    UNUSED(options);
    memset(testDisplayPortBuffer, ' ', UNITTEST_DISPLAYPORT_BUFFER_LEN);
    return 0;
}

static int displayPortTestDrawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int displayPortTestScreenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int displayPortTestWriteString(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    UNUSED(displayPort);
    UNUSED(attr);
    for (unsigned int i = 0; i < strlen(s); i++) {
        testDisplayPortBuffer[(y * UNITTEST_DISPLAYPORT_COLS) + x + i] = s[i];
    }
    return 0;
}

static int displayPortTestWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    UNUSED(displayPort);
    UNUSED(attr);
    testDisplayPortBuffer[(y * UNITTEST_DISPLAYPORT_COLS) + x] = c;
    return 0;
}

static bool displayPortTestIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int displayPortTestHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static void displayPortTestRedraw(displayPort_t *displayPort)
{
    UNUSED(displayPort);
}

static uint32_t displayPortTestTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static const displayPortVTable_t testDisplayPortVTable = {
    .grab = displayPortTestGrab,
    .release = displayPortTestRelease,
    .clearScreen = displayPortTestClearScreen,
    .drawScreen = displayPortTestDrawScreen,
    .screenSize = displayPortTestScreenSize,
    .writeString = displayPortTestWriteString,
    .writeChar = displayPortTestWriteChar,
    .isTransferInProgress = displayPortTestIsTransferInProgress,
    .heartbeat = displayPortTestHeartbeat,
    .redraw = displayPortTestRedraw,
    .txBytesFree = displayPortTestTxBytesFree
};

displayPort_t *displayPortTestInit(void)
{
    displayInit(&testDisplayPort, &testDisplayPortVTable, DISPLAYPORT_DEVICE_TYPE_MAX7456);
    testDisplayPort.rows = UNITTEST_DISPLAYPORT_ROWS;
    testDisplayPort.cols = UNITTEST_DISPLAYPORT_COLS;
    return &testDisplayPort;
}

void displayPortTestPrint(void)
{
    for (int i = 0; i < UNITTEST_DISPLAYPORT_BUFFER_LEN; i++) {
        if (i > 0 && i % UNITTEST_DISPLAYPORT_COLS == 0) {
            printf("\n");
        }
        printf("%c", testDisplayPortBuffer[i]);
    }
    printf("\n\n");
}

void displayPortTestBufferIsEmpty()
{
    for (size_t i = 0; i < UNITTEST_DISPLAYPORT_BUFFER_LEN; i++) {
        EXPECT_EQ(' ', testDisplayPortBuffer[i]);
        if (testDisplayPortBuffer[i] != ' ') {
            testDisplayPortBuffer[UNITTEST_DISPLAYPORT_BUFFER_LEN - 1] = '\0';
            printf("FIRST ERROR AT:%d\r\n", (int)i);
            printf("DISPLAY:%s\r\n", testDisplayPortBuffer);
            break;
        }
    }
}

void displayPortTestBufferSubstring(int x, int y, const char * expectedFormat, ...)
{
    char expected[UNITTEST_DISPLAYPORT_BUFFER_LEN];

    va_list args;
    va_start(args, expectedFormat);
    vsnprintf(expected, UNITTEST_DISPLAYPORT_BUFFER_LEN, expectedFormat, args);
    va_end(args);

#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif

    for (size_t i = 0; i < strlen(expected); i++) {
        EXPECT_EQ(expected[i], testDisplayPortBuffer[(y * testDisplayPort.cols) + x + i]);
    }
}
