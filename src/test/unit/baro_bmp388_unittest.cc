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

extern "C" {

#include "platform.h"
#include "target.h"
#include "drivers/barometer/barometer.h"
#include "drivers/bus.h"

void bmp388Calculate(int32_t *pressure, int32_t *temperature);

extern uint32_t bmp388_up;
extern uint32_t bmp388_ut;
extern int64_t t_lin; // when temperature is calculated this is updated, the result is used in the pressure calculations

typedef struct bmp388_calib_param_s {
    uint16_t T1;
    uint16_t T2;
    int8_t T3;
    int16_t P1;
    int16_t P2;
    int8_t P3;
    int8_t P4;
    uint16_t P5;
    uint16_t P6;
    int8_t P7;
    int8_t P8;
    int16_t P9;
    int8_t P10;
    int8_t P11;
} __attribute__((packed)) bmp388_calib_param_t; // packed as we read directly from the device into this structure.

bmp388_calib_param_t bmp388_cal;

} // extern "C"


#include "unittest_macros.h"
#include "gtest/gtest.h"

void baroBmp388ConfigureZeroCalibration(void)
{
    bmp388_cal = {
        .T1 = 0,
        .T2 = 0,
        .T3 = 0,
        .P1 = 0,
        .P2 = 0,
        .P3 = 0,
        .P4 = 0,
        .P5 = 0,
        .P6 = 0,
        .P7 = 0,
        .P8 = 0,
        .P9 = 0,
        .P10 = 0,
        .P11 = 0
    };
}

void baroBmp388ConfigureSampleCalibration(void)
{
    bmp388_cal = {
        .T1 = 27772,
        .T2 = 18638,
        .T3 = -10,
        .P1 = 878,
        .P2 = -2023,
        .P3 = 35,
        .P4 = 0,
        .P5 = 24476,
        .P6 = 30501,
        .P7 = -13,
        .P8 = -10,
        .P9 = 16545,
        .P10 = 21,
        .P11 = -60
    };
}

TEST(baroBmp388Test, TestBmp388CalculateWithZeroCalibration)
{
    // given
    int32_t pressure, temperature;
    bmp388_up = 0; // uncompensated pressure value
    bmp388_ut = 0; // uncompensated temperature value
    t_lin = 0;

    // and
    baroBmp388ConfigureZeroCalibration();

    // when
    bmp388Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(0, temperature);
    EXPECT_EQ(0, t_lin);

    // and
    EXPECT_EQ(0, pressure);
}

TEST(baroBmp388Test, TestBmp388CalculateWithSampleCalibration)
{
    // given
    int32_t pressure, temperature;
    bmp388_up = 7323488; // uncompensated pressure value
    bmp388_ut = 9937920; // uncompensated temperature value
    t_lin = 0;

    // and
    baroBmp388ConfigureSampleCalibration();

    // when
    bmp388Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(4880, temperature); // 48.80 degrees C
    EXPECT_NE(0, t_lin);

    // and
    EXPECT_EQ(39535, pressure); // 39535 Pa
}

// STUBS

extern "C" {

void delay() {}

bool busBusy(const extDevice_t*, bool*) {return false;}
bool busReadRegisterBuffer(const extDevice_t*, uint8_t, uint8_t*, uint8_t) {return true;}
bool busReadRegisterBufferStart(const extDevice_t*, uint8_t, uint8_t*, uint8_t) {return true;}
bool busWriteRegister(const extDevice_t*, uint8_t, uint8_t) {return true;}
bool busWriteRegisterStart(const extDevice_t*, uint8_t, uint8_t) {return true;}
void busDeviceRegister(const extDevice_t*) {}

uint16_t spiCalculateDivider() {
    return 2;
}

void spiSetClkDivisor() {
}

void spiPreinitByIO(IO_t) {
}

void IOConfigGPIO() {
}

void IOHi() {
}

IO_t IOGetByTag(ioTag_t) {
    return IO_NONE;
}

void IOInit() {
}

void EXTIHandlerInit(extiCallbackRec_t *, extiHandlerCallback *) {
}

void EXTIConfig(IO_t, extiCallbackRec_t *, int, ioConfig_t, extiTrigger_t) {
}

void EXTIEnable(IO_t, bool) {
}


} // extern "C"
