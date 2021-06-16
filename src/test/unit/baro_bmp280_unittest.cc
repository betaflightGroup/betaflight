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

void bmp280Calculate(int32_t *pressure, int32_t *temperature);

extern uint32_t bmp280_up;
extern uint32_t bmp280_ut;
extern int32_t t_fine; /* calibration t_fine data */

typedef struct bmp280_calib_param_s {
    uint16_t dig_T1; /* calibration T1 data */
    int16_t dig_T2; /* calibration T2 data */
    int16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1; /* calibration P1 data */
    int16_t dig_P2; /* calibration P2 data */
    int16_t dig_P3; /* calibration P3 data */
    int16_t dig_P4; /* calibration P4 data */
    int16_t dig_P5; /* calibration P5 data */
    int16_t dig_P6; /* calibration P6 data */
    int16_t dig_P7; /* calibration P7 data */
    int16_t dig_P8; /* calibration P8 data */
    int16_t dig_P9; /* calibration P9 data */
} __attribute__((packed)) bmp280_calib_param_t; // packed as we read directly from the device into this structure.

bmp280_calib_param_t bmp280_cal;
}


#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(baroBmp280Test, TestBmp280Calculate)
{
    // given
    int32_t pressure, temperature;
    bmp280_up = 415148; // Digital pressure value
    bmp280_ut = 519888; // Digital temperature value
    t_fine = 0;

    // and
    bmp280_cal.dig_T1 = 27504;
    bmp280_cal.dig_T2 = 26435;
    bmp280_cal.dig_T3 = -1000;
    bmp280_cal.dig_P1 = 36477;
    bmp280_cal.dig_P2 = -10685;
    bmp280_cal.dig_P3 = 3024;
    bmp280_cal.dig_P4 = 2855;
    bmp280_cal.dig_P5 = 140;
    bmp280_cal.dig_P6 = -7;
    bmp280_cal.dig_P7 = 15500;
    bmp280_cal.dig_P8 = -14600;
    bmp280_cal.dig_P9 = 6000;

    // when
    bmp280Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(100653, pressure); // 100653 Pa
    EXPECT_EQ(2508, temperature); // 25.08 degC
}

TEST(baroBmp280Test, TestBmp280CalculateHighP)
{
    // given
    int32_t pressure, temperature;
    bmp280_up = 215148; // Digital pressure value
    bmp280_ut = 519888; // Digital temperature value
    t_fine = 0;

    // and
    bmp280_cal.dig_T1 = 27504;
    bmp280_cal.dig_T2 = 26435;
    bmp280_cal.dig_T3 = -1000;
    bmp280_cal.dig_P1 = 36477;
    bmp280_cal.dig_P2 = -10685;
    bmp280_cal.dig_P3 = 3024;
    bmp280_cal.dig_P4 = 2855;
    bmp280_cal.dig_P5 = 140;
    bmp280_cal.dig_P6 = -7;
    bmp280_cal.dig_P7 = 15500;
    bmp280_cal.dig_P8 = -14600;
    bmp280_cal.dig_P9 = 6000;

    // when
    bmp280Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(135382, pressure); // 135385 Pa
    EXPECT_EQ(2508, temperature); // 25.08 degC
}

TEST(baroBmp280Test, TestBmp280CalculateZeroP)
{
    // given
    int32_t pressure, temperature;
    bmp280_up = 415148; // Digital pressure value
    bmp280_ut = 519888; // Digital temperature value
    t_fine = 0;

    // and
    bmp280_cal.dig_T1 = 27504;
    bmp280_cal.dig_T2 = 26435;
    bmp280_cal.dig_T3 = -1000;
    bmp280_cal.dig_P1 = 0;
    bmp280_cal.dig_P2 = -10685;
    bmp280_cal.dig_P3 = 3024;
    bmp280_cal.dig_P4 = 2855;
    bmp280_cal.dig_P5 = 140;
    bmp280_cal.dig_P6 = -7;
    bmp280_cal.dig_P7 = 15500;
    bmp280_cal.dig_P8 = -14600;
    bmp280_cal.dig_P9 = 6000;

    // when
    bmp280Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(0, pressure); // P1=0 trips pressure to 0 Pa, avoiding division by zero
    EXPECT_EQ(2508, temperature); // 25.08 degC
}

// STUBS

extern "C" {

void delay(uint32_t) {}
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

void spiPreinitByIO() {
}

void IOConfigGPIO() {
}

void IOHi() {
}

void IOInit() {
}

void IORelease() {
}


}
