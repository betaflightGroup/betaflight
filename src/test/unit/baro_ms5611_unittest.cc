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

int8_t ms5611CRC(uint16_t *prom);
void ms5611Calculate(int32_t *pressure, int32_t *temperature);

extern uint16_t ms5611_c[8];
extern uint32_t ms5611_up;
extern uint32_t ms5611_ut;

}


#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(baroMS5611Test, TestValidMs5611Crc)
{
    // given
    uint16_t ms5611Prom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x450B};

    // when
    int8_t result = ms5611CRC(ms5611Prom);

    // then
    EXPECT_EQ(0, result);
}

TEST(baroMS5611Test, TestInvalidMs5611Crc)
{
    // given
    uint16_t ms5611Prom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500};

    // when
    int8_t result = ms5611CRC(ms5611Prom);

    // then
    EXPECT_EQ(-1, result);
}

TEST(baroMS5611Test, TestMs5611AllZeroProm)
{
    // given
    uint16_t ms5611Prom[] = {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};

    // when
    int8_t result = ms5611CRC(ms5611Prom);

    // then
    EXPECT_EQ(-1, result);
}

TEST(baroMS5611Test, TestMs5611AllOnesProm)
{
    // given
    uint16_t ms5611Prom[] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};

    // when
    int8_t result = ms5611CRC(ms5611Prom);

    // then
    EXPECT_EQ(-1, result);
}

TEST(baroMS5611Test, TestMs5611CalculatePressureGT20Deg)
{
    // given
    int32_t pressure, temperature;
    uint16_t ms5611_c_test[] = {0x0000, 40127, 36924, 23317, 23282, 33464, 28312, 0x0000}; // calibration data from MS5611 datasheet
    memcpy(&ms5611_c, &ms5611_c_test, sizeof(ms5611_c_test));

    ms5611_up = 9085466; // Digital pressure value from MS5611 datasheet
    ms5611_ut = 8569150; // Digital temperature value from MS5611 datasheet

    // when
    ms5611Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(2007, temperature); // 20.07 deg C
    EXPECT_EQ(100009, pressure);  // 1000.09 mbar
}

TEST(baroMS5611Test, TestMs5611CalculatePressureLT20Deg)
{
    // given
    int32_t pressure, temperature;
    uint16_t ms5611_c_test[] = {0x0000, 40127, 36924, 23317, 23282, 33464, 28312, 0x0000}; // calibration data from MS5611 datasheet
    memcpy(&ms5611_c, &ms5611_c_test, sizeof(ms5611_c_test));

    ms5611_up = 9085466; // Digital pressure value from MS5611 datasheet
    ms5611_ut = 8069150; // Digital temperature value

    // when
    ms5611Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(205, temperature); // 2.05 deg C
    EXPECT_EQ(96512, pressure);  // 965.12 mbar
}

TEST(baroMS5611Test, TestMs5611CalculatePressureLTMinus15Deg)
{
    // given
    int32_t pressure, temperature;
    uint16_t ms5611_c_test[] = {0x0000, 40127, 36924, 23317, 23282, 33464, 28312, 0x0000}; // calibration data from MS5611 datasheet
    memcpy(&ms5611_c, &ms5611_c_test, sizeof(ms5611_c_test));

    ms5611_up = 9085466; // Digital pressure value from MS5611 datasheet
    ms5611_ut = 7369150; // Digital temperature value

    // when
    ms5611Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(-2710, temperature); // -27.10 deg C
    EXPECT_EQ(90613, pressure);  // 906.13 mbar
}

// STUBS

extern "C" {

void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}

bool busBusy(const extDevice_t*, bool*) {return false;}
bool busRawReadRegisterBuffer(const extDevice_t*, uint8_t, uint8_t*, uint8_t) {return true;}
bool busRawReadRegisterBufferStart(const extDevice_t*, uint8_t, uint8_t*, uint8_t) {return true;}
bool busRawWriteRegister(const extDevice_t*, uint8_t, uint8_t) {return true;}
bool busRawWriteRegisterStart(const extDevice_t*, uint8_t, uint8_t) {return true;}
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
