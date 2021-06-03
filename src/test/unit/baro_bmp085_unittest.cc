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

void bmp085Calculate(int32_t *pressure, int32_t *temperature);
extern uint32_t bmp085_up;
extern uint16_t bmp085_ut;

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp085_smd500_calibration_param_t;

typedef struct {
    bmp085_smd500_calibration_param_t cal_param;
    uint8_t mode;
    uint8_t chip_id, ml_version, al_version;
    uint8_t dev_addr;
    int32_t param_b5;
    int16_t oversampling_setting;
} bmp085_t;

bmp085_t bmp085;

}


#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(baroBmp085Test, TestBmp085CalculateOss0)
{

    // given
    int32_t pressure, temperature;
    bmp085_up = 23843; // Digital pressure value
    bmp085_ut = 27898; // Digital temperature value

    // and
    bmp085.cal_param.ac1 = 408;
    bmp085.cal_param.ac2 = -72;
    bmp085.cal_param.ac3 = -14383;
    bmp085.cal_param.ac4 = 32741;
    bmp085.cal_param.ac5 = 32757;
    bmp085.cal_param.ac6 = 23153;
    bmp085.cal_param.b1 = 6190;
    bmp085.cal_param.b2 = 4;
    bmp085.cal_param.mb = -32767;
    bmp085.cal_param.mc = -8711;
    bmp085.cal_param.md = 2868;
    bmp085.oversampling_setting = 0;

    // when
    bmp085Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(69964, pressure); // Datasheet says 69965
    EXPECT_EQ(1500, temperature);

}

TEST(baroBmp085Test, TestBmp085CalculateOss3)
{

    // given
    int32_t pressure, temperature;
    bmp085_up = 271097; // Digital pressure value
    bmp085_ut = 27898; // Digital temperature value

    // and
    bmp085.cal_param.ac1 = 408;
    bmp085.cal_param.ac2 = -72;
    bmp085.cal_param.ac3 = -14383;
    bmp085.cal_param.ac4 = 32741;
    bmp085.cal_param.ac5 = 32757;
    bmp085.cal_param.ac6 = 23153;
    bmp085.cal_param.b1 = 6190;
    bmp085.cal_param.b2 = 4;
    bmp085.cal_param.mb = -32767;
    bmp085.cal_param.mc = -8711;
    bmp085.cal_param.md = 2868;
    bmp085.oversampling_setting = 3;

    // when
    bmp085Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(99998, pressure);
    EXPECT_EQ(1500, temperature);

}

TEST(baroBmp085Test, TestBmp085CalculateOss3Cold)
{

    // given
    int32_t pressure, temperature;
    bmp085_up = 271097; // Digital pressure value
    bmp085_ut = 24342; // Digital temperature value  24342 = -20degC    27898 = 15degC

    // and
    bmp085.cal_param.ac1 = 408;
    bmp085.cal_param.ac2 = -72;
    bmp085.cal_param.ac3 = -14383;
    bmp085.cal_param.ac4 = 32741;
    bmp085.cal_param.ac5 = 32757;
    bmp085.cal_param.ac6 = 23153;
    bmp085.cal_param.b1 = 6190;
    bmp085.cal_param.b2 = 4;
    bmp085.cal_param.mb = -32767;
    bmp085.cal_param.mc = -8711;
    bmp085.cal_param.md = 2868;
    bmp085.oversampling_setting = 3;

    // when
    bmp085Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(92251, pressure);
    EXPECT_EQ(-2006, temperature); // -20.06 degC

}

TEST(baroBmp085Test, TestBmp085CalculateOss3Hot)
{

    // given
    int32_t pressure, temperature;
    bmp085_up = 271097; // Digital pressure value
    bmp085_ut = 33315; // Digital temperature value

    // and
    bmp085.cal_param.ac1 = 408;
    bmp085.cal_param.ac2 = -72;
    bmp085.cal_param.ac3 = -14383;
    bmp085.cal_param.ac4 = 32741;
    bmp085.cal_param.ac5 = 32757;
    bmp085.cal_param.ac6 = 23153;
    bmp085.cal_param.b1 = 6190;
    bmp085.cal_param.b2 = 4;
    bmp085.cal_param.mb = -32767;
    bmp085.cal_param.mc = -8711;
    bmp085.cal_param.md = 2868;
    bmp085.oversampling_setting = 3;

    // when
    bmp085Calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(108922, pressure);
    EXPECT_EQ(5493, temperature); // 54.93 degC

}

// STUBS

extern "C" {

    void gpioInit() {}
    void RCC_APB2PeriphClockCmd() {}
    void delay(uint32_t) {}
    void delayMicroseconds(uint32_t) {}
    bool busReadRegisterBuffer(const extDevice_t*, uint8_t, uint8_t*, uint8_t) {return true;}
    bool busWriteRegister(const extDevice_t*, uint8_t, uint8_t) {return true;}
    void IOConfigGPIO() {}
    void IOHi() {}
    void IOLo() {}
    void IOInit() {}
    void IOGetByTag() {}
    bool busBusy(const extDevice_t*, bool*) {return false;}
    void busDeviceRegister(const extDevice_t*) {}
    bool busReadRegisterBufferStart(const extDevice_t*, uint8_t, uint8_t*, uint8_t) {return true;}
    bool busWriteRegisterStart(const extDevice_t*, uint8_t, uint8_t) {return true;}
}
