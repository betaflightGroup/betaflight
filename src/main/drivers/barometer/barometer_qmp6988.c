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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "barometer.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "drivers/serial.h"
#include "io/serial.h"
#include "common/printf.h"
#include "barometer_qmp6988.h"

#if defined(USE_BARO) && (defined(USE_BARO_QMP6988) || defined(USE_BARO_SPI_QMP6988))

// 10 MHz max SPI frequency
#define QMP6988_MAX_SPI_CLK_HZ 10000000

#define QMP6988_I2C_ADDR                0x70
#define QMP6988_DEFAULT_CHIP_ID         0x5c
#define QMP6988_CHIP_ID_REG             0xD1  /* Chip ID Register */

#define QMP6988_IO_SETUP_REG            0xF5
#define QMP6988_SET_IIR_REG             0xF1
#define QMP6988_CTRL_MEAS_REG           0xF4
#define QMP6988_COE_B00_1_REG           0xA0
#define QMP6988_PRESSURE_MSB_REG        0xF7  /* Pressure MSB Register */
#define QMP6988_PRESSURE_LSB_REG        0xF8  /* Pressure LSB Register */
#define QMP6988_PRESSURE_XLSB_REG       0xF9  /* Pressure XLSB Register */
#define QMP6988_TEMPERATURE_MSB_REG     0xFA  /* Temperature MSB Reg */
#define QMP6988_TEMPERATURE_LSB_REG     0xFB  /* Temperature LSB Reg */
#define QMP6988_TEMPERATURE_XLSB_REG    0xFC  /* Temperature XLSB Reg */
#define QMP6988_DATA_FRAME_SIZE         6
#define QMP6988_FORCED_MODE             0x01
#define QMP6988_PWR_SAMPLE_MODE         0x7B

#define QMP6988_OVERSAMP_SKIPPED        0x00
#define QMP6988_OVERSAMP_1X             0x01
#define QMP6988_OVERSAMP_2X             0x02
#define QMP6988_OVERSAMP_4X             0x03
#define QMP6988_OVERSAMP_8X             0x04
#define QMP6988_OVERSAMP_16X            0x05

// configure pressure and temperature oversampling, forced sampling mode
#define QMP6988_PRESSURE_OSR            QMP6988_OVERSAMP_8X
#define QMP6988_TEMPERATURE_OSR         QMP6988_OVERSAMP_1X
#define QMP6988_MODE                    (QMP6988_PRESSURE_OSR << 2 | QMP6988_TEMPERATURE_OSR << 5 | QMP6988_FORCED_MODE)

#define T_INIT_MAX                      20
#define T_MEASURE_PER_OSRS_MAX          37
#define T_SETUP_PRESSURE_MAX            10

typedef struct qmp6988_calib_param_s {
    float Coe_a0;
    float Coe_a1;
    float Coe_a2;
    float Coe_b00;
    float Coe_bt1;
    float Coe_bt2;
    float Coe_bp1;
    float Coe_b11;
    float Coe_bp2;
    float Coe_b12;
    float Coe_b21;
    float Coe_bp3;
} qmp6988_calib_param_t;

static uint8_t qmp6988_chip_id = 0;
STATIC_UNIT_TESTED qmp6988_calib_param_t qmp6988_cal;
// uncompensated pressure and temperature
int32_t qmp6988_up = 0;
int32_t qmp6988_ut = 0;
static DMA_DATA_ZERO_INIT uint8_t sensor_data[QMP6988_DATA_FRAME_SIZE];

static void qmp6988StartUT(baroDev_t *baro);
static bool qmp6988ReadUT(baroDev_t *baro);
static bool qmp6988GetUT(baroDev_t *baro);
static void qmp6988StartUP(baroDev_t *baro);
static bool qmp6988ReadUP(baroDev_t *baro);
static bool qmp6988GetUP(baroDev_t *baro);

STATIC_UNIT_TESTED void qmp6988Calculate(int32_t *pressure, int32_t *temperature);

void qmp6988BusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_QMP6988
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin);
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(QMP6988_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

void qmp6988BusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_QMP6988
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
        IORelease(dev->busType_u.spi.csnPin);
        IOInit(dev->busType_u.spi.csnPin, OWNER_PREINIT, 0);
    }
#else
    UNUSED(dev);
#endif
}

bool qmp6988Detect(baroDev_t *baro)
{
    uint8_t databuf[25] = {0};
    int Coe_a0_;
    int Coe_a1_;
    int Coe_a2_;
    int Coe_b00_;
    int Coe_bt1_;
    int Coe_bt2_;
    int Coe_bp1_;
    int Coe_b11_;
    int Coe_bp2_;
    int Coe_b12_;
    int Coe_b21_;
    int Coe_bp3_;
    uint16_t lb=0,hb=0;
    uint32_t lw=0,hw=0,temp1,temp2;

    delay(20);

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    qmp6988BusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        dev->busType_u.i2c.address = QMP6988_I2C_ADDR;
        defaultAddressApplied = true;
    }

    busReadRegisterBuffer(dev, QMP6988_CHIP_ID_REG, &qmp6988_chip_id, 1);  /* read Chip Id */

    if (qmp6988_chip_id != QMP6988_DEFAULT_CHIP_ID) {
        qmp6988BusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // SetIIR
    busWriteRegister(dev, QMP6988_SET_IIR_REG, 0x05);

    //read OTP
    busReadRegisterBuffer(dev, QMP6988_COE_B00_1_REG, databuf, 25);

    //algo OTP
    hw = databuf[0];
    lw =  databuf[1];
    temp1 = hw<<12 | lw<<4;

    hb = databuf[2];
    lb = databuf[3];
    Coe_bt1_ = hb<<8 | lb;

    hb = databuf[4];
    lb = databuf[5];
    Coe_bt2_ = hb<<8 | lb;

    hb = databuf[6];
    lb = databuf[7];
    Coe_bp1_ = hb<<8 | lb;

    hb = databuf[8];
    lb = databuf[9];
    Coe_b11_ = hb<<8 | lb;

    hb = databuf[10];
    lb = databuf[11];
    Coe_bp2_ = hb<<8 | lb;

    hb = databuf[12];
    lb = databuf[13];
    Coe_b12_ = hb<<8 | lb;

    hb = databuf[14];
    lb = databuf[15];
    Coe_b21_ = hb<<8 | lb;

    hb = databuf[16];
    lb = databuf[17];
    Coe_bp3_ = hb<<8 | lb;

    hw = databuf[18];
    lw = databuf[19];
    temp2 = hw<<12 | lw<<4;

    hb = databuf[20];
    lb = databuf[21];
    Coe_a1_ = hb<<8 | lb;

    hb = databuf[22];
    lb = databuf[23];
    Coe_a2_ = hb<<8 | lb;

    hb = databuf[24];

    temp1 = temp1|((hb&0xf0)>>4);
    if(temp1&0x80000)
       Coe_b00_ = ((int)temp1 - (int)0x100000);
    else
       Coe_b00_ = temp1;

    temp2 = temp2|(hb&0x0f);
    if(temp2&0x80000)
        Coe_a0_  = ((int)temp2 - (int)0x100000);
    else
        Coe_a0_ = temp2;

    qmp6988_cal.Coe_a0=(float)Coe_a0_/16.0;
    qmp6988_cal.Coe_a1=(-6.30E-03)+(4.30E-04)*(float)Coe_a1_/32767.0;
    qmp6988_cal.Coe_a2=(-1.9E-11)+(1.2E-10)*(float)Coe_a2_/32767.0;

    qmp6988_cal.Coe_b00 = Coe_b00_/16.0;
    qmp6988_cal.Coe_bt1 = (1.00E-01)+(9.10E-02)*(float)Coe_bt1_/32767.0;
    qmp6988_cal.Coe_bt2= (1.20E-08)+(1.20E-06)*(float)Coe_bt2_/32767.0;

    qmp6988_cal.Coe_bp1 = (3.30E-02)+(1.90E-02)*(float)Coe_bp1_/32767.0;
    qmp6988_cal.Coe_b11= (2.10E-07)+(1.40E-07)*(float)Coe_b11_/32767.0;

    qmp6988_cal.Coe_bp2 = (-6.30E-10)+(3.50E-10)*(float)Coe_bp2_/32767.0;
    qmp6988_cal.Coe_b12= (2.90E-13)+(7.60E-13)*(float)Coe_b12_/32767.0;

    qmp6988_cal.Coe_b21 = (2.10E-15)+(1.20E-14)*(float)Coe_b21_/32767.0;
    qmp6988_cal.Coe_bp3= (1.30E-16)+(7.90E-17)*(float)Coe_bp3_/32767.0;

    // Set power mode and sample times
    busWriteRegister(dev, QMP6988_CTRL_MEAS_REG, QMP6988_PWR_SAMPLE_MODE);

    // these are dummy as temperature is measured as part of pressure
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = qmp6988StartUT;
    baro->read_ut = qmp6988ReadUT;
    baro->get_ut = qmp6988GetUT;
    // only _up part is executed, and gets both temperature and pressure
    baro->start_up = qmp6988StartUP;
    baro->read_up = qmp6988ReadUP;
    baro->get_up = qmp6988GetUP;
    baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << QMP6988_TEMPERATURE_OSR) >> 1) + ((1 << QMP6988_PRESSURE_OSR) >> 1)) + (QMP6988_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;
    baro->calculate = qmp6988Calculate;

    return true;
}

static void qmp6988StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
}

static bool qmp6988ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
    return true;
}

static bool qmp6988GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
    return true;
}

static void qmp6988StartUP(baroDev_t *baro)
{
    // start measurement
    busWriteRegister(&baro->dev, QMP6988_CTRL_MEAS_REG, QMP6988_PWR_SAMPLE_MODE);
}

static bool qmp6988ReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // read data from sensor
    busReadRegisterBufferStart(&baro->dev, QMP6988_PRESSURE_MSB_REG, sensor_data, QMP6988_DATA_FRAME_SIZE);

    return true;
}

static bool qmp6988GetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    qmp6988_up = sensor_data[0] << 16 | sensor_data[1] << 8 | sensor_data[2];
    qmp6988_ut = sensor_data[3] << 16 | sensor_data[4] << 8 | sensor_data[5];

    return true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static float qmp6988CompensateTemperature(int32_t adc_T)
{
    int32_t var1;
    float T;

    var1=adc_T-1024*1024*8;
    T= qmp6988_cal.Coe_a0+qmp6988_cal.Coe_a1*var1+qmp6988_cal.Coe_a2*var1*var1;

    return T;
}



STATIC_UNIT_TESTED void qmp6988Calculate(int32_t *pressure, int32_t *temperature)
{
    float tr,pr;
    int32_t Dp;

    tr = qmp6988CompensateTemperature(qmp6988_ut);
    Dp = qmp6988_up - 1024*1024*8;

    pr = qmp6988_cal.Coe_b00+qmp6988_cal.Coe_bt1*tr+qmp6988_cal.Coe_bp1*Dp+qmp6988_cal.Coe_b11*tr*Dp+qmp6988_cal.Coe_bt2*tr*tr+qmp6988_cal.Coe_bp2*Dp*Dp+qmp6988_cal.Coe_b12*Dp*tr*tr
    +qmp6988_cal.Coe_b21*Dp*Dp*tr+qmp6988_cal.Coe_bp3*Dp*Dp*Dp;

    if (pr)
    *pressure = (int32_t)(pr);
    if (tr)
    *temperature = (int32_t)tr/256;
}

#endif
