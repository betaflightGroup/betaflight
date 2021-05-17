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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#include "barometer.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "barometer_bmp085.h"

#ifdef USE_BARO

static bool isConversionComplete = false;
static bool isEOCConnected = false;
#define BMP085_DATA_TEMP_SIZE 2
#define BMP085_DATA_PRES_SIZE 3
#define BMP085_DATA_FRAME_SIZE 3
static uint8_t sensor_data[BMP085_DATA_FRAME_SIZE];

#ifdef USE_EXTI
static IO_t eocIO;
static extiCallbackRec_t exti;

static void bmp085ExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);
    isConversionComplete = true;
}
#endif

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

#define BMP085_I2C_ADDR         0x77
#define BMP085_CHIP_ID          0x55
#define BOSCH_PRESSURE_BMP085   85
#define BMP085_CHIP_ID_REG      0xD0
#define BMP085_VERSION_REG      0xD1
#define E_SENSOR_NOT_DETECTED   (char) 0
#define BMP085_PROM_START__ADDR 0xaa
#define BMP085_PROM_DATA__LEN   22
#define BMP085_T_MEASURE        0x2E                // temperature measurent
#define BMP085_P_MEASURE        0x34                // pressure measurement
#define BMP085_CTRL_MEAS_REG    0xF4
#define BMP085_ADC_OUT_MSB_REG  0xF6
#define BMP085_ADC_OUT_LSB_REG  0xF7
#define BMP085_CHIP_ID__POS     0
#define BMP085_CHIP_ID__MSK     0xFF
#define BMP085_CHIP_ID__LEN     8
#define BMP085_CHIP_ID__REG     BMP085_CHIP_ID_REG

#define BMP085_ML_VERSION__POS      0
#define BMP085_ML_VERSION__LEN      4
#define BMP085_ML_VERSION__MSK      0x0F
#define BMP085_ML_VERSION__REG      BMP085_VERSION_REG

#define BMP085_AL_VERSION__POS      4
#define BMP085_AL_VERSION__LEN      4
#define BMP085_AL_VERSION__MSK      0xF0
#define BMP085_AL_VERSION__REG      BMP085_VERSION_REG

#define BMP085_GET_BITSLICE(regvar, bitname) (regvar & bitname##__MSK) >> bitname##__POS
#define BMP085_SET_BITSLICE(regvar, bitname, val) (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

#define SMD500_PARAM_MG      3038        //calibration parameter
#define SMD500_PARAM_MH     -7357        //calibration parameter
#define SMD500_PARAM_MI      3791        //calibration parameter

STATIC_UNIT_TESTED bmp085_t bmp085;

#define UT_DELAY    6000        // 1.5ms margin according to the spec (4.5ms T conversion time)
#define UP_DELAY    27000       // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P conversion time with OSS=3)

static bool bmp085InitDone = false;
STATIC_UNIT_TESTED uint16_t bmp085_ut;  // static result of temperature measurement
STATIC_UNIT_TESTED uint32_t bmp085_up;  // static result of pressure measurement

static void bmp085ReadCalibrarionParameters(extDevice_t *dev);
static void bmp085StartUT(baroDev_t *baro);
static bool bmp085ReadUT(baroDev_t *baro);
static bool bmp085GetUT(baroDev_t *baro);
static void bmp085StartUP(baroDev_t *baro);
static bool bmp085ReadUP(baroDev_t *baro);
static bool bmp085GetUP(baroDev_t *baro);
static int32_t bmp085GetTemperature(uint32_t ut);
static int32_t bmp085GetPressure(uint32_t up);
STATIC_UNIT_TESTED void bmp085Calculate(int32_t *pressure, int32_t *temperature);

static bool bmp085TestEOCConnected(baroDev_t *baro, const bmp085Config_t *config);

static IO_t xclrIO = IO_NONE;
#define BMP085_OFF  IOLo(xclrIO);
#define BMP085_ON   IOHi(xclrIO);

static void bmp085InitXclrIO(const bmp085Config_t *config)
{
    xclrIO = IOGetByTag(config->xclrTag);
    IOInit(xclrIO, OWNER_BARO_XCLR, 0);
    IOConfigGPIO(xclrIO, IOCFG_OUT_PP);
}

bool bmp085Detect(const bmp085Config_t *config, baroDev_t *baro)
{
    uint8_t data;
    bool ack;
    bool defaultAddressApplied = false;

    if (bmp085InitDone) {
        return true;
    }

    bmp085InitXclrIO(config);
    BMP085_ON;   // enable baro

#ifdef USE_EXTI
    // EXTI interrupt for barometer EOC

    eocIO = IOGetByTag(config->eocTag);
    IOInit(eocIO, OWNER_BARO_EOC, 0);
    EXTIHandlerInit(&exti, bmp085ExtiHandler);
    EXTIConfig(eocIO, &exti, NVIC_PRIO_BARO_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(eocIO, true);
#else
    UNUSED(config);
#endif

    delay(20); // datasheet says 10ms, we'll be careful and do 20.

    extDevice_t *dev = &baro->dev;

    if ((dev->bus->busType == BUSTYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for BMP085
        dev->busType_u.i2c.address = BMP085_I2C_ADDR;
        defaultAddressApplied = true;
    }

    ack = busReadRegisterBuffer(dev, BMP085_CHIP_ID__REG, &data, 1); /* read Chip Id */
    if (ack) {
        bmp085.chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
        bmp085.oversampling_setting = 3;

        if (bmp085.chip_id == BMP085_CHIP_ID) { /* get bitslice */
            busDeviceRegister(dev);

            busReadRegisterBuffer(dev, BMP085_VERSION_REG, &data, 1); /* read Version reg */
            bmp085.ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION); /* get ML Version */
            bmp085.al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION); /* get AL Version */
            bmp085ReadCalibrarionParameters(dev); /* readout bmp085 calibparam structure */
            baro->ut_delay = UT_DELAY;
            baro->up_delay = UP_DELAY;
            baro->start_ut = bmp085StartUT;
            baro->read_ut = bmp085ReadUT;
            baro->get_ut = bmp085GetUT;
            baro->start_up = bmp085StartUP;
            baro->read_up = bmp085ReadUP;
            baro->get_up = bmp085GetUP;
            baro->calculate = bmp085Calculate;

            isEOCConnected = bmp085TestEOCConnected(baro, config);

            bmp085InitDone = true;
            return true;
        }
    }

#ifdef USE_EXTI
    if (eocIO) {
        IORelease(eocIO);
        EXTIRelease(eocIO);
    }
#endif

    BMP085_OFF;

    if (defaultAddressApplied) {
        dev->busType_u.i2c.address = 0;
    }

    return false;
}

static int32_t bmp085GetTemperature(uint32_t ut)
{
    int32_t temperature;
    int32_t x1, x2;

    x1 = (((int32_t) ut - (int32_t) bmp085.cal_param.ac6) * (int32_t) bmp085.cal_param.ac5) >> 15;
    x2 = ((int32_t) bmp085.cal_param.mc << 11) / (x1 + bmp085.cal_param.md);
    bmp085.param_b5 = x1 + x2;
    temperature = ((bmp085.param_b5 * 10 + 8) >> 4);  // temperature in 0.01 C (make same as MS5611)

    return temperature;
}

static int32_t bmp085GetPressure(uint32_t up)
{
    int32_t pressure, x1, x2, x3, b3, b6;
    uint32_t b4, b7;

    b6 = bmp085.param_b5 - 4000;
    // *****calculate B3************
    x1 = (b6 * b6) >> 12;
    x1 *= bmp085.cal_param.b2;
    x1 >>= 11;

    x2 = (bmp085.cal_param.ac2 * b6);
    x2 >>= 11;

    x3 = x1 + x2;

    b3 = (((((int32_t) bmp085.cal_param.ac1) * 4 + x3) << bmp085.oversampling_setting) + 2) >> 2;

    // *****calculate B4************
    x1 = (bmp085.cal_param.ac3 * b6) >> 13;
    x2 = (bmp085.cal_param.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (bmp085.cal_param.ac4 * (uint32_t)(x3 + 32768)) >> 15;

    b7 = ((uint32_t)(up - b3) * (50000 >> bmp085.oversampling_setting));
    if (b7 < 0x80000000) {
        pressure = (b7 << 1) / b4;
    } else {
        pressure = (b7 / b4) << 1;
    }

    x1 = pressure >> 8;
    x1 *= x1;
    x1 = (x1 * SMD500_PARAM_MG) >> 16;
    x2 = (pressure * SMD500_PARAM_MH) >> 16;
    pressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;   // pressure in Pa

    return pressure;
}

static void bmp085StartUT(baroDev_t *baro)
{
    isConversionComplete = false;

    busWriteRegisterStart(&baro->dev, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);
}

static bool bmp085ReadUT(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // return old baro value if conversion time exceeds datasheet max when EOC is connected
    if (isEOCConnected && !isConversionComplete) {
        return false;
    }

    busReadRegisterBufferStart(&baro->dev, BMP085_ADC_OUT_MSB_REG, sensor_data, BMP085_DATA_TEMP_SIZE);

    return true;
}

static bool bmp085GetUT(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    bmp085_ut = sensor_data[0] << 8 | sensor_data[1];

    return true;
}

static void bmp085StartUP(baroDev_t *baro)
{
    uint8_t ctrl_reg_data;

    ctrl_reg_data = BMP085_P_MEASURE + (bmp085.oversampling_setting << 6);

    isConversionComplete = false;

    busWriteRegisterStart(&baro->dev, BMP085_CTRL_MEAS_REG, ctrl_reg_data);
}

static bool bmp085ReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // return old baro value if conversion time exceeds datasheet max when EOC is connected
    if (isEOCConnected && !isConversionComplete) {
        return false;
    }

    busReadRegisterBufferStart(&baro->dev, BMP085_ADC_OUT_MSB_REG, sensor_data, BMP085_DATA_PRES_SIZE);

    return true;
}

/** read out up for pressure conversion
 depending on the oversampling ratio setting up can be 16 to 19 bit
 \return up parameter that represents the uncompensated pressure value
 */
static bool bmp085GetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    bmp085_up = (uint32_t)(sensor_data[0] << 16 | sensor_data[1] << 8 | sensor_data[2])
            >> (8 - bmp085.oversampling_setting);

    return true;
}

STATIC_UNIT_TESTED void bmp085Calculate(int32_t *pressure, int32_t *temperature)
{
    int32_t temp, press;

    temp = bmp085GetTemperature(bmp085_ut);
    press = bmp085GetPressure(bmp085_up);
    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}

static void bmp085ReadCalibrarionParameters(extDevice_t *dev)
{
    uint8_t data[22];
    busReadRegisterBuffer(dev, BMP085_PROM_START__ADDR, data, BMP085_PROM_DATA__LEN);

    /*parameters AC1-AC6*/
    bmp085.cal_param.ac1 = data[0] << 8 | data[1];
    bmp085.cal_param.ac2 = data[2] << 8 | data[3];
    bmp085.cal_param.ac3 = data[4] << 8 | data[5];
    bmp085.cal_param.ac4 = data[6] << 8 | data[7];
    bmp085.cal_param.ac5 = data[8] << 8 | data[9];
    bmp085.cal_param.ac6 = data[10] << 8 | data[11];

    /*parameters B1,B2*/
    bmp085.cal_param.b1 = data[12] << 8 | data[13];
    bmp085.cal_param.b2 = data[14] << 8 | data[15];

    /*parameters MB,MC,MD*/
    bmp085.cal_param.mb = data[16] << 8 | data[17];
    bmp085.cal_param.mc = data[18] << 8 | data[19];
    bmp085.cal_param.md = data[20] << 8 | data[21];
}

static bool bmp085TestEOCConnected(baroDev_t *baro, const bmp085Config_t *config)
{
    UNUSED(config);

#ifdef USE_EXTI
    if (!bmp085InitDone && eocIO) {
        // EOC should be low at this point. If not, assume EOC is not working
        if (IORead(eocIO)) {
            return false;
        }

        bmp085StartUT(baro);
        delayMicroseconds(UT_DELAY * 2); // wait twice as long as normal, just to be sure

        // conversion should have finished now so check if EOC is high
        uint8_t status = IORead(eocIO);
        if (status) {
            return true;
        }
    }
#else
    UNUSED(baro);
#endif

    return false; // assume EOC is not connected
}

#endif /* BARO */
