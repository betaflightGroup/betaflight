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
#include "build/debug.h"

#include "barometer.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer_bmp280.h"

// 10 MHz max SPI frequency
#define BMP280_MAX_SPI_CLK_HZ 10000000

#if defined(USE_BARO) && (defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280))


#define BMP280_I2C_ADDR                      (0x76)
#define BMP280_DEFAULT_CHIP_ID               (0x58)
#define BME280_DEFAULT_CHIP_ID               (0x60)

#define BMP280_CHIP_ID_REG                   (0xD0)  /* Chip ID Register */
#define BMP280_RST_REG                       (0xE0)  /* Softreset Register */
#define BMP280_STAT_REG                      (0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG                 (0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG                    (0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */
#define BMP280_FORCED_MODE                   (0x01)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE               (6)

#define BMP280_OVERSAMP_SKIPPED          (0x00)
#define BMP280_OVERSAMP_1X               (0x01)
#define BMP280_OVERSAMP_2X               (0x02)
#define BMP280_OVERSAMP_4X               (0x03)
#define BMP280_OVERSAMP_8X               (0x04)
#define BMP280_OVERSAMP_16X              (0x05)

// configure pressure and temperature oversampling, forced sampling mode
#define BMP280_PRESSURE_OSR              (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR           (BMP280_OVERSAMP_1X)
#define BMP280_MODE                      (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_FORCED_MODE)

#define T_INIT_MAX                       (20)
// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)
// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)
// 10/16 = 0.625 ms

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

STATIC_ASSERT(sizeof(bmp280_calib_param_t) == BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH, bmp280_calibration_structure_incorrectly_packed);

STATIC_UNIT_TESTED int32_t t_fine; /* calibration t_fine data */

static uint8_t bmp280_chip_id = 0;
STATIC_UNIT_TESTED bmp280_calib_param_t bmp280_cal;
// uncompensated pressure and temperature
int32_t bmp280_up = 0;
int32_t bmp280_ut = 0;
static DMA_DATA_ZERO_INIT uint8_t sensor_data[BMP280_DATA_FRAME_SIZE];

static void bmp280StartUT(baroDev_t *baro);
static bool bmp280ReadUT(baroDev_t *baro);
static bool bmp280GetUT(baroDev_t *baro);
static void bmp280StartUP(baroDev_t *baro);
static bool bmp280ReadUP(baroDev_t *baro);
static bool bmp280GetUP(baroDev_t *baro);

STATIC_UNIT_TESTED void bmp280Calculate(int32_t *pressure, int32_t *temperature);

void bmp280BusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_BMP280
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(BMP280_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

void bmp280BusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_BMP280
    if (dev->bus->busType == BUS_TYPE_SPI) {
        spiPreinitByIO(dev->busType_u.spi.csnPin);
    }
#else
    UNUSED(dev);
#endif
}

#include "drivers/time.h"

bool bmp280Detect(baroDev_t *baro)
{
    delay(20);

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    bmp280BusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for BMP280
        dev->busType_u.i2c.address = BMP280_I2C_ADDR;
        defaultAddressApplied = true;
    }

    busReadRegisterBuffer(dev, BMP280_CHIP_ID_REG, &bmp280_chip_id, 1);  /* read Chip Id */

    if ((bmp280_chip_id != BMP280_DEFAULT_CHIP_ID) && (bmp280_chip_id != BME280_DEFAULT_CHIP_ID)) {
        bmp280BusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // read calibration
    busReadRegisterBuffer(dev, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, (uint8_t *)&bmp280_cal, sizeof(bmp280_calib_param_t));

    // set oversampling + power mode (forced), and start sampling
    busWriteRegister(dev, BMP280_CTRL_MEAS_REG, BMP280_MODE);

    // these are dummy as temperature is measured as part of pressure
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = bmp280StartUT;
    baro->get_ut = bmp280GetUT;
    baro->read_ut = bmp280ReadUT;
    // only _up part is executed, and gets both temperature and pressure
    baro->start_up = bmp280StartUP;
    baro->get_up = bmp280GetUP;
    baro->read_up = bmp280ReadUP;
    baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << BMP280_TEMPERATURE_OSR) >> 1) + ((1 << BMP280_PRESSURE_OSR) >> 1)) + (BMP280_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;
    baro->calculate = bmp280Calculate;

    return true;
}

static void bmp280StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
}

static bool bmp280ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
    return true;
}

static bool bmp280GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
    return true;
}

static void bmp280StartUP(baroDev_t *baro)
{
    // start measurement
    // set oversampling + power mode (forced), and start sampling
    busWriteRegisterStart(&baro->dev, BMP280_CTRL_MEAS_REG, BMP280_MODE);
}

static bool bmp280ReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // read data from sensor
    busReadRegisterBufferStart(&baro->dev, BMP280_PRESSURE_MSB_REG, sensor_data, BMP280_DATA_FRAME_SIZE);

    return true;
}

static bool bmp280GetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    bmp280_up = (int32_t)(sensor_data[0] << 12 | sensor_data[1] << 4 | sensor_data[2] >> 4);
    bmp280_ut = (int32_t)(sensor_data[3] << 12 | sensor_data[4] << 4 | sensor_data[5] >> 4);

    return true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t bmp280CompensateTemperature(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bmp280CompensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal.dig_P7) << 4);
    return (uint32_t)p;
}

STATIC_UNIT_TESTED void bmp280Calculate(int32_t *pressure, int32_t *temperature)
{
    // calculate
    int32_t t;
    uint32_t p;
    t = bmp280CompensateTemperature(bmp280_ut);
    p = bmp280CompensatePressure(bmp280_up);

    if (pressure)
        *pressure = (int32_t)(p / 256);
    if (temperature)
        *temperature = t;
}

#endif
