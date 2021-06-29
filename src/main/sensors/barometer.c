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
#include <math.h>

#include "platform.h"

#ifdef USE_BARO

#include "build/debug.h"

#include "common/maths.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp085.h"
#include "drivers/barometer/barometer_bmp280.h"
#include "drivers/barometer/barometer_bmp388.h"
#include "drivers/barometer/barometer_dps310.h"
#include "drivers/barometer/barometer_qmp6988.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/barometer/barometer_ms5611.h"
#include "drivers/barometer/barometer_lps.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"

#include "scheduler/scheduler.h"

#include "barometer.h"

baro_t baro;                        // barometer access functions

PG_REGISTER_WITH_RESET_FN(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 1);

void pgResetFn_barometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_sample_count = 21;
    barometerConfig->baro_noise_lpf = 600;
    barometerConfig->baro_cf_vel = 985;
    barometerConfig->baro_hardware = BARO_DEFAULT;

    // For backward compatibility; ceate a valid default value for bus parameters
    //
    // 1. If DEFAULT_BARO_xxx is defined, use it.
    // 2. Determine default based on USE_BARO_xxx
    //   a. Precedence is in the order of popularity; BMP388, BMP280, MS5611 then BMP085, then
    //   b. If SPI variant is specified, it is likely onboard, so take it.

#if !(defined(DEFAULT_BARO_SPI_BMP388) || defined(DEFAULT_BARO_BMP388) || defined(DEFAULT_BARO_SPI_BMP280) || defined(DEFAULT_BARO_BMP280) || defined(DEFAULT_BARO_SPI_MS5611) || defined(DEFAULT_BARO_MS5611) || defined(DEFAULT_BARO_BMP085) || defined(DEFAULT_BARO_SPI_LPS) || defined(DEFAULT_BARO_SPI_QMP6988) || defined(DEFAULT_BARO_QMP6988)) || defined(DEFAULT_BARO_DPS310) || defined(DEFAULT_BARO_SPI_DPS310)

#if defined(USE_BARO_DPS310) || defined(USE_BARO_SPI_DPS310)
#if defined(USE_BARO_SPI_DPS310)
#define DEFAULT_BARO_SPI_DPS310
#else
#define DEFAULT_BARO_DPS310
#endif
#elif defined(USE_BARO_BMP388) || defined(USE_BARO_SPI_BMP388)
#if defined(USE_BARO_SPI_BMP388)
#define DEFAULT_BARO_SPI_BMP388
#else
#define DEFAULT_BARO_BMP388
#endif
#elif defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
#if defined(USE_BARO_SPI_BMP280)
#define DEFAULT_BARO_SPI_BMP280
#else
#define DEFAULT_BARO_BMP280
#endif
#elif defined(USE_BARO_MS5611) || defined(USE_BARO_SPI_MS5611)
#if defined(USE_BARO_SPI_MS5611)
#define DEFAULT_BARO_SPI_MS5611
#else
#define DEFAULT_BARO_MS5611
#endif
#elif defined(USE_BARO_QMP6988) || defined(USE_BARO_SPI_QMP6988)
#if defined(USE_BARO_SPI_QMP6988)
#define DEFAULT_BARO_SPI_QMP6988
#else
#define DEFAULT_BARO_QMP6988
#endif
#elif defined(USE_BARO_SPI_LPS)
#define DEFAULT_BARO_SPI_LPS
#elif defined(DEFAULT_BARO_BMP085)
#define DEFAULT_BARO_BMP085
#endif
#endif

#if defined(DEFAULT_BARO_SPI_BMP388) || defined(DEFAULT_BARO_SPI_BMP280) || defined(DEFAULT_BARO_SPI_MS5611) || defined(DEFAULT_BARO_SPI_QMP6988) || defined(DEFAULT_BARO_SPI_LPS) || defined(DEFAULT_BARO_SPI_DPS310)
    barometerConfig->baro_busType = BUS_TYPE_SPI;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(spiDeviceByInstance(BARO_SPI_INSTANCE));
    barometerConfig->baro_spi_csn = IO_TAG(BARO_CS_PIN);
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
#elif defined(DEFAULT_BARO_MS5611) || defined(DEFAULT_BARO_BMP388) || defined(DEFAULT_BARO_BMP280) || defined(DEFAULT_BARO_BMP085) ||defined(DEFAULT_BARO_QMP6988) || defined(DEFAULT_BARO_DPS310)
    // All I2C devices shares a default config with address = 0 (per device default)
    barometerConfig->baro_busType = BUS_TYPE_I2C;
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(BARO_I2C_INSTANCE);
    barometerConfig->baro_i2c_address = 0;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    barometerConfig->baro_spi_csn = IO_TAG_NONE;
#else
    barometerConfig->baro_hardware = BARO_NONE;
    barometerConfig->baro_busType = BUS_TYPE_NONE;
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    barometerConfig->baro_spi_csn = IO_TAG_NONE;
#endif

    barometerConfig->baro_eoc_tag = IO_TAG(BARO_EOC_PIN);
    barometerConfig->baro_xclr_tag = IO_TAG(BARO_XCLR_PIN);
}

static uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
static int32_t baroPressure = 0;
static int32_t baroTemperature = 0;

static int32_t baroGroundAltitude = 0;
static int32_t baroGroundPressure = 8*101325;
static uint32_t baroPressureSum = 0;

#define CALIBRATING_BARO_CYCLES 200 // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
#define SET_GROUND_LEVEL_BARO_CYCLES 10 // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)

static bool baroReady = false;

void baroPreInit(void)
{
#ifdef USE_SPI
    if (barometerConfig()->baro_busType == BUS_TYPE_SPI) {
        spiPreinitRegister(barometerConfig()->baro_spi_csn, IOCFG_IPU, 1);
    }
#endif
}

bool baroDetect(baroDev_t *baroDev, baroSensor_e baroHardwareToUse)
{
    extDevice_t *dev = &baroDev->dev;

    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = baroHardwareToUse;

#if !defined(USE_BARO_BMP085) && !defined(USE_BARO_MS5611) && !defined(USE_BARO_SPI_MS5611) && !defined(USE_BARO_BMP388) && !defined(USE_BARO_BMP280) && !defined(USE_BARO_SPI_BMP280)&& !defined(USE_BARO_QMP6988) && !defined(USE_BARO_SPI_QMP6988) && !defined(USE_BARO_DPS310) && !defined(USE_BARO_SPI_DPS310)
    UNUSED(dev);
#endif

    switch (barometerConfig()->baro_busType) {
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        i2cBusSetInstance(dev, barometerConfig()->baro_i2c_device);
        dev->busType_u.i2c.address = barometerConfig()->baro_i2c_address;
        break;
#endif

#ifdef USE_SPI
    case BUS_TYPE_SPI:
        {
            if (!spiSetBusInstance(dev, barometerConfig()->baro_spi_device, OWNER_BARO_CS)) {
                return false;
            }

            dev->busType_u.spi.csnPin = IOGetByTag(barometerConfig()->baro_spi_csn);
        }
        break;
#endif

    default:
        return false;
    }

    switch (baroHardware) {
    case BARO_DEFAULT:
        FALLTHROUGH;

    case BARO_BMP085:
#ifdef USE_BARO_BMP085
        {
            static bmp085Config_t defaultBMP085Config;
            defaultBMP085Config.xclrTag = barometerConfig()->baro_xclr_tag;
            defaultBMP085Config.eocTag = barometerConfig()->baro_eoc_tag;

            static const bmp085Config_t *bmp085Config = &defaultBMP085Config;

            if (bmp085Detect(bmp085Config, baroDev)) {
                baroHardware = BARO_BMP085;
                break;
            }
        }
#endif
        FALLTHROUGH;

    case BARO_MS5611:
#if defined(USE_BARO_MS5611) || defined(USE_BARO_SPI_MS5611)
        if (ms5611Detect(baroDev)) {
            baroHardware = BARO_MS5611;
            break;
        }
#endif
        FALLTHROUGH;

    case BARO_LPS:
#if defined(USE_BARO_SPI_LPS)
        if (lpsDetect(baroDev)) {
            baroHardware = BARO_LPS;
            break;
        }
#endif
        FALLTHROUGH;

    case BARO_DPS310:
#if defined(USE_BARO_DPS310) || defined(USE_BARO_SPI_DPS310)
        {
            if (baroDPS310Detect(baroDev)) {
                baroHardware = BARO_DPS310;
                break;
            }
        }
#endif
        FALLTHROUGH;

    case BARO_BMP388:
#if defined(USE_BARO_BMP388) || defined(USE_BARO_SPI_BMP388)
        {
            static bmp388Config_t defaultBMP388Config;

            defaultBMP388Config.eocTag = barometerConfig()->baro_eoc_tag;

            static const bmp388Config_t *bmp388Config = &defaultBMP388Config;

            if (bmp388Detect(bmp388Config, baroDev)) {
                baroHardware = BARO_BMP388;
                break;
            }
        }
#endif
        FALLTHROUGH;

    case BARO_BMP280:
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
        if (bmp280Detect(baroDev)) {
            baroHardware = BARO_BMP280;
            break;
        }
#endif
        FALLTHROUGH;

     case BARO_QMP6988:
#if defined(USE_BARO_QMP6988) || defined(USE_BARO_SPI_QMP6988)
        if (qmp6988Detect(baroDev)) {
            baroHardware = BARO_QMP6988;
            break;
        }
#endif
        FALLTHROUGH;
    case BARO_NONE:
        baroHardware = BARO_NONE;
        break;
    }

    if (baroHardware == BARO_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
    sensorsSet(SENSOR_BARO);
    return true;
}

bool baroIsCalibrationComplete(void)
{
    return calibratingB == 0;
}

static void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingB = calibrationCyclesRequired;
}

void baroStartCalibration(void)
{
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
}

void baroSetGroundLevel(void)
{
    baroSetCalibrationCycles(SET_GROUND_LEVEL_BARO_CYCLES);
}

#define PRESSURE_SAMPLES_MEDIAN 3

static int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;

    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}

static uint32_t recalculateBarometerTotal(uint32_t pressureTotal, int32_t newPressureReading)
{
    static int32_t barometerSamples[BARO_SAMPLE_COUNT_MAX + 1];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // store current pressure in barometerSamples
    if (currentSampleIndex >= barometerConfig()->baro_sample_count) {
        nextSampleIndex = 0;
        baroReady = true;
    } else {
        nextSampleIndex = (currentSampleIndex + 1);
    }
    barometerSamples[currentSampleIndex] = applyBarometerMedianFilter(newPressureReading);

    // recalculate pressure total
    pressureTotal += barometerSamples[currentSampleIndex];
    pressureTotal -= barometerSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return pressureTotal;
}

typedef enum {
    BAROMETER_NEEDS_TEMPERATURE_READ = 0,
    BAROMETER_NEEDS_TEMPERATURE_SAMPLE,
    BAROMETER_NEEDS_PRESSURE_START,
    BAROMETER_NEEDS_PRESSURE_READ,
    BAROMETER_NEEDS_PRESSURE_SAMPLE,
    BAROMETER_NEEDS_TEMPERATURE_START
} barometerState_e;


bool isBaroReady(void) {
    return baroReady;
}

uint32_t baroUpdate(void)
{
    static barometerState_e state = BAROMETER_NEEDS_PRESSURE_START;
    timeUs_t sleepTime = 1000; // Wait 1ms between states

    if (debugMode == DEBUG_BARO) {
        debug[0] = state;
    }

    // Tell the scheduler to ignore how long this task takes unless the pressure is being read
    // as that takes the longest
    if (state != BAROMETER_NEEDS_PRESSURE_READ) {
        ignoreTaskTime();
    }

    switch (state) {
        default:
        case BAROMETER_NEEDS_TEMPERATURE_START:
            baro.dev.start_ut(&baro.dev);
            state = BAROMETER_NEEDS_TEMPERATURE_READ;
            sleepTime = baro.dev.ut_delay;
            break;

        case BAROMETER_NEEDS_TEMPERATURE_READ:
            if (baro.dev.read_ut(&baro.dev)) {
                state = BAROMETER_NEEDS_TEMPERATURE_SAMPLE;
            }
        break;

        case BAROMETER_NEEDS_TEMPERATURE_SAMPLE:
            if (baro.dev.get_ut(&baro.dev)) {
                state = BAROMETER_NEEDS_PRESSURE_START;
            }
        break;

        case BAROMETER_NEEDS_PRESSURE_START:
            baro.dev.start_up(&baro.dev);
            state = BAROMETER_NEEDS_PRESSURE_READ;
            sleepTime = baro.dev.up_delay;
        break;

        case BAROMETER_NEEDS_PRESSURE_READ:
            if (baro.dev.read_up(&baro.dev)) {
                state = BAROMETER_NEEDS_PRESSURE_SAMPLE;
            } else {
                ignoreTaskTime();
            }
        break;

        case BAROMETER_NEEDS_PRESSURE_SAMPLE:
            if (!baro.dev.get_up(&baro.dev)) {
                break;
            }

            baro.dev.calculate(&baroPressure, &baroTemperature);
            baro.baroPressure = baroPressure;
            baro.baroTemperature = baroTemperature;
            baroPressureSum = recalculateBarometerTotal(baroPressureSum, baroPressure);
            if (baro.dev.combined_read) {
                state = BAROMETER_NEEDS_PRESSURE_START;
            } else {
                state = BAROMETER_NEEDS_TEMPERATURE_START;
            }

            if (debugMode == DEBUG_BARO) {
                debug[1] = baroTemperature;
                debug[2] = baroPressure;
                debug[3] = baroPressureSum;
            }

            sleepTime = baro.dev.ut_delay;
            break;
    }

    return sleepTime;
}

static float pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

int32_t baroCalculateAltitude(void)
{
    int32_t BaroAlt_tmp;

    // calculates height from ground via baro readings
    if (baroIsCalibrationComplete()) {
        BaroAlt_tmp = lrintf(pressureToAltitude((float)(baroPressureSum / barometerConfig()->baro_sample_count)));
        BaroAlt_tmp -= baroGroundAltitude;
        baro.BaroAlt = lrintf((float)baro.BaroAlt * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_noise_lpf) + (float)BaroAlt_tmp * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_noise_lpf))); // additional LPF to reduce baro noise
    }
    else {
        baro.BaroAlt = 0;
    }
    return baro.BaroAlt;
}

void performBaroCalibrationCycle(void)
{
    static int32_t savedGroundPressure = 0;

    baroGroundPressure -= baroGroundPressure / 8;
    baroGroundPressure += baroPressureSum / barometerConfig()->baro_sample_count;
    baroGroundAltitude = (1.0f - pow_approx((baroGroundPressure / 8) / 101325.0f, 0.190259f)) * 4433000.0f;

    if (baroGroundPressure == savedGroundPressure) {
        calibratingB = 0;
    } else {
        calibratingB--;
        savedGroundPressure = baroGroundPressure;
    }
}

#endif /* BARO */
