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

#pragma once

#include "pg/pg.h"
#include "drivers/barometer/barometer.h"

typedef enum {
    BARO_DEFAULT = 0,
    BARO_NONE = 1,
    BARO_BMP085 = 2,
    BARO_MS5611 = 3,
    BARO_BMP280 = 4,
    BARO_LPS = 5,
    BARO_QMP6988 = 6,
    BARO_BMP388 = 7,
    BARO_DPS310 = 8,
} baroSensor_e;

#define BARO_SAMPLE_COUNT_MAX   48

typedef struct barometerConfig_s {
    uint8_t baro_busType;
    uint8_t baro_spi_device;
    ioTag_t baro_spi_csn;                   // Also used as XCLR (positive logic) for BMP085
    uint8_t baro_i2c_device;
    uint8_t baro_i2c_address;
    uint8_t baro_hardware;                  // Barometer hardware to use
    uint8_t baro_sample_count;              // size of baro filter array
    uint16_t baro_noise_lpf;                // additional LPF to reduce baro noise
    uint16_t baro_cf_vel;                   // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
    ioTag_t baro_eoc_tag;
    ioTag_t baro_xclr_tag;
} barometerConfig_t;

PG_DECLARE(barometerConfig_t, barometerConfig);

typedef struct baro_s {
    baroDev_t dev;
    int32_t BaroAlt;
    int32_t baroTemperature;             // Use temperature for telemetry
    int32_t baroPressure;                // Use pressure for telemetry
} baro_t;

extern baro_t baro;

void baroPreInit(void);
bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse);
bool baroIsCalibrationComplete(void);
void baroStartCalibration(void);
void baroSetGroundLevel(void);
uint32_t baroUpdate(void);
bool isBaroReady(void);
int32_t baroCalculateAltitude(void);
void performBaroCalibrationCycle(void);
