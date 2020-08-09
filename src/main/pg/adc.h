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

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/adc.h"
#include "drivers/io_types.h"
#include "drivers/dma_reqmap.h"

typedef struct adcChannelConfig_t {
    bool enabled;
    ioTag_t ioTag;
#if defined(STM32H7)
    int8_t device; // ADCDevice
#endif
} adcChannelConfig_t;

typedef struct adcConfig_s {
    adcChannelConfig_t vbat;
    adcChannelConfig_t rssi;
    adcChannelConfig_t current;
    adcChannelConfig_t external1;
    int8_t device; // ADCDevice

    uint16_t vrefIntCalibration;
    uint16_t tempSensorCalibration1;
    uint16_t tempSensorCalibration2;

    int8_t dmaopt[ADCDEV_COUNT]; // One per ADCDEV_x
} adcConfig_t;

PG_DECLARE(adcConfig_t, adcConfig);
