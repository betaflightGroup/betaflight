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

#include <stdint.h>

#include "platform.h"

#include "debug.h"

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

#ifdef DEBUG_SECTION_TIMES
uint32_t sectionTimes[2][4];
#endif

// Please ensure that these names are aligned with the enum values defined in 'debug.h' - for OSD be sure the name is unique with 12 chars.
const char * const debugModeNames[DEBUG_COUNT] = {
    "NONE",
    "CYCLETIME",
    "BATTERY",
    "GYRO_FILTERED",
    "ACCELEROMETER",
    "PIDLOOP",
    "GYRO_SCALED",
    "RC_INTERPOLATION",
    "ANGLERATE",
    "ESC_SENSOR",
    "SCHEDULER",
    "STACK",
    "ESC_SENSOR_RPM",
    "ESC_SENSOR_TMP",
    "ALTITUDE",
    "FFT",
    "FFT_TIME",
    "FFT_FREQ",
    "RX_FRSKY_SPI",
    "RX_SFHSS_SPI",
    "GYRO_RAW",
    "DUAL_GYRO_RAW",
    "DUAL_GYRO_DIFF",
    "MAX7456_SIGNAL",
    "MAX7456_SPICLOCK",
    "SBUS",
    "FPORT",
    "RANGEFINDER",
    "RANGEFINDER_QUALITY",
    "LIDAR_TF",
    "ADC_INTERNAL",
    "RUNAWAY_TAKEOFF",
    "SDIO",
    "CURRENT_SENSOR",
    "USB",
    "SMARTAUDIO",
    "RTH",
    "ITERM_RELAX",
    "ACRO_TRAINER",
    "RC_SMOOTHING",
    "RX_SIGNAL_LOSS",
    "RC_SMOOTHING_RATE",
    "ANTI_GRAVITY",
    "DYN_LPF",
    "RX_SPEKTRUM_SPI",
    "DSHOT_RPM_TELEMETRY",
    "RPM_FILTER",
    "D_MIN",
    "AC_CORRECTION",
    "AC_ERROR",
    "DUAL_GYRO_SCALED",
    "DSHOT_RPM_ERRORS",
    "CRSF_LINK_STATISTICS_UPLINK",
    "CRSF_LINK_STATISTICS_PWR",
    "CRSF_LINK_STATISTICS_DOWN",
    "BARO",
    "GPS_RESCUE_THROTTLE_PID",
    "DYN_IDLE",
    "FF_LIMIT",
    "FF_INTERPOLATED",
    "BLACKBOX_OUTPUT",
    "GYRO_SAMPLE",
    "RX_TIMING",
    "D_LPF",
    "VTX_TRAMP",
    "GHST",
    "RX_EXPRESSLRS_SPI",
};
