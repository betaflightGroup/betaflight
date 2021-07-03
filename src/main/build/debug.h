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

#define DEBUG16_VALUE_COUNT 4
extern int16_t debug[DEBUG16_VALUE_COUNT];
extern uint8_t debugMode;

#define DEBUG_SET(mode, index, value) {if (debugMode == (mode)) {debug[(index)] = (value);}}

#define DEBUG_SECTION_TIMES

#ifdef DEBUG_SECTION_TIMES
extern uint32_t sectionTimes[2][4];

#define TIME_SECTION_BEGIN(index) { \
    extern uint32_t sectionTimes[2][4]; \
    sectionTimes[0][index] = micros(); \
}

#define TIME_SECTION_END(index) { \
    extern uint32_t sectionTimes[2][4]; \
    sectionTimes[1][index] = micros(); \
    debug[index] = sectionTimes[1][index] - sectionTimes[0][index]; \
}
#else

#define TIME_SECTION_BEGIN(index) {}
#define TIME_SECTION_END(index) {}

#endif

typedef enum {
    DEBUG_NONE,
    DEBUG_CYCLETIME,
    DEBUG_BATTERY,
    DEBUG_GYRO_FILTERED,
    DEBUG_ACCELEROMETER,
    DEBUG_PIDLOOP,
    DEBUG_GYRO_SCALED,
    DEBUG_RC_INTERPOLATION,
    DEBUG_ANGLERATE,
    DEBUG_ESC_SENSOR,
    DEBUG_SCHEDULER,
    DEBUG_STACK,
    DEBUG_ESC_SENSOR_RPM,
    DEBUG_ESC_SENSOR_TMP,
    DEBUG_ALTITUDE,
    DEBUG_FFT,
    DEBUG_FFT_TIME,
    DEBUG_FFT_FREQ,
    DEBUG_RX_FRSKY_SPI,
    DEBUG_RX_SFHSS_SPI,
    DEBUG_GYRO_RAW,
    DEBUG_DUAL_GYRO_RAW,
    DEBUG_DUAL_GYRO_DIFF,
    DEBUG_MAX7456_SIGNAL,
    DEBUG_MAX7456_SPICLOCK,
    DEBUG_SBUS,
    DEBUG_FPORT,
    DEBUG_RANGEFINDER,
    DEBUG_RANGEFINDER_QUALITY,
    DEBUG_LIDAR_TF,
    DEBUG_ADC_INTERNAL,
    DEBUG_RUNAWAY_TAKEOFF,
    DEBUG_SDIO,
    DEBUG_CURRENT_SENSOR,
    DEBUG_USB,
    DEBUG_SMARTAUDIO,
    DEBUG_RTH,
    DEBUG_ITERM_RELAX,
    DEBUG_ACRO_TRAINER,
    DEBUG_RC_SMOOTHING,
    DEBUG_RX_SIGNAL_LOSS,
    DEBUG_RC_SMOOTHING_RATE,
    DEBUG_ANTI_GRAVITY,
    DEBUG_DYN_LPF,
    DEBUG_RX_SPEKTRUM_SPI,
    DEBUG_DSHOT_RPM_TELEMETRY,
    DEBUG_RPM_FILTER,
    DEBUG_D_MIN,
    DEBUG_AC_CORRECTION,
    DEBUG_AC_ERROR,
    DEBUG_DUAL_GYRO_SCALED,
    DEBUG_DSHOT_RPM_ERRORS,
    DEBUG_CRSF_LINK_STATISTICS_UPLINK,
    DEBUG_CRSF_LINK_STATISTICS_PWR,
    DEBUG_CRSF_LINK_STATISTICS_DOWN,
    DEBUG_BARO,
    DEBUG_GPS_RESCUE_THROTTLE_PID,
    DEBUG_DYN_IDLE,
    DEBUG_FEEDFORWARD_LIMIT,
    DEBUG_FEEDFORWARD,
    DEBUG_BLACKBOX_OUTPUT,
    DEBUG_GYRO_SAMPLE,
    DEBUG_RX_TIMING,
    DEBUG_D_LPF,
    DEBUG_VTX_TRAMP,
    DEBUG_GHST,
    DEBUG_COUNT
} debugType_e;

extern const char * const debugModeNames[DEBUG_COUNT];
