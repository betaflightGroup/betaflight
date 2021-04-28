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

#include "common/time.h"
#include "common/unit.h"

#include "drivers/display.h"

#include "pg/pg.h"

#include "sensors/esc_sensor.h"

#define OSD_NUM_TIMER_TYPES 4
extern const char * const osdTimerSourceNames[OSD_NUM_TIMER_TYPES];

#define OSD_ELEMENT_BUFFER_LENGTH 32

#define OSD_PROFILE_NAME_LENGTH 16

#ifdef USE_OSD_PROFILES
#define OSD_PROFILE_COUNT 3
#else
#define OSD_PROFILE_COUNT 1
#endif

#define OSD_RCCHANNELS_COUNT 4

#define OSD_CAMERA_FRAME_MIN_WIDTH  2
#define OSD_CAMERA_FRAME_MAX_WIDTH  30    // Characters per row supportes by MAX7456
#define OSD_CAMERA_FRAME_MIN_HEIGHT 2
#define OSD_CAMERA_FRAME_MAX_HEIGHT 16    // Rows supported by MAX7456 (PAL)

#define OSD_TASK_FREQUENCY_MIN 30
#define OSD_TASK_FREQUENCY_MAX 300
#define OSD_TASK_FREQUENCY_DEFAULT 60

#define OSD_PROFILE_BITS_POS 11
#define OSD_PROFILE_MASK    (((1 << OSD_PROFILE_COUNT) - 1) << OSD_PROFILE_BITS_POS)
#define OSD_POS_MAX   0x3FF
#define OSD_POSCFG_MAX UINT16_MAX  // element positions now use all 16 bits
#define OSD_PROFILE_FLAG(x)  (1 << ((x) - 1 + OSD_PROFILE_BITS_POS))
#define OSD_PROFILE_1_FLAG  OSD_PROFILE_FLAG(1)


#ifdef USE_OSD_PROFILES
#define VISIBLE(x) osdElementVisible(x)
#define VISIBLE_IN_OSD_PROFILE(item, profile)    ((item) & ((OSD_PROFILE_1_FLAG) << ((profile)-1)))
#else
#define VISIBLE(x) ((x) & OSD_PROFILE_MASK)
#define VISIBLE_IN_OSD_PROFILE(item, profile) VISIBLE(item)
#endif


// Character coordinate
#define OSD_POSITION_BITS 5 // 5 bits gives a range 0-31
#define OSD_POSITION_XY_MASK ((1 << OSD_POSITION_BITS) - 1)
#define OSD_TYPE_MASK 0xC000   // bits 14-15
#define OSD_POS(x,y)  ((x & OSD_POSITION_XY_MASK) | ((y & OSD_POSITION_XY_MASK) << OSD_POSITION_BITS))
#define OSD_X(x)      (x & OSD_POSITION_XY_MASK)
#define OSD_Y(x)      ((x >> OSD_POSITION_BITS) & OSD_POSITION_XY_MASK)
#define OSD_TYPE(x)   ((x & OSD_TYPE_MASK) >> 14)

// Timer configuration
// Stored as 15[alarm:8][precision:4][source:4]0
#define OSD_TIMER(src, prec, alarm) ((src & 0x0F) | ((prec & 0x0F) << 4) | ((alarm & 0xFF ) << 8))
#define OSD_TIMER_SRC(timer)        (timer & 0x0F)
#define OSD_TIMER_PRECISION(timer)  ((timer >> 4) & 0x0F)
#define OSD_TIMER_ALARM(timer)      ((timer >> 8) & 0xFF)

#ifdef USE_MAX7456
#define OSD_DRAW_FREQ_DENOM 5
#else
// MWOSD @ 115200 baud
#define OSD_DRAW_FREQ_DENOM 10
#endif

// NB: to ensure backwards compatibility, new enum values must be appended at the end but before the OSD_XXXX_COUNT entry.

// *** IMPORTANT ***
// See the information at the top of osd/osd_elements.c for instructions
// on how to add OSD elements.
typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PIDRATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_GPS_LON,
    OSD_GPS_LAT,
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_HOME_DIR,
    OSD_HOME_DIST,
    OSD_NUMERICAL_HEADING,
    OSD_NUMERICAL_VARIO,
    OSD_COMPASS_BAR,
    OSD_ESC_TMP,
    OSD_ESC_RPM,
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_RTC_DATETIME,
    OSD_ADJUSTMENT_RANGE,
    OSD_CORE_TEMPERATURE,
    OSD_ANTI_GRAVITY,
    OSD_G_FORCE,
    OSD_MOTOR_DIAG,
    OSD_LOG_STATUS,
    OSD_FLIP_ARROW,
    OSD_LINK_QUALITY,
    OSD_FLIGHT_DIST,
    OSD_STICK_OVERLAY_LEFT,
    OSD_STICK_OVERLAY_RIGHT,
    OSD_DISPLAY_NAME,
    OSD_ESC_RPM_FREQ,
    OSD_RATE_PROFILE_NAME,
    OSD_PID_PROFILE_NAME,
    OSD_PROFILE_NAME,
    OSD_RSSI_DBM_VALUE,
    OSD_RC_CHANNELS,
    OSD_CAMERA_FRAME,
    OSD_EFFICIENCY,
    OSD_TOTAL_FLIGHTS,
    OSD_UP_DOWN_REFERENCE,
    OSD_TX_UPLINK_POWER,
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

// *** IMPORTANT ***
// Whenever new elements are added to 'osd_items_e', make sure to increment
// the parameter group version for 'osdConfig' in 'osd.c'

// *** IMPORTANT ***
// DO NOT REORDER THE STATS ENUMERATION. The order here cooresponds to the enabled flag bit position
// storage and changing the order will corrupt user settings. Any new stats MUST be added to the end
// just before the OSD_STAT_COUNT entry. YOU MUST ALSO add the new stat to the
// osdStatsDisplayOrder array in osd.c.
//
// IF YOU WANT TO REORDER THE STATS DISPLAY, then adjust the ordering of the osdStatsDisplayOrder array
typedef enum {
    OSD_STAT_RTC_DATE_TIME,
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_END_BATTERY,
    OSD_STAT_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_USED_MAH,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_BLACKBOX,
    OSD_STAT_BLACKBOX_NUMBER,
    OSD_STAT_MAX_G_FORCE,
    OSD_STAT_MAX_ESC_TEMP,
    OSD_STAT_MAX_ESC_RPM,
    OSD_STAT_MIN_LINK_QUALITY,
    OSD_STAT_FLIGHT_DISTANCE,
    OSD_STAT_MAX_FFT,
    OSD_STAT_TOTAL_FLIGHTS,
    OSD_STAT_TOTAL_TIME,
    OSD_STAT_TOTAL_DIST,
    OSD_STAT_MIN_RSSI_DBM,
    OSD_STAT_COUNT // MUST BE LAST
} osd_stats_e;

// Make sure the number of stats do not exceed the available 32bit storage
STATIC_ASSERT(OSD_STAT_COUNT <= 32, osdstats_overflow);

typedef enum {
    OSD_TIMER_1,
    OSD_TIMER_2,
    OSD_TIMER_COUNT
} osd_timer_e;

typedef enum {
    OSD_TIMER_SRC_ON,
    OSD_TIMER_SRC_TOTAL_ARMED,
    OSD_TIMER_SRC_LAST_ARMED,
    OSD_TIMER_SRC_ON_OR_ARMED,
    OSD_TIMER_SRC_COUNT
} osd_timer_source_e;

typedef enum {
    OSD_TIMER_PREC_SECOND,
    OSD_TIMER_PREC_HUNDREDTHS,
    OSD_TIMER_PREC_TENTHS,
    OSD_TIMER_PREC_COUNT
} osd_timer_precision_e;

typedef enum {
    OSD_WARNING_ARMING_DISABLE,
    OSD_WARNING_BATTERY_NOT_FULL,
    OSD_WARNING_BATTERY_WARNING,
    OSD_WARNING_BATTERY_CRITICAL,
    OSD_WARNING_VISUAL_BEEPER,
    OSD_WARNING_CRASH_FLIP,
    OSD_WARNING_ESC_FAIL,
    OSD_WARNING_CORE_TEMPERATURE,
    OSD_WARNING_RC_SMOOTHING,
    OSD_WARNING_FAIL_SAFE,
    OSD_WARNING_LAUNCH_CONTROL,
    OSD_WARNING_GPS_RESCUE_UNAVAILABLE,
    OSD_WARNING_GPS_RESCUE_DISABLED,
    OSD_WARNING_RSSI,
    OSD_WARNING_LINK_QUALITY,
    OSD_WARNING_RSSI_DBM,
    OSD_WARNING_OVER_CAP,
    OSD_WARNING_COUNT // MUST BE LAST
} osdWarningsFlags_e;

typedef enum {
    OSD_DISPLAYPORT_DEVICE_NONE = 0,
    OSD_DISPLAYPORT_DEVICE_AUTO,
    OSD_DISPLAYPORT_DEVICE_MAX7456,
    OSD_DISPLAYPORT_DEVICE_MSP,
    OSD_DISPLAYPORT_DEVICE_FRSKYOSD,
} osdDisplayPortDevice_e;

// Make sure the number of warnings do not exceed the available 32bit storage
STATIC_ASSERT(OSD_WARNING_COUNT <= 32, osdwarnings_overflow);

#define ESC_RPM_ALARM_OFF -1
#define ESC_TEMP_ALARM_OFF INT8_MIN
#define ESC_CURRENT_ALARM_OFF -1

#define OSD_GPS_RESCUE_DISABLED_WARNING_DURATION_US 3000000 // 3 seconds

extern const uint16_t osdTimerDefault[OSD_TIMER_COUNT];
extern const osd_stats_e osdStatsDisplayOrder[OSD_STAT_COUNT];

typedef struct osdConfig_s {
    // Alarms
    uint16_t cap_alarm;
    uint16_t alt_alarm;
    uint8_t rssi_alarm;

    uint8_t units;

    uint16_t timers[OSD_TIMER_COUNT];
    uint32_t enabledWarnings;

    uint8_t ahMaxPitch;
    uint8_t ahMaxRoll;
    uint32_t enabled_stats;
    int8_t esc_temp_alarm;
    int16_t esc_rpm_alarm;
    int16_t esc_current_alarm;
    uint8_t core_temp_alarm;
    uint8_t ahInvert;                         // invert the artificial horizon
    uint8_t osdProfileIndex;
    uint8_t overlay_radio_mode;
    char profile[OSD_PROFILE_COUNT][OSD_PROFILE_NAME_LENGTH + 1];
    uint16_t link_quality_alarm;
    int16_t rssi_dbm_alarm;
    uint8_t gps_sats_show_hdop;
    int8_t rcChannels[OSD_RCCHANNELS_COUNT];  // RC channel values to display, -1 if none
    uint8_t displayPortDevice;                // osdDisplayPortDevice_e
    uint16_t distance_alarm;
    uint8_t logo_on_arming;                   // show the logo on arming
    uint8_t logo_on_arming_duration;          // display duration in 0.1s units
    uint8_t camera_frame_width;               // The width of the box for the camera frame element
    uint8_t camera_frame_height;              // The height of the box for the camera frame element
    uint16_t task_frequency;
    uint8_t cms_background_type;              // For supporting devices, determines whether the CMS background is transparent or opaque
    uint8_t stat_show_cell_value;
} osdConfig_t;

PG_DECLARE(osdConfig_t, osdConfig);

typedef struct osdElementConfig_s {
    uint16_t item_pos[OSD_ITEM_COUNT];
} osdElementConfig_t;

PG_DECLARE(osdElementConfig_t, osdElementConfig);

typedef struct statistic_s {
    timeUs_t armed_time;
    int16_t max_speed;
    int16_t min_voltage; // /100
    uint16_t end_voltage;
    int16_t max_current; // /10
    uint8_t min_rssi;
    int32_t max_altitude;
    int16_t max_distance;
    float max_g_force;
    int16_t max_esc_temp;
    int32_t max_esc_rpm;
    uint16_t min_link_quality;
    int16_t min_rssi_dbm;
} statistic_t;

extern timeUs_t resumeRefreshAt;
extern timeUs_t osdFlyTime;
#if defined(USE_ACC)
extern float osdGForce;
#endif
#ifdef USE_ESC_SENSOR
extern escSensorData_t *osdEscDataCombined;
#endif

void osdInit(displayPort_t *osdDisplayPort, osdDisplayPortDevice_e displayPortDevice);
void osdUpdate(timeUs_t currentTimeUs);

void osdStatSetState(uint8_t statIndex, bool enabled);
bool osdStatGetState(uint8_t statIndex);
void osdSuppressStats(bool flag);
void osdAnalyzeActiveElements(void);
void changeOsdProfileIndex(uint8_t profileIndex);
uint8_t getCurrentOsdProfileIndex(void);
displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *displayPortDevice);

void osdWarnSetState(uint8_t warningIndex, bool enabled);
bool osdWarnGetState(uint8_t warningIndex);
bool osdElementVisible(uint16_t value);
bool osdGetVisualBeeperState(void);
statistic_t *osdGetStats(void);
bool osdNeedsAccelerometer(void);
int osdPrintFloat(char *buffer, char leadingSymbol, float value, char *formatString, unsigned decimalPlaces, bool round, char trailingSymbol);
