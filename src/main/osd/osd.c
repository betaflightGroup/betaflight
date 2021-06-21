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

/*
 Created by Marcin Baliniak
 some functions based on MinimOSD

 OSD-CMS separation by jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_OSD

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/version.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"
#include "common/unit.h"

#include "config/feature.h"

#include "drivers/display.h"
#include "drivers/dshot.h"
#include "drivers/flash.h"
#include "drivers/osd_symbols.h"
#include "drivers/sdcard.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#if defined(USE_GYRO_DATA_ANALYSE)
#include "flight/gyroanalyse.h"
#endif
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/position.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gps.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/stats.h"

#include "rx/crsf.h"
#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

typedef enum {
    OSD_LOGO_ARMING_OFF,
    OSD_LOGO_ARMING_ON,
    OSD_LOGO_ARMING_FIRST
} osd_logo_on_arming_e;

const char * const osdTimerSourceNames[] = {
    "ON TIME  ",
    "TOTAL ARM",
    "LAST ARM ",
    "ON/ARM   "
};

// Things in both OSD and CMS

#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

timeUs_t osdFlyTime = 0;
#if defined(USE_ACC)
float osdGForce = 0;
#endif

static bool showVisualBeeper = false;

static statistic_t stats;
timeUs_t resumeRefreshAt = 0;
#define REFRESH_1S    1000 * 1000

static uint8_t armState;
#ifdef USE_OSD_PROFILES
static uint8_t osdProfile = 1;
#endif
static displayPort_t *osdDisplayPort;
static osdDisplayPortDevice_e osdDisplayPortDeviceType;
static bool osdIsReady;

static bool suppressStatsDisplay = false;
static uint8_t osdStatsRowCount = 0;

static bool backgroundLayerSupported = false;

#ifdef USE_ESC_SENSOR
escSensorData_t *osdEscDataCombined;
#endif

STATIC_ASSERT(OSD_POS_MAX == OSD_POS(31,31), OSD_POS_MAX_incorrect);

PG_REGISTER_WITH_RESET_FN(osdConfig_t, osdConfig, PG_OSD_CONFIG, 9);

PG_REGISTER_WITH_RESET_FN(osdElementConfig_t, osdElementConfig, PG_OSD_ELEMENT_CONFIG, 0);

// Controls the display order of the OSD post-flight statistics.
// Adjust the ordering here to control how the post-flight stats are presented.
// Every entry in osd_stats_e should be represented. Any that are missing will not
// be shown on the the post-flight statistics page.
// If you reorder the stats it's likely that you'll need to make likewise updates
// to the unit tests.

// If adding new stats, please add to the osdStatsNeedAccelerometer() function
// if the statistic utilizes the accelerometer.
//
const osd_stats_e osdStatsDisplayOrder[OSD_STAT_COUNT] = {
    OSD_STAT_RTC_DATE_TIME,
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_FLIGHT_DISTANCE,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_END_BATTERY,
    OSD_STAT_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_USED_MAH,
    OSD_STAT_BLACKBOX,
    OSD_STAT_BLACKBOX_NUMBER,
    OSD_STAT_MAX_G_FORCE,
    OSD_STAT_MAX_ESC_TEMP,
    OSD_STAT_MAX_ESC_RPM,
    OSD_STAT_MIN_LINK_QUALITY,
    OSD_STAT_MAX_FFT,
    OSD_STAT_MIN_RSSI_DBM,
    OSD_STAT_TOTAL_FLIGHTS,
    OSD_STAT_TOTAL_TIME,
    OSD_STAT_TOTAL_DIST,
};

// Group elements in a number of groups to reduce task scheduling overhead
#define OSD_GROUP_COUNT 20
// Aim to render a group of elements within a target time
#define OSD_ELEMENT_RENDER_TARGET 40
// Allow a margin by which a group render can exceed that of the sum of the elements before declaring insane
// This will most likely be violated by a USB interrupt whilst using the CLI
#define OSD_ELEMENT_RENDER_GROUP_MARGIN 5

// Format a float to the specified number of decimal places with optional rounding.
// OSD symbols can optionally be placed before and after the formatted number (use SYM_NONE for no symbol).
// The formatString can be used for customized formatting of the integer part. Follow the printf style.
// Pass an empty formatString for default.
int osdPrintFloat(char *buffer, char leadingSymbol, float value, char *formatString, unsigned decimalPlaces, bool round, char trailingSymbol)
{
    char mask[7];
    int pos = 0;
    int multiplier = 1;
    for (unsigned i = 0; i < decimalPlaces; i++) {
        multiplier *= 10;
    }

    value *= multiplier;
    const int scaledValueAbs = ABS(round ? lrintf(value) : value);
    const int integerPart = scaledValueAbs / multiplier;
    const int fractionalPart = scaledValueAbs % multiplier;

    if (leadingSymbol != SYM_NONE) {
        buffer[pos++] = leadingSymbol;
    }
    if (value < 0 && (integerPart || fractionalPart)) {
        buffer[pos++] = '-';
    }

    pos += tfp_sprintf(buffer + pos, (strlen(formatString) ? formatString : "%01u"), integerPart);
    if (decimalPlaces) {
        tfp_sprintf((char *)&mask, ".%%0%uu", decimalPlaces); // builds up the format string to be like ".%03u" for decimalPlaces == 3 as an example
        pos += tfp_sprintf(buffer + pos, mask, fractionalPart);
    }

    if (trailingSymbol != SYM_NONE) {
        buffer[pos++] = trailingSymbol;
    }
    buffer[pos] = '\0';

    return pos;
}

void osdStatSetState(uint8_t statIndex, bool enabled)
{
    if (enabled) {
        osdConfigMutable()->enabled_stats |= (1 << statIndex);
    } else {
        osdConfigMutable()->enabled_stats &= ~(1 << statIndex);
    }
}

bool osdStatGetState(uint8_t statIndex)
{
    return osdConfig()->enabled_stats & (1 << statIndex);
}

void osdWarnSetState(uint8_t warningIndex, bool enabled)
{
    if (enabled) {
        osdConfigMutable()->enabledWarnings |= (1 << warningIndex);
    } else {
        osdConfigMutable()->enabledWarnings &= ~(1 << warningIndex);
    }
}

bool osdWarnGetState(uint8_t warningIndex)
{
    return osdConfig()->enabledWarnings & (1 << warningIndex);
}

#ifdef USE_OSD_PROFILES
void setOsdProfile(uint8_t value)
{
    // 1 ->> 001
    // 2 ->> 010
    // 3 ->> 100
    if (value <= OSD_PROFILE_COUNT) {
        if (value == 0) {
            osdProfile = 1;
        } else {
            osdProfile = 1 << (value - 1);
        }
    }
 }

uint8_t getCurrentOsdProfileIndex(void)
{
    return osdConfig()->osdProfileIndex;
}

void changeOsdProfileIndex(uint8_t profileIndex)
{
    if (profileIndex <= OSD_PROFILE_COUNT) {
        osdConfigMutable()->osdProfileIndex = profileIndex;
        setOsdProfile(profileIndex);
        osdAnalyzeActiveElements();
    }
}
#endif

void osdAnalyzeActiveElements(void)
{
    osdAddActiveElements();
    osdDrawActiveElementsBackground(osdDisplayPort);
}

const uint16_t osdTimerDefault[OSD_TIMER_COUNT] = {
        OSD_TIMER(OSD_TIMER_SRC_ON, OSD_TIMER_PREC_SECOND, 10),
        OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_SECOND, 10)
};

void pgResetFn_osdConfig(osdConfig_t *osdConfig)
{
    // Enable the default stats
    osdConfig->enabled_stats = 0; // reset all to off and enable only a few initially
    osdStatSetState(OSD_STAT_MAX_SPEED, true);
    osdStatSetState(OSD_STAT_MIN_BATTERY, true);
    osdStatSetState(OSD_STAT_MIN_RSSI, true);
    osdStatSetState(OSD_STAT_MAX_CURRENT, true);
    osdStatSetState(OSD_STAT_USED_MAH, true);
    osdStatSetState(OSD_STAT_BLACKBOX, true);
    osdStatSetState(OSD_STAT_BLACKBOX_NUMBER, true);
    osdStatSetState(OSD_STAT_TIMER_2, true);

    osdConfig->units = UNIT_METRIC;

    // Enable all warnings by default
    for (int i=0; i < OSD_WARNING_COUNT; i++) {
        osdWarnSetState(i, true);
    }
    // turn off RSSI & Link Quality warnings by default
    osdWarnSetState(OSD_WARNING_RSSI, false);
    osdWarnSetState(OSD_WARNING_LINK_QUALITY, false);
    osdWarnSetState(OSD_WARNING_RSSI_DBM, false);
    // turn off the over mah capacity warning
    osdWarnSetState(OSD_WARNING_OVER_CAP, false);

    osdConfig->timers[OSD_TIMER_1] = osdTimerDefault[OSD_TIMER_1];
    osdConfig->timers[OSD_TIMER_2] = osdTimerDefault[OSD_TIMER_2];

    osdConfig->overlay_radio_mode = 2;

    osdConfig->rssi_alarm = 20;
    osdConfig->link_quality_alarm = 80;
    osdConfig->cap_alarm  = 2200;
    osdConfig->alt_alarm  = 100; // meters or feet depend on configuration
    osdConfig->esc_temp_alarm = ESC_TEMP_ALARM_OFF; // off by default
    osdConfig->esc_rpm_alarm = ESC_RPM_ALARM_OFF; // off by default
    osdConfig->esc_current_alarm = ESC_CURRENT_ALARM_OFF; // off by default
    osdConfig->core_temp_alarm = 70; // a temperature above 70C should produce a warning, lockups have been reported above 80C

    osdConfig->ahMaxPitch = 20; // 20 degrees
    osdConfig->ahMaxRoll = 40; // 40 degrees

    osdConfig->osdProfileIndex = 1;
    osdConfig->ahInvert = false;
    for (int i=0; i < OSD_PROFILE_COUNT; i++) {
        osdConfig->profile[i][0] = '\0';
    }
    osdConfig->rssi_dbm_alarm = -60;
    osdConfig->gps_sats_show_hdop = false;

    for (int i = 0; i < OSD_RCCHANNELS_COUNT; i++) {
        osdConfig->rcChannels[i] = -1;
    }

    osdConfig->displayPortDevice = OSD_DISPLAYPORT_DEVICE_AUTO;

    osdConfig->distance_alarm = 0;
    osdConfig->logo_on_arming = OSD_LOGO_ARMING_OFF;
    osdConfig->logo_on_arming_duration = 5;  // 0.5 seconds

    osdConfig->camera_frame_width = 24;
    osdConfig->camera_frame_height = 11;

    osdConfig->stat_show_cell_value = false;
    osdConfig->task_frequency = OSD_TASK_FREQUENCY_DEFAULT;
    osdConfig->cms_background_type = DISPLAY_BACKGROUND_TRANSPARENT;
}

void pgResetFn_osdElementConfig(osdElementConfig_t *osdElementConfig)
{
    // Position elements near centre of screen and disabled by default
    for (int i = 0; i < OSD_ITEM_COUNT; i++) {
        osdElementConfig->item_pos[i] = OSD_POS(10, 7);
    }

    // Always enable warnings elements by default
    uint16_t profileFlags = 0;
    for (unsigned i = 1; i <= OSD_PROFILE_COUNT; i++) {
        profileFlags |= OSD_PROFILE_FLAG(i);
    }
    osdElementConfig->item_pos[OSD_WARNINGS] = OSD_POS(9, 10) | profileFlags;

    // Default to old fixed positions for these elements
    osdElementConfig->item_pos[OSD_CROSSHAIRS]         = OSD_POS(13, 6);
    osdElementConfig->item_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(14, 2);
    osdElementConfig->item_pos[OSD_HORIZON_SIDEBARS]   = OSD_POS(14, 6);
    osdElementConfig->item_pos[OSD_CAMERA_FRAME]       = OSD_POS(3, 1);
    osdElementConfig->item_pos[OSD_UP_DOWN_REFERENCE]  = OSD_POS(13, 6);
}

static void osdDrawLogo(int x, int y)
{
    // display logo and help
    int fontOffset = 160;
    for (int row = 0; row < 4; row++) {
        for (int column = 0; column < 24; column++) {
            if (fontOffset <= SYM_END_OF_FONT)
                displayWriteChar(osdDisplayPort, x + column, y + row, DISPLAYPORT_ATTR_NONE, fontOffset++);
        }
    }
}

static void osdCompleteInitialization(void)
{
    armState = ARMING_FLAG(ARMED);

    osdResetAlarms();

    backgroundLayerSupported = displayLayerSupported(osdDisplayPort, DISPLAYPORT_LAYER_BACKGROUND);
    displayLayerSelect(osdDisplayPort, DISPLAYPORT_LAYER_FOREGROUND);

    displayBeginTransaction(osdDisplayPort, DISPLAY_TRANSACTION_OPT_RESET_DRAWING);
    displayClearScreen(osdDisplayPort);

    osdDrawLogo(3, 1);

    char string_buffer[30];
    tfp_sprintf(string_buffer, "V%s", FC_VERSION_STRING);
    displayWrite(osdDisplayPort, 20, 6, DISPLAYPORT_ATTR_NONE, string_buffer);
#ifdef USE_CMS
    displayWrite(osdDisplayPort, 7, 8,  DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT1);
    displayWrite(osdDisplayPort, 11, 9, DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT2);
    displayWrite(osdDisplayPort, 11, 10, DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT3);
#endif

#ifdef USE_RTC_TIME
    char dateTimeBuffer[FORMATTED_DATE_TIME_BUFSIZE];
    if (osdFormatRtcDateTime(&dateTimeBuffer[0])) {
        displayWrite(osdDisplayPort, 5, 12, DISPLAYPORT_ATTR_NONE, dateTimeBuffer);
    }
#endif

    resumeRefreshAt = micros() + (4 * REFRESH_1S);
#ifdef USE_OSD_PROFILES
    setOsdProfile(osdConfig()->osdProfileIndex);
#endif

    osdElementsInit(backgroundLayerSupported);
    osdAnalyzeActiveElements();

    osdIsReady = true;
}

void osdInit(displayPort_t *osdDisplayPortToUse, osdDisplayPortDevice_e displayPortDeviceType)
{
    osdDisplayPortDeviceType = displayPortDeviceType;

    if (!osdDisplayPortToUse) {
        return;
    }

    osdDisplayPort = osdDisplayPortToUse;
#ifdef USE_CMS
    cmsDisplayPortRegister(osdDisplayPort);
#endif
}

static void osdResetStats(void)
{
    stats.max_current  = 0;
    stats.max_speed    = 0;
    stats.min_voltage  = 5000;
    stats.end_voltage  = 0;
    stats.min_rssi     = 99; // percent
    stats.max_altitude = 0;
    stats.max_distance = 0;
    stats.armed_time   = 0;
    stats.max_g_force  = 0;
    stats.max_esc_temp = 0;
    stats.max_esc_rpm  = 0;
    stats.min_link_quality = (linkQualitySource == LQ_SOURCE_NONE) ? 99 : 100; // percent
    stats.min_rssi_dbm = CRSF_SNR_MAX;
}

#if defined(USE_ESC_SENSOR) || defined(USE_DSHOT_TELEMETRY)
static int32_t getAverageEscRpm(void)
{
#ifdef USE_DSHOT_TELEMETRY
    if (motorConfig()->dev.useDshotTelemetry) {
        uint32_t rpm = 0;
        for (int i = 0; i < getMotorCount(); i++) {
            rpm += getDshotTelemetry(i);
        }
        rpm = rpm / getMotorCount();
        return rpm * 100 * 2 / motorConfig()->motorPoleCount;
    }
#endif
#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return calcEscRpm(osdEscDataCombined->rpm);
    }
#endif
    return 0;
}
#endif

static uint16_t getStatsVoltage(void)
{
    return osdConfig()->stat_show_cell_value ? getBatteryAverageCellVoltage() : getBatteryVoltage();
}

static void osdUpdateStats(void)
{
    int16_t value = 0;

#ifdef USE_GPS
    if (gpsConfig()->gps_use_3d_speed) {
        value = gpsSol.speed3d;
    } else {
        value = gpsSol.groundSpeed;
    }
    if (stats.max_speed < value) {
        stats.max_speed = value;
    }
#endif

    value = getStatsVoltage();
    if (stats.min_voltage > value) {
        stats.min_voltage = value;
    }

    value = getAmperage() / 100;
    if (stats.max_current < value) {
        stats.max_current = value;
    }

    value = getRssiPercent();
    if (stats.min_rssi > value) {
        stats.min_rssi = value;
    }

    int32_t altitudeCm = getEstimatedAltitudeCm();
    if (stats.max_altitude < altitudeCm) {
        stats.max_altitude = altitudeCm;
    }

#if defined(USE_ACC)
    if (stats.max_g_force < osdGForce) {
        stats.max_g_force = osdGForce;
    }
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    value = rxGetLinkQualityPercent();
    if (stats.min_link_quality > value) {
        stats.min_link_quality = value;
    }
#endif

#ifdef USE_RX_RSSI_DBM
    value = getRssiDbm();
    if (stats.min_rssi_dbm > value) {
        stats.min_rssi_dbm = value;
    }
#endif

#ifdef USE_GPS
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        if (stats.max_distance < GPS_distanceToHome) {
            stats.max_distance = GPS_distanceToHome;
        }
    }
#endif

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        value = osdEscDataCombined->temperature;
        if (stats.max_esc_temp < value) {
            stats.max_esc_temp = value;
        }
    }
#endif

#if defined(USE_ESC_SENSOR) || defined(USE_DSHOT_TELEMETRY)
    int32_t rpm = getAverageEscRpm();
    if (stats.max_esc_rpm < rpm) {
        stats.max_esc_rpm = rpm;
    }
#endif
}

#ifdef USE_BLACKBOX

static void osdGetBlackboxStatusString(char * buff)
{
    bool storageDeviceIsWorking = isBlackboxDeviceWorking();
    uint32_t storageUsed = 0;
    uint32_t storageTotal = 0;

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        if (storageDeviceIsWorking) {
            storageTotal = sdcard_getMetadata()->numBlocks / 2000;
            storageUsed = storageTotal - (afatfs_getContiguousFreeSpace() / 1024000);
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        if (storageDeviceIsWorking) {

            const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);
            const flashGeometry_t *flashGeometry = flashGetGeometry();

            storageTotal = ((FLASH_PARTITION_SECTOR_COUNT(flashPartition) * flashGeometry->sectorSize) / 1024);
            storageUsed = flashfsGetOffset() / 1024;
        }
        break;
#endif

    default:
        break;
    }

    if (storageDeviceIsWorking) {
        const uint16_t storageUsedPercent = (storageUsed * 100) / storageTotal;
        tfp_sprintf(buff, "%d%%", storageUsedPercent);
    } else {
        tfp_sprintf(buff, "FAULT");
    }
}
#endif

static void osdDisplayStatisticLabel(uint8_t y, const char * text, const char * value)
{
    displayWrite(osdDisplayPort, 2, y, DISPLAYPORT_ATTR_NONE, text);
    displayWrite(osdDisplayPort, 20, y, DISPLAYPORT_ATTR_NONE, ":");
    displayWrite(osdDisplayPort, 22, y, DISPLAYPORT_ATTR_NONE, value);
}

/*
 * Test if there's some stat enabled
 */
static bool isSomeStatEnabled(void)
{
    return (osdConfig()->enabled_stats != 0);
}

// *** IMPORTANT ***
// The stats display order was previously required to match the enumeration definition so it matched
// the order shown in the configurator. However, to allow reordering this screen without breaking the
// compatibility, this requirement has been relaxed to a best effort approach. Reordering the elements
// on the stats screen will have to be more beneficial than the hassle of not matching exactly to the
// configurator list.

static bool osdDisplayStat(int statistic, uint8_t displayRow)
{
    char buff[OSD_ELEMENT_BUFFER_LENGTH];

    switch (statistic) {
    case OSD_STAT_RTC_DATE_TIME: {
        bool success = false;
#ifdef USE_RTC_TIME
        success = osdFormatRtcDateTime(&buff[0]);
#endif
        if (!success) {
            tfp_sprintf(buff, "NO RTC");
        }

        displayWrite(osdDisplayPort, 2, displayRow, DISPLAYPORT_ATTR_NONE, buff);
        return true;
    }

    case OSD_STAT_TIMER_1:
        osdFormatTimer(buff, false, (OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1]) == OSD_TIMER_SRC_ON ? false : true), OSD_TIMER_1);
        osdDisplayStatisticLabel(displayRow, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1])], buff);
        return true;

    case OSD_STAT_TIMER_2:
        osdFormatTimer(buff, false, (OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2]) == OSD_TIMER_SRC_ON ? false : true), OSD_TIMER_2);
        osdDisplayStatisticLabel(displayRow, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2])], buff);
        return true;

    case OSD_STAT_MAX_ALTITUDE: {
        osdPrintFloat(buff, SYM_NONE, osdGetMetersToSelectedUnit(stats.max_altitude) / 100.0f, "", 1, true, osdGetMetersToSelectedUnitSymbol());
        osdDisplayStatisticLabel(displayRow, "MAX ALTITUDE", buff);
        return true;
    }

#ifdef USE_GPS
    case OSD_STAT_MAX_SPEED:
        if (featureIsEnabled(FEATURE_GPS)) {
            tfp_sprintf(buff, "%d%c", osdGetSpeedToSelectedUnit(stats.max_speed), osdGetSpeedToSelectedUnitSymbol());
            osdDisplayStatisticLabel(displayRow, "MAX SPEED", buff);
            return true;
        }
        break;

    case OSD_STAT_MAX_DISTANCE:
        if (featureIsEnabled(FEATURE_GPS)) {
            osdFormatDistanceString(buff, stats.max_distance, SYM_NONE);
            osdDisplayStatisticLabel(displayRow, "MAX DISTANCE", buff);
            return true;
        }
        break;

    case OSD_STAT_FLIGHT_DISTANCE:
        if (featureIsEnabled(FEATURE_GPS)) {
            const int distanceFlown = GPS_distanceFlownInCm / 100;
            osdFormatDistanceString(buff, distanceFlown, SYM_NONE);
            osdDisplayStatisticLabel(displayRow, "FLIGHT DISTANCE", buff);
            return true;
        }
        break;
#endif

    case OSD_STAT_MIN_BATTERY:
        osdPrintFloat(buff, SYM_NONE, stats.min_voltage / 100.0f, "", 2, true, SYM_VOLT);
        osdDisplayStatisticLabel(displayRow, osdConfig()->stat_show_cell_value? "MIN AVG CELL" : "MIN BATTERY", buff);
        return true;

    case OSD_STAT_END_BATTERY:
        osdPrintFloat(buff, SYM_NONE, stats.end_voltage / 100.0f, "", 2, true, SYM_VOLT);
        osdDisplayStatisticLabel(displayRow, osdConfig()->stat_show_cell_value ? "END AVG CELL" : "END BATTERY", buff);
        return true;

    case OSD_STAT_BATTERY: 
        {
            const uint16_t statsVoltage = getStatsVoltage();
            osdPrintFloat(buff, SYM_NONE, statsVoltage / 100.0f, "", 2, true, SYM_VOLT);
            osdDisplayStatisticLabel(displayRow, osdConfig()->stat_show_cell_value ? "AVG BATT CELL" : "BATTERY", buff);
            return true;
        }
        break;
        
    case OSD_STAT_MIN_RSSI:
        itoa(stats.min_rssi, buff, 10);
        strcat(buff, "%");
        osdDisplayStatisticLabel(displayRow, "MIN RSSI", buff);
        return true;

    case OSD_STAT_MAX_CURRENT:
        if (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) {
            tfp_sprintf(buff, "%d%c", stats.max_current, SYM_AMP);
            osdDisplayStatisticLabel(displayRow, "MAX CURRENT", buff);
            return true;
        }
        break;

    case OSD_STAT_USED_MAH:
        if (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) {
            tfp_sprintf(buff, "%d%c", getMAhDrawn(), SYM_MAH);
            osdDisplayStatisticLabel(displayRow, "USED MAH", buff);
            return true;
        }
        break;

#ifdef USE_BLACKBOX
    case OSD_STAT_BLACKBOX:
        if (blackboxConfig()->device && blackboxConfig()->device != BLACKBOX_DEVICE_SERIAL) {
            osdGetBlackboxStatusString(buff);
            osdDisplayStatisticLabel(displayRow, "BLACKBOX", buff);
            return true;
        }
        break;

    case OSD_STAT_BLACKBOX_NUMBER:
        {
            int32_t logNumber = blackboxGetLogNumber();
            if (logNumber >= 0) {
                itoa(logNumber, buff, 10);
                osdDisplayStatisticLabel(displayRow, "BB LOG NUM", buff);
                return true;
            }
        }
        break;
#endif

#if defined(USE_ACC)
    case OSD_STAT_MAX_G_FORCE:
        if (sensors(SENSOR_ACC)) {
            osdPrintFloat(buff, SYM_NONE, stats.max_g_force, "", 1, true, 'G');
            osdDisplayStatisticLabel(displayRow, "MAX G-FORCE", buff);
            return true;
        }
        break;
#endif

#ifdef USE_ESC_SENSOR
    case OSD_STAT_MAX_ESC_TEMP:
        tfp_sprintf(buff, "%d%c", osdConvertTemperatureToSelectedUnit(stats.max_esc_temp), osdGetTemperatureSymbolForSelectedUnit());
        osdDisplayStatisticLabel(displayRow, "MAX ESC TEMP", buff);
        return true;
#endif

#if defined(USE_ESC_SENSOR) || defined(USE_DSHOT_TELEMETRY)
    case OSD_STAT_MAX_ESC_RPM:
        itoa(stats.max_esc_rpm, buff, 10);
        osdDisplayStatisticLabel(displayRow, "MAX ESC RPM", buff);
        return true;
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    case OSD_STAT_MIN_LINK_QUALITY:
        tfp_sprintf(buff, "%d", stats.min_link_quality);
        strcat(buff, "%");
        osdDisplayStatisticLabel(displayRow, "MIN LINK", buff);
        return true;
#endif

#if defined(USE_GYRO_DATA_ANALYSE)
    case OSD_STAT_MAX_FFT:
        if (featureIsEnabled(FEATURE_DYNAMIC_FILTER)) {
            int value = getMaxFFT();
            if (value > 0) {
                tfp_sprintf(buff, "%dHZ", value);
                osdDisplayStatisticLabel(displayRow, "PEAK FFT", buff);
            } else {
                osdDisplayStatisticLabel(displayRow, "PEAK FFT", "THRT<20%");
            }
            return true;
        }
        break;
#endif

#ifdef USE_RX_RSSI_DBM
    case OSD_STAT_MIN_RSSI_DBM:
        tfp_sprintf(buff, "%3d", stats.min_rssi_dbm);
        osdDisplayStatisticLabel(displayRow, "MIN RSSI DBM", buff);
        return true;
#endif

#ifdef USE_PERSISTENT_STATS
    case OSD_STAT_TOTAL_FLIGHTS:
        itoa(statsConfig()->stats_total_flights, buff, 10);
        osdDisplayStatisticLabel(displayRow, "TOTAL FLIGHTS", buff);
        return true;

    case OSD_STAT_TOTAL_TIME: {
        int minutes = statsConfig()->stats_total_time_s / 60;
        tfp_sprintf(buff, "%d:%02dH", minutes / 60, minutes % 60);
        osdDisplayStatisticLabel(displayRow, "TOTAL FLIGHT TIME", buff);
        return true;
    }

    case OSD_STAT_TOTAL_DIST:
        #define METERS_PER_KILOMETER 1000
        #define METERS_PER_MILE      1609
        if (osdConfig()->units == UNIT_IMPERIAL) {
            tfp_sprintf(buff, "%d%c", statsConfig()->stats_total_dist_m / METERS_PER_MILE, SYM_MILES);
        } else {
            tfp_sprintf(buff, "%d%c", statsConfig()->stats_total_dist_m / METERS_PER_KILOMETER, SYM_KM);
        }
        osdDisplayStatisticLabel(displayRow, "TOTAL DISTANCE", buff);
        return true;
#endif
    }
    return false;
}

static uint8_t osdShowStats(int statsRowCount)
{
    uint8_t top = 0;
    bool displayLabel = false;

    // if statsRowCount is 0 then we're running an initial analysis of the active stats items
    if (statsRowCount > 0) {
        const int availableRows = osdDisplayPort->rows;
        int displayRows = MIN(statsRowCount, availableRows);
        if (statsRowCount < availableRows) {
            displayLabel = true;
            displayRows++;
        }
        top = (availableRows - displayRows) / 2;  // center the stats vertically
    }

    if (displayLabel) {
        displayWrite(osdDisplayPort, 2, top++, DISPLAYPORT_ATTR_NONE, "  --- STATS ---");
    }

    for (int i = 0; i < OSD_STAT_COUNT; i++) {
        if (osdStatGetState(osdStatsDisplayOrder[i])) {
            if (osdDisplayStat(osdStatsDisplayOrder[i], top)) {
                top++;
            }
        }
    }
    return top;
}

static void osdRefreshStats(void)
{
    displayClearScreen(osdDisplayPort);
    if (osdStatsRowCount == 0) {
        // No stats row count has been set yet.
        // Go through the logic one time to determine how many stats are actually displayed.
        osdStatsRowCount = osdShowStats(0);
        // Then clear the screen and commence with normal stats display which will
        // determine if the heading should be displayed and also center the content vertically.
        displayClearScreen(osdDisplayPort);
    }
    osdShowStats(osdStatsRowCount);
}

static timeDelta_t osdShowArmed(void)
{
    timeDelta_t ret;

    displayClearScreen(osdDisplayPort);

    if ((osdConfig()->logo_on_arming == OSD_LOGO_ARMING_ON) || ((osdConfig()->logo_on_arming == OSD_LOGO_ARMING_FIRST) && !ARMING_FLAG(WAS_EVER_ARMED))) {
        osdDrawLogo(3, 1);
        ret = osdConfig()->logo_on_arming_duration * 1e5;
    } else {
        ret = (REFRESH_1S / 2);
    }
    displayWrite(osdDisplayPort, 12, 7, DISPLAYPORT_ATTR_NONE, "ARMED");

    return ret;
}

static bool osdStatsVisible = false;
static bool osdStatsEnabled = false;

STATIC_UNIT_TESTED void osdDrawStats1(timeUs_t currentTimeUs)
{
    static timeUs_t lastTimeUs = 0;
    static timeUs_t osdStatsRefreshTimeUs;

    // detect arm/disarm
    if (armState != ARMING_FLAG(ARMED)) {
        if (ARMING_FLAG(ARMED)) {
            osdStatsEnabled = false;
            osdStatsVisible = false;
            osdResetStats();
            resumeRefreshAt = osdShowArmed() + currentTimeUs;
        } else if (isSomeStatEnabled()
                   && !suppressStatsDisplay
                   && (!(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))
                       || !VISIBLE(osdElementConfig()->item_pos[OSD_WARNINGS]))) { // suppress stats if runaway takeoff triggered disarm and WARNINGS element is visible
            osdStatsEnabled = true;
            resumeRefreshAt = currentTimeUs + (60 * REFRESH_1S);
            stats.end_voltage = getStatsVoltage();
            osdStatsRowCount = 0; // reset to 0 so it will be recalculated on the next stats refresh
        }

        armState = ARMING_FLAG(ARMED);
    }

    if (ARMING_FLAG(ARMED)) {
        osdUpdateStats();
        timeUs_t deltaT = currentTimeUs - lastTimeUs;
        osdFlyTime += deltaT;
        stats.armed_time += deltaT;
    } else if (osdStatsEnabled) {  // handle showing/hiding stats based on OSD disable switch position
        if (displayIsGrabbed(osdDisplayPort)) {
            osdStatsEnabled = false;
            resumeRefreshAt = 0;
            stats.armed_time = 0;
        } else {
            if (IS_RC_MODE_ACTIVE(BOXOSD) && osdStatsVisible) {
                osdStatsVisible = false;
                displayClearScreen(osdDisplayPort);
            } else if (!IS_RC_MODE_ACTIVE(BOXOSD)) {
                if (!osdStatsVisible) {
                    osdStatsVisible = true;
                    osdStatsRefreshTimeUs = 0;
                }
                if (currentTimeUs >= osdStatsRefreshTimeUs) {
                    osdStatsRefreshTimeUs = currentTimeUs + REFRESH_1S;
                    osdRefreshStats();
                }
            }
        }
    }
    lastTimeUs = currentTimeUs;
}

void osdDrawStats2(timeUs_t currentTimeUs)
{
    displayBeginTransaction(osdDisplayPort, DISPLAY_TRANSACTION_OPT_RESET_DRAWING);

    if (resumeRefreshAt) {
        if (cmp32(currentTimeUs, resumeRefreshAt) < 0) {
            // in timeout period, check sticks for activity to resume display.
            if (IS_HI(THROTTLE) || IS_HI(PITCH)) {
                resumeRefreshAt = currentTimeUs;
            }
            return;
        } else {
            displayClearScreen(osdDisplayPort);
            resumeRefreshAt = 0;
            osdStatsEnabled = false;
            stats.armed_time = 0;
        }
    }
#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        osdEscDataCombined = getEscSensorData(ESC_SENSOR_COMBINED);
    }
#endif
}

void osdDrawStats3()
{
#if defined(USE_ACC)
    if (sensors(SENSOR_ACC)
       && (VISIBLE(osdElementConfig()->item_pos[OSD_G_FORCE]) || osdStatGetState(OSD_STAT_MAX_G_FORCE))) {
            // only calculate the G force if the element is visible or the stat is enabled
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            const float a = accAverage[axis];
            osdGForce += a * a;
        }
        osdGForce = sqrtf(osdGForce) * acc.dev.acc_1G_rec;
    }
#endif
}

typedef enum {
    OSD_STATE_INIT,
    OSD_STATE_IDLE,
    OSD_STATE_CHECK,
    OSD_STATE_UPDATE_STATS1,
    OSD_STATE_UPDATE_STATS2,
    OSD_STATE_UPDATE_STATS3,
    OSD_STATE_UPDATE_ALARMS,
    OSD_STATE_UPDATE_CANVAS,
    OSD_STATE_UPDATE_ELEMENTS,
    OSD_STATE_UPDATE_HEARTBEAT,
    OSD_STATE_COMMIT,
    OSD_STATE_TRANSFER,
    OSD_STATE_COUNT
} osdState_e;

osdState_e osdState = OSD_STATE_INIT;

#define OSD_UPDATE_INTERVAL_US (1000000 / osdConfig()->task_frequency)

// Called periodically by the scheduler
bool osdUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentDeltaTimeUs);
    static timeUs_t osdUpdateDueUs = 0;

    if (osdState == OSD_STATE_IDLE) {
        // If the OSD is due a refresh, mark that as being the case
        if (cmpTimeUs(currentTimeUs, osdUpdateDueUs) > 0) {
            osdState = OSD_STATE_CHECK;

            // Determine time of next update
            if (osdUpdateDueUs) {
                osdUpdateDueUs += OSD_UPDATE_INTERVAL_US;
            } else {
                osdUpdateDueUs = currentTimeUs + OSD_UPDATE_INTERVAL_US;
            }
        }
    }

    return (osdState != OSD_STATE_IDLE);
}

// Called when there is OSD update work to be done
void osdUpdate(timeUs_t currentTimeUs)
{
    static timeUs_t osdStateDurationUs[OSD_STATE_COUNT] = { 0 };
    static timeUs_t osdElementDurationUs[OSD_ITEM_COUNT] = { 0 };
    static timeUs_t osdElementGroupMembership[OSD_ITEM_COUNT];
    static timeUs_t osdElementGroupTargetUs[OSD_GROUP_COUNT] = { 0 };
    static timeUs_t osdElementGroupDurationUs[OSD_GROUP_COUNT] = { 0 };
    static uint8_t osdElementGroup;
    static bool firstPass = true;
    uint8_t osdCurElementGroup = 0;
    timeUs_t executeTimeUs;
    osdState_e osdCurState = osdState;

    if (osdState != OSD_STATE_UPDATE_CANVAS) {
        ignoreTaskExecRate();
    }

    switch (osdState) {
    case OSD_STATE_INIT:
        if (!displayCheckReady(osdDisplayPort, false)) {
            displayRedraw(osdDisplayPort);
            break;
        }

        osdCompleteInitialization();
        displayRedraw(osdDisplayPort);
        osdState = OSD_STATE_COMMIT;

        break;

    case OSD_STATE_CHECK:
        if (isBeeperOn()) {
            showVisualBeeper = true;
        }

        // don't touch buffers if DMA transaction is in progress
        if (displayIsTransferInProgress(osdDisplayPort)) {
            break;
        }

        osdState = OSD_STATE_UPDATE_HEARTBEAT;
        break;

    case OSD_STATE_UPDATE_HEARTBEAT:
        if (displayHeartbeat(osdDisplayPort)) {
            // Extraordinary action was taken, so return without allowing osdStateDurationUs table to be updated
            return;
        }

        osdState = OSD_STATE_UPDATE_STATS1;
        break;

    case OSD_STATE_UPDATE_STATS1:
        osdDrawStats1(currentTimeUs);
        showVisualBeeper = false;

        osdState = OSD_STATE_UPDATE_STATS2;
        break;

    case OSD_STATE_UPDATE_STATS2:
        osdDrawStats2(currentTimeUs);

        osdState = OSD_STATE_UPDATE_STATS3;
        break;

    case OSD_STATE_UPDATE_STATS3:
        osdDrawStats3();

#ifdef USE_CMS
        if (!displayIsGrabbed(osdDisplayPort))
#endif
        {
            osdState = OSD_STATE_UPDATE_ALARMS;
            break;
        }

        osdState = OSD_STATE_COMMIT;
        break;

    case OSD_STATE_UPDATE_ALARMS:
        osdUpdateAlarms();

        if (resumeRefreshAt) {
            osdState = OSD_STATE_IDLE;
        } else {
            osdState = OSD_STATE_UPDATE_CANVAS;
        }
        break;

    case OSD_STATE_UPDATE_CANVAS:
        // Hide OSD when OSDSW mode is active
        if (IS_RC_MODE_ACTIVE(BOXOSD)) {
            displayClearScreen(osdDisplayPort);
            osdState = OSD_STATE_COMMIT;
            break;
        }

        if (backgroundLayerSupported) {
            // Background layer is supported, overlay it onto the foreground
            // so that we only need to draw the active parts of the elements.
            displayLayerCopy(osdDisplayPort, DISPLAYPORT_LAYER_FOREGROUND, DISPLAYPORT_LAYER_BACKGROUND);
        } else {
            // Background layer not supported, just clear the foreground in preparation
            // for drawing the elements including their backgrounds.
            displayClearScreen(osdDisplayPort);
        }

#ifdef USE_GPS
        static bool lastGpsSensorState;
        // Handle the case that the GPS_SENSOR may be delayed in activation
        // or deactivate if communication is lost with the module.
        const bool currentGpsSensorState = sensors(SENSOR_GPS);
        if (lastGpsSensorState != currentGpsSensorState) {
            lastGpsSensorState = currentGpsSensorState;
            osdAnalyzeActiveElements();
        }
#endif // USE_GPS

        osdSyncBlink();

        uint8_t elementGroup;
        uint8_t activeElements = osdGetActiveElementCount();

        // Reset groupings
        for (elementGroup = 0; elementGroup < OSD_GROUP_COUNT; elementGroup++) {
            if (osdElementGroupDurationUs[elementGroup] >  (osdElementGroupTargetUs[elementGroup] + OSD_ELEMENT_RENDER_GROUP_MARGIN)) {
                osdElementGroupDurationUs[elementGroup] = 0;
            }
            osdElementGroupTargetUs[elementGroup] = 0;
        }

        elementGroup = 0;

        // Based on the current element rendering, group to execute in approx 40us
        for (uint8_t curElement = 0; curElement < activeElements; curElement++) {
            if ((osdElementGroupTargetUs[elementGroup] == 0) ||
                ((osdElementGroupTargetUs[elementGroup] + osdElementDurationUs[curElement]) <= OSD_ELEMENT_RENDER_TARGET) ||
                (elementGroup == (OSD_GROUP_COUNT - 1))) {
                osdElementGroupTargetUs[elementGroup] += osdElementDurationUs[curElement];
                // If group membership changes, reset the stats for the group
                if (osdElementGroupMembership[curElement] != elementGroup) {
                    osdElementGroupDurationUs[elementGroup] = 0;
                }
                osdElementGroupMembership[curElement] = elementGroup;
            } else {
                elementGroup++;
                // Try again for this element
                curElement--;
            }
        }

        // Start with group 0
        osdElementGroup = 0;

        if (activeElements > 0) {
            osdState = OSD_STATE_UPDATE_ELEMENTS;
        } else {
            osdState = OSD_STATE_COMMIT;
        }
        break;

    case OSD_STATE_UPDATE_ELEMENTS:
        {
            osdCurElementGroup = osdElementGroup;
            bool moreElements = true;

            do {
                timeUs_t startElementTime = micros();
                uint8_t osdCurElement = osdGetActiveElement();

                // This element should be rendered in the next group
                if (osdElementGroupMembership[osdCurElement] != osdElementGroup) {
                    osdElementGroup++;
                    break;
                }

                moreElements = osdDrawNextActiveElement(osdDisplayPort, currentTimeUs);

                executeTimeUs = micros() - startElementTime;

                if (executeTimeUs > osdElementDurationUs[osdCurElement]) {
                    osdElementDurationUs[osdCurElement] = executeTimeUs;
                }
            } while (moreElements);

            if (moreElements) {
                // There are more elements to draw
                break;
            }

            osdElementGroup = 0;

            osdState = OSD_STATE_COMMIT;
        }
        break;

    case OSD_STATE_COMMIT:
        displayCommitTransaction(osdDisplayPort);

        if (resumeRefreshAt) {
            osdState = OSD_STATE_IDLE;
        } else {
            osdState = OSD_STATE_TRANSFER;
        }
        break;

    case OSD_STATE_TRANSFER:
        // Wait for any current transfer to complete
        if (displayIsTransferInProgress(osdDisplayPort)) {
            break;
        }

        // Transfer may be broken into many parts
        if (displayDrawScreen(osdDisplayPort)) {
            break;
        }

        firstPass = false;
        osdState = OSD_STATE_IDLE;
        break;

    case OSD_STATE_IDLE:
    default:
        osdState = OSD_STATE_IDLE;
        break;
    }

    executeTimeUs = micros() - currentTimeUs;


    // On the first pass no element groups will have been formed, so all elements will have been
    // rendered which is unrepresentative, so ignore
    if (!firstPass) {
        if (osdCurState == OSD_STATE_UPDATE_ELEMENTS) {
            if (executeTimeUs > osdElementGroupDurationUs[osdCurElementGroup]) {
                osdElementGroupDurationUs[osdCurElementGroup] = executeTimeUs;
            }
        }

        if (executeTimeUs > osdStateDurationUs[osdCurState]) {
            osdStateDurationUs[osdCurState] = executeTimeUs;
        }
    }

    if (osdState == OSD_STATE_UPDATE_ELEMENTS) {
        schedulerSetNextStateTime(osdElementGroupDurationUs[osdElementGroup]);
    } else {
        if (osdState == OSD_STATE_IDLE) {
            schedulerSetNextStateTime(osdStateDurationUs[OSD_STATE_CHECK]);
        } else {
            schedulerSetNextStateTime(osdStateDurationUs[osdState]);
        }
    }
}

void osdSuppressStats(bool flag)
{
    suppressStatsDisplay = flag;
}

#ifdef USE_OSD_PROFILES
bool osdElementVisible(uint16_t value)
{
    return (bool)((((value & OSD_PROFILE_MASK) >> OSD_PROFILE_BITS_POS) & osdProfile) != 0);
}
#endif

bool osdGetVisualBeeperState(void)
{
    return showVisualBeeper;
}

statistic_t *osdGetStats(void)
{
    return &stats;
}

#ifdef USE_ACC
// Determine if there are any enabled stats that need
// the ACC (currently only MAX_G_FORCE).
static bool osdStatsNeedAccelerometer(void)
{
    return osdStatGetState(OSD_STAT_MAX_G_FORCE);
}

// Check if any enabled elements or stats need the ACC
bool osdNeedsAccelerometer(void)
{
    return osdStatsNeedAccelerometer() || osdElementsNeedAccelerometer();
}
#endif // USE_ACC

displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *displayPortDeviceType)
{
    if (displayPortDeviceType) {
        *displayPortDeviceType = osdDisplayPortDeviceType;
    }
    return osdDisplayPort;
}

#endif // USE_OSD
