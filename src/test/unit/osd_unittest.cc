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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "blackbox/blackbox.h"
    #include "blackbox/blackbox_io.h"

    #include "common/time.h"

    #include "config/config.h"
    #include "config/feature.h"

    #include "drivers/osd_symbols.h"
    #include "drivers/persistent.h"
    #include "drivers/serial.h"

    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/gps_rescue.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"

    #include "io/beeper.h"
    #include "io/gps.h"

    #include "osd/osd.h"
    #include "osd/osd_elements.h"
    #include "osd/osd_warnings.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "sensors/acceleration.h"
    #include "sensors/battery.h"

    #include "rx/rx.h"

    void osdRefresh(timeUs_t currentTimeUs);
    void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time);
    int osdConvertTemperatureToSelectedUnit(int tempInDegreesCelcius);

    uint16_t rssi;
    attitudeEulerAngles_t attitude;
    float rMat[3][3];

    pidProfile_t *currentPidProfile;
    int16_t debug[DEBUG16_VALUE_COUNT];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint8_t GPS_numSat;
    uint16_t GPS_distanceToHome;
    int16_t GPS_directionToHome;
    uint32_t GPS_distanceFlownInCm;
    int32_t GPS_coord[2];
    gpsSolutionData_t gpsSol;
    float motor[8];

    linkQualitySource_e linkQualitySource;

    acc_t acc;
    float accAverage[XYZ_AXIS_COUNT];

    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);
    PG_REGISTER(imuConfig_t, imuConfig, PG_IMU_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    
    timeUs_t simulationTime = 0;
    batteryState_e simulationBatteryState;
    uint8_t simulationBatteryCellCount;
    uint16_t simulationBatteryVoltage;
    uint32_t simulationBatteryAmperage;
    uint32_t simulationMahDrawn;
    int32_t simulationAltitude;
    int32_t simulationVerticalSpeed;
    uint16_t simulationCoreTemperature;
    bool simulationGpsHealthy;
}

uint32_t simulationFeatureFlags = FEATURE_GPS;

/* #define DEBUG_OSD */

#include "unittest_macros.h"
#include "unittest_displayport.h"
#include "gtest/gtest.h"

void setDefaultSimulationState()
{
    memset(osdElementConfigMutable(), 0, sizeof(osdElementConfig_t));

    osdConfigMutable()->enabled_stats = 0;

    rssi = 1024;

    simulationBatteryState = BATTERY_OK;
    simulationBatteryCellCount = 4;
    simulationBatteryVoltage = 1680;
    simulationBatteryAmperage = 0;
    simulationMahDrawn = 0;
    simulationAltitude = 0;
    simulationVerticalSpeed = 0;
    simulationCoreTemperature = 0;
    simulationGpsHealthy = false;

    rcData[PITCH] = 1500;

    simulationTime = 0;
    osdFlyTime = 0;
}

/*
 * Performs a test of the OSD actions on arming.
 * (reused throughout the test suite)
 */
void doTestArm(bool testEmpty = true)
{
    // given
    // craft has been armed
    ENABLE_ARMING_FLAG(ARMED);

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // arming alert displayed
    displayPortTestBufferSubstring(12, 7, "ARMED");

    // given
    // armed alert times out (0.5 seconds)
    simulationTime += 0.5e6;

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // arming alert disappears
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    if (testEmpty) {
        displayPortTestBufferIsEmpty();
    }
}

/*
 * Auxiliary function. Test is there're stats that must be shown
 */
bool isSomeStatEnabled(void) {
    return (osdConfigMutable()->enabled_stats != 0);
}

/*
 * Performs a test of the OSD actions on disarming.
 * (reused throughout the test suite)
 */
void doTestDisarm()
{
    // given
    // craft is disarmed after having been armed
    DISABLE_ARMING_FLAG(ARMED);

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // post flight statistics displayed
    if (isSomeStatEnabled()) {
        unsigned enabledStats = osdConfigMutable()->enabled_stats;
        unsigned count = 0;
        while (enabledStats) {
           count += enabledStats & 1;
           enabledStats >>= 1;
        }

        displayPortTestBufferSubstring(2, 7 - count / 2, "  --- STATS ---");
    }
}

void setupStats(void)
{
    // this set of enabled post flight statistics
    osdStatSetState(OSD_STAT_MAX_SPEED, true);
    osdStatSetState(OSD_STAT_MIN_BATTERY, true);
    osdStatSetState(OSD_STAT_MIN_RSSI, true);
    osdStatSetState(OSD_STAT_MAX_CURRENT, false);
    osdStatSetState(OSD_STAT_USED_MAH, false);
    osdStatSetState(OSD_STAT_MAX_ALTITUDE, true);
    osdStatSetState(OSD_STAT_BLACKBOX, false);
    osdStatSetState(OSD_STAT_END_BATTERY, true);
    osdStatSetState(OSD_STAT_RTC_DATE_TIME, true);
    osdStatSetState(OSD_STAT_MAX_DISTANCE, true);
    osdStatSetState(OSD_STAT_FLIGHT_DISTANCE, true);
    osdStatSetState(OSD_STAT_BLACKBOX_NUMBER, false);
    osdStatSetState(OSD_STAT_MAX_G_FORCE, false);
    osdStatSetState(OSD_STAT_MAX_ESC_TEMP, false);
    osdStatSetState(OSD_STAT_MAX_ESC_RPM, false);
}

void simulateFlight(void)
{
    // these conditions occur during flight
    rssi = 1024;
    gpsSol.groundSpeed = 500;
    GPS_distanceToHome = 20;
    GPS_distanceFlownInCm = 2000;
    simulationBatteryVoltage = 1580;
    simulationAltitude = 100;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    rssi = 512;
    gpsSol.groundSpeed = 800;
    GPS_distanceToHome = 50;
    GPS_distanceFlownInCm = 10000;
    simulationBatteryVoltage = 1470;
    simulationAltitude = 150;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    rssi = 256;
    gpsSol.groundSpeed = 200;
    GPS_distanceToHome = 100;
    GPS_distanceFlownInCm = 20000;
    simulationBatteryVoltage = 1520;
    simulationAltitude = 200;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    rssi = 256;
    gpsSol.groundSpeed = 800;
    GPS_distanceToHome = 100;
    GPS_distanceFlownInCm = 10000;
    simulationBatteryVoltage = 1470;
    simulationAltitude = 200; // converts to 6.56168 feet which rounds to 6.6 in imperial units stats test
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    simulationBatteryVoltage = 1520;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    rssi = 256;
    gpsSol.groundSpeed = 800;
    GPS_distanceToHome = 1150;
    GPS_distanceFlownInCm = 1050000;
    simulationBatteryVoltage = 1470;
    simulationAltitude = 200;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    simulationBatteryVoltage = 1520;
    simulationTime += 1e6;
    osdRefresh(simulationTime);
}

class OsdTest : public ::testing::Test
{
protected:
    static void SetUpTestCase() {
        displayPortTestInit();
    }

    virtual void SetUp() {
        setDefaultSimulationState();
    }

    virtual void TearDown() {
        // Clean up the armed state without showing stats at the end of a test
        osdConfigMutable()->enabled_stats = 0;

        doTestDisarm();
    }
};

/*
 * Tests initialisation of the OSD and the power on splash screen.
 */
TEST_F(OsdTest, TestInit)
{
    // given
    // this battery configuration (used for battery voltage elements)
    batteryConfigMutable()->vbatmincellvoltage = 330;
    batteryConfigMutable()->vbatmaxcellvoltage = 430;

    // when
    // OSD is initialised
    osdInit(&testDisplayPort, OSD_DISPLAYPORT_DEVICE_AUTO);

    // then
    // display buffer should contain splash screen
    displayPortTestBufferSubstring(7, 8, "MENU:THR MID");
    displayPortTestBufferSubstring(11, 9, "+ YAW LEFT");
    displayPortTestBufferSubstring(11, 10, "+ PITCH UP");

    // when
    // splash screen timeout has elapsed
    simulationTime += 4e6;
    osdUpdate(simulationTime);

    // then
    // display buffer should be empty
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    displayPortTestBufferIsEmpty();
}

/*
 * Tests visibility of the ARMED notification after arming.
 */
TEST_F(OsdTest, TestArm)
{
    doTestArm();
}

/*
 * Tests display and timeout of the post flight statistics screen after disarming.
 */
TEST_F(OsdTest, TestDisarm)
{
    doTestArm();

    doTestDisarm();

    // given
    // post flight stats times out (60 seconds)
    simulationTime += 60e6;

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // post flight stats screen disappears
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    displayPortTestBufferIsEmpty();
}

/*
 * Tests disarming and immediately rearming clears post flight stats and shows ARMED notification.
 */
TEST_F(OsdTest, TestDisarmWithImmediateRearm)
{
    doTestArm();

    doTestDisarm();

    doTestArm();
}

/*
 * Tests dismissing the statistics screen with pitch stick after disarming.
 */
TEST_F(OsdTest, TestDisarmWithDismissStats)
{
    doTestArm();

    doTestDisarm();

    // given
    // sticks have been moved
    rcData[PITCH] = 1800;

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // post flight stats screen disappears
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    displayPortTestBufferIsEmpty();
}

/*
 * Tests the calculation of timing in statistics
 */
TEST_F(OsdTest, TestStatsTiming)
{
    // given
    osdStatSetState(OSD_STAT_RTC_DATE_TIME, true);
    osdStatSetState(OSD_STAT_TIMER_1, true);
    osdStatSetState(OSD_STAT_TIMER_2, true);

    // and
    // this timer 1 configuration
    osdConfigMutable()->timers[OSD_TIMER_1] = OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_HUNDREDTHS, 0);

    // and
    // this timer 2 configuration
    osdConfigMutable()->timers[OSD_TIMER_2] = OSD_TIMER(OSD_TIMER_SRC_LAST_ARMED, OSD_TIMER_PREC_SECOND, 0);

    // and
    // this RTC time
    dateTime_t dateTime;
    dateTime.year = 2017;
    dateTime.month = 11;
    dateTime.day = 19;
    dateTime.hours = 10;
    dateTime.minutes = 12;
    dateTime.seconds = 0;
    dateTime.millis = 0;
    rtcSetDateTime(&dateTime);

    // when
    // the craft is armed
    doTestArm();

    // and
    // these conditions occur during flight
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    // and
    // the craft is disarmed
    doTestDisarm();

    // and
    // the craft is armed again
    doTestArm();

    // and
    // these conditions occur during flight
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    // and
    // the craft is disarmed
    doTestDisarm();

    // then
    // statistics screen should display the following
    int row = 7;
    displayPortTestBufferSubstring(2, row++, "2017-11-19 10:12:");
    displayPortTestBufferSubstring(2, row++, "TOTAL ARM         : 00:02.50");
    displayPortTestBufferSubstring(2, row++, "LAST ARM          : 00:01");
}

/*
 * Tests the calculation of statistics with imperial unit output.
 */
TEST_F(OsdTest, TestStatsImperial)
{
    // given
    setupStats();

    // and
    // using imperial unit system
    osdConfigMutable()->units = UNIT_IMPERIAL;

    // and
    // a GPS fix is present
    stateFlags |= GPS_FIX | GPS_FIX_HOME;

    // when
    // the craft is armed
    doTestArm();

    // and
    simulateFlight();

    // and
    // the craft is disarmed
    doTestDisarm();

    // then
    // statistics screen should display the following
    int row = 5;
    displayPortTestBufferSubstring(2, row++, "MAX ALTITUDE      : 6.6%c", SYM_FT);
    displayPortTestBufferSubstring(2, row++, "MAX SPEED         : 17");
    displayPortTestBufferSubstring(2, row++, "MAX DISTANCE      : 3772%c", SYM_FT);
    displayPortTestBufferSubstring(2, row++, "FLIGHT DISTANCE   : 6.52%c", SYM_MILES);
    displayPortTestBufferSubstring(2, row++, "MIN BATTERY       : 14.70%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "END BATTERY       : 15.20%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "MIN RSSI          : 25%%");
}

/*
 * Tests the calculation of statistics with metric unit output.
 * (essentially an abridged version of the previous test
 */
TEST_F(OsdTest, TestStatsMetric)
{
    // given
    setupStats();

    // and
    // using metric unit system
    osdConfigMutable()->units = UNIT_METRIC;

    // when
    // the craft is armed
    doTestArm();

    // and
    simulateFlight();

    // and
    // the craft is disarmed
    doTestDisarm();

    // then
    // statistics screen should display the following
    int row = 5;
    displayPortTestBufferSubstring(2, row++, "MAX ALTITUDE      : 2.0%c", SYM_M);
    displayPortTestBufferSubstring(2, row++, "MAX SPEED         : 28");
    displayPortTestBufferSubstring(2, row++, "MAX DISTANCE      : 1.15%c", SYM_KM);
    displayPortTestBufferSubstring(2, row++, "FLIGHT DISTANCE   : 10.5%c", SYM_KM);
    displayPortTestBufferSubstring(2, row++, "MIN BATTERY       : 14.70%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "END BATTERY       : 15.20%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "MIN RSSI          : 25%%");
}

/*
 * Tests the calculation of statistics with metric unit output.
 * (essentially an abridged version of the previous test
 */
TEST_F(OsdTest, TestStatsMetricDistanceUnits)
{
    // given
    setupStats();

    // and
    // using metric unit system
    osdConfigMutable()->units = UNIT_METRIC;

    // when
    // the craft is armed
    doTestArm();

    // and
    simulateFlight();

    // and
    // the craft is disarmed
    doTestDisarm();

    // then
    // statistics screen should display the following
    int row = 5;
    displayPortTestBufferSubstring(2, row++, "MAX ALTITUDE      : 2.0%c", SYM_M);
    displayPortTestBufferSubstring(2, row++, "MAX SPEED         : 28");
    displayPortTestBufferSubstring(2, row++, "MAX DISTANCE      : 1.15%c", SYM_KM);
    displayPortTestBufferSubstring(2, row++, "FLIGHT DISTANCE   : 10.5%c", SYM_KM);
    displayPortTestBufferSubstring(2, row++, "MIN BATTERY       : 14.70%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "END BATTERY       : 15.20%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "MIN RSSI          : 25%%");
}

/*
 * Tests activation of alarms and element flashing.
 */
TEST_F(OsdTest, TestAlarms)
{
    // given
    sensorsSet(SENSOR_GPS);

    // and
    // the following OSD elements are visible
    osdElementConfigMutable()->item_pos[OSD_RSSI_VALUE]              = OSD_POS(8, 1)  | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE]       = OSD_POS(12, 1) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_ITEM_TIMER_1]            = OSD_POS(20, 1) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_ITEM_TIMER_2]            = OSD_POS(1, 1)  | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_REMAINING_TIME_ESTIMATE] = OSD_POS(1, 2) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_ALTITUDE]                = OSD_POS(23, 7) | OSD_PROFILE_1_FLAG;

    // and
    // this set of alarm values
    osdConfigMutable()->rssi_alarm = 20;
    osdConfigMutable()->cap_alarm  = 2200;
    osdConfigMutable()->alt_alarm  = 100; // meters

    osdAnalyzeActiveElements();

    // and
    // this timer 1 configuration
    osdConfigMutable()->timers[OSD_TIMER_1] = OSD_TIMER(OSD_TIMER_SRC_ON, OSD_TIMER_PREC_HUNDREDTHS, 3);
    EXPECT_EQ(OSD_TIMER_SRC_ON, OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1]));
    EXPECT_EQ(OSD_TIMER_PREC_HUNDREDTHS, OSD_TIMER_PRECISION(osdConfig()->timers[OSD_TIMER_1]));
    EXPECT_EQ(3, OSD_TIMER_ALARM(osdConfig()->timers[OSD_TIMER_1]));

    // and
    // this timer 2 configuration
    osdConfigMutable()->timers[OSD_TIMER_2] = OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_SECOND, 2);
    EXPECT_EQ(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2]));
    EXPECT_EQ(OSD_TIMER_PREC_SECOND, OSD_TIMER_PRECISION(osdConfig()->timers[OSD_TIMER_2]));
    EXPECT_EQ(2, OSD_TIMER_ALARM(osdConfig()->timers[OSD_TIMER_2]));

    // and
    // using the metric unit system
    osdConfigMutable()->units = UNIT_METRIC;

    // when
    // time is passing by
    simulationTime += 60e6;
    osdRefresh(simulationTime);

    // and
    // the craft is armed
    doTestArm(false);

    simulationTime += 70e6;
    osdRefresh(simulationTime);

    // then
    // no elements should flash as all values are out of alarm range
    for (int i = 0; i < 30; i++) {
        // Check for visibility every 100ms, elements should always be visible
        simulationTime += 0.1e6;
        osdRefresh(simulationTime);

#ifdef DEBUG_OSD
        printf("%d\n", i);
#endif
        displayPortTestBufferSubstring(1,  1, "%c01:", SYM_FLY_M); // only test the minute part of the timer
        displayPortTestBufferSubstring(8,  1, "%c99", SYM_RSSI);
        displayPortTestBufferSubstring(12, 1, "%c16.8%c", SYM_BATT_FULL, SYM_VOLT);
        displayPortTestBufferSubstring(20, 1, "%c02:", SYM_ON_M); // only test the minute part of the timer
        displayPortTestBufferSubstring(23, 7, "%c0.0%c", SYM_ALTITUDE, SYM_M);
    }

    // when
    // all values are out of range
    rssi = 128;
    simulationBatteryState = BATTERY_CRITICAL;
    simulationBatteryVoltage = 1350;
    simulationAltitude = 12000;
    simulationMahDrawn = 999999;

    simulationTime += 60e6;
    osdRefresh(simulationTime);

    // then
    // elements showing values in alarm range should flash
    for (int i = 0; i < 15; i++) {
        // Blinking should happen at 5Hz
        simulationTime += 0.2e6;
        osdRefresh(simulationTime);

#ifdef DEBUG_OSD
        printf("%d\n", i);
        displayPortTestPrint();
#endif
        if (i % 2 == 1) {
            displayPortTestBufferSubstring(8,  1, "%c12", SYM_RSSI);
            displayPortTestBufferSubstring(12, 1, "%c13.5%c", SYM_MAIN_BATT, SYM_VOLT);
            displayPortTestBufferSubstring(1,  1, "%c02:", SYM_FLY_M); // only test the minute part of the timer
            displayPortTestBufferSubstring(20, 1, "%c03:", SYM_ON_M); // only test the minute part of the timer
            displayPortTestBufferSubstring(23, 7, "%c120.0%c", SYM_ALTITUDE, SYM_M);
        } else {
            displayPortTestBufferIsEmpty();
        }
    }
}

/*
 * Tests the RSSI OSD element.
 */
TEST_F(OsdTest, TestElementRssi)
{
    // given
    osdElementConfigMutable()->item_pos[OSD_RSSI_VALUE] = OSD_POS(8, 1) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->rssi_alarm = 0;

    osdAnalyzeActiveElements();

    // when
    rssi = 1024;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(8, 1, "%c99", SYM_RSSI);

    // when
    rssi = 0;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(8, 1, "%c 0", SYM_RSSI);

    // when
    rssi = 512;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(8, 1, "%c50", SYM_RSSI);
}

/*
 * Tests the instantaneous battery current OSD element.
 */
TEST_F(OsdTest, TestElementAmperage)
{
    // given
    osdElementConfigMutable()->item_pos[OSD_CURRENT_DRAW] = OSD_POS(1, 12) | OSD_PROFILE_1_FLAG;

    osdAnalyzeActiveElements();

    // when
    simulationBatteryAmperage = 0;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 12, "  0.00%c", SYM_AMP);

    // when
    simulationBatteryAmperage = 2156;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 12, " 21.56%c", SYM_AMP);

    // when
    simulationBatteryAmperage = 12345;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 12, "123.45%c", SYM_AMP);
}

/*
 * Tests the battery capacity drawn OSD element.
 */
TEST_F(OsdTest, TestElementMahDrawn)
{
    // given
    osdElementConfigMutable()->item_pos[OSD_MAH_DRAWN] = OSD_POS(1, 11) | OSD_PROFILE_1_FLAG;

    osdAnalyzeActiveElements();

    // when
    simulationMahDrawn = 0;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, "   0%c", SYM_MAH);

    // when
    simulationMahDrawn = 4;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, "   4%c", SYM_MAH);

    // when
    simulationMahDrawn = 15;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, "  15%c", SYM_MAH);

    // when
    simulationMahDrawn = 246;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, " 246%c", SYM_MAH);

    // when
    simulationMahDrawn = 1042;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, "1042%c", SYM_MAH);
}

/*
 * Tests the instantaneous electrical power OSD element.
 */
TEST_F(OsdTest, TestElementPower)
{
    // given
    osdElementConfigMutable()->item_pos[OSD_POWER] = OSD_POS(1, 10)  | OSD_PROFILE_1_FLAG;

    osdAnalyzeActiveElements();

    // and
    simulationBatteryVoltage = 1000; // 10V

    // and
    simulationBatteryAmperage = 0; // 0A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, "   0W");

    // given
    simulationBatteryAmperage = 10; // 0.1A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, "   1W");

    // given
    simulationBatteryAmperage = 120; // 1.2A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, "  12W");

    // given
    simulationBatteryAmperage = 1230; // 12.3A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, " 123W");

    // given
    simulationBatteryAmperage = 12340; // 123.4A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, "1234W");
}

/*
 * Tests the altitude OSD element.
 */
TEST_F(OsdTest, TestElementAltitude)
{
    // given
    osdElementConfigMutable()->item_pos[OSD_ALTITUDE] = OSD_POS(23, 7) | OSD_PROFILE_1_FLAG;

    osdAnalyzeActiveElements();

    // and
    osdConfigMutable()->units = UNIT_METRIC;
    sensorsClear(SENSOR_GPS);

    // when
    simulationAltitude = 0;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "%c-", SYM_ALTITUDE);

    // when
    sensorsSet(SENSOR_GPS);
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "%c0.0%c", SYM_ALTITUDE, SYM_M);

    // when
    simulationAltitude = 247;  // rounds to 2.5m
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "%c2.5%c", SYM_ALTITUDE, SYM_M);

    // when
    simulationAltitude = 4247;  // rounds to 42.5m
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "%c42.5%c", SYM_ALTITUDE, SYM_M);

    // when
    simulationAltitude = -247;  // rounds to -2.5m
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "%c-2.5%c", SYM_ALTITUDE, SYM_M);

    // when
    simulationAltitude = -70;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "%c-0.7%c", SYM_ALTITUDE, SYM_M);

}

/*
 * Tests the core temperature OSD element.
 */
TEST_F(OsdTest, TestElementCoreTemperature)
{
    // given
    osdElementConfigMutable()->item_pos[OSD_CORE_TEMPERATURE] = OSD_POS(1, 8) | OSD_PROFILE_1_FLAG;

    osdAnalyzeActiveElements();

    // and
    osdConfigMutable()->units = UNIT_METRIC;

    // and
    simulationCoreTemperature = 0;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 8, "C%c  0%c", SYM_TEMPERATURE, SYM_C);

    // given
    simulationCoreTemperature = 33;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 8, "C%c 33%c", SYM_TEMPERATURE, SYM_C);

    // given
    osdConfigMutable()->units = UNIT_IMPERIAL;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 8, "C%c 91%c", SYM_TEMPERATURE, SYM_F);
}

/*
 * Tests the battery notifications shown on the warnings OSD element.
 */
TEST_F(OsdTest, TestElementWarningsBattery)
{
    // given
    osdElementConfigMutable()->item_pos[OSD_WARNINGS] = OSD_POS(9, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->enabledWarnings = 0;  // disable all warnings
    osdWarnSetState(OSD_WARNING_BATTERY_WARNING, true);
    osdWarnSetState(OSD_WARNING_BATTERY_CRITICAL, true);
    osdWarnSetState(OSD_WARNING_BATTERY_NOT_FULL, true);

    osdAnalyzeActiveElements();

    // and
    batteryConfigMutable()->vbatfullcellvoltage = 410;

    // and
    // 4S battery
    simulationBatteryCellCount = 4;

    // and
    // used battery
    simulationBatteryVoltage = ((batteryConfig()->vbatmaxcellvoltage - 20) * simulationBatteryCellCount) - 1;
    simulationBatteryState = BATTERY_OK;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, "BATT < FULL");

    // given
    // full battery
    simulationBatteryVoltage = 1680;
    simulationBatteryState = BATTERY_OK;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, "           ");

    // given
    // low battery
    simulationBatteryVoltage = 1400;
    simulationBatteryState = BATTERY_WARNING;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, "LOW BATTERY ");

    // given
    // critical battery
    simulationBatteryVoltage = 1320;
    simulationBatteryState = BATTERY_CRITICAL;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, " LAND NOW   ");

    // given
    // full battery
    simulationBatteryVoltage = ((batteryConfig()->vbatmaxcellvoltage - 20) * simulationBatteryCellCount);
    simulationBatteryState = BATTERY_OK;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, "             ");

    // TODO
}

/*
 * Tests the time string formatting function with a series of precision settings and time values.
 */
TEST_F(OsdTest, TestFormatTimeString)
{
    char buff[OSD_ELEMENT_BUFFER_LENGTH];

    /* Seconds precision, 0 us */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 0);
    EXPECT_EQ(0, strcmp("00:00", buff));

    /* Seconds precision, 0.9 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 0.9e6);
    EXPECT_EQ(0, strcmp("00:00", buff));

    /* Seconds precision, 10 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 10e6);
    EXPECT_EQ(0, strcmp("00:10", buff));

    /* Seconds precision, 1 minute */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 60e6);
    EXPECT_EQ(0, strcmp("01:00", buff));

    /* Seconds precision, 1 minute 59 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 119e6);
    EXPECT_EQ(0, strcmp("01:59", buff));

    /* Hundredths precision, 0 us */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 0);
    EXPECT_EQ(0, strcmp("00:00.00", buff));

    /* Hundredths precision, 10 milliseconds (one 100th of a second) */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 10e3);
    EXPECT_EQ(0, strcmp("00:00.01", buff));

    /* Hundredths precision, 0.9 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 0.9e6);
    EXPECT_EQ(0, strcmp("00:00.90", buff));

    /* Hundredths precision, 10 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 10e6);
    EXPECT_EQ(0, strcmp("00:10.00", buff));

    /* Hundredths precision, 1 minute */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 60e6);
    EXPECT_EQ(0, strcmp("01:00.00", buff));

    /* Hundredths precision, 1 minute 59 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 119e6);
    EXPECT_EQ(0, strcmp("01:59.00", buff));
}

TEST_F(OsdTest, TestConvertTemperatureUnits)
{
    /* In Celsius */
    osdConfigMutable()->units = UNIT_METRIC;
    EXPECT_EQ(osdConvertTemperatureToSelectedUnit(40), 40);

    /* In Fahrenheit */
    osdConfigMutable()->units = UNIT_IMPERIAL;
    EXPECT_EQ(osdConvertTemperatureToSelectedUnit(40), 104);

    /* In Fahrenheit with rounding */
    osdConfigMutable()->units = UNIT_IMPERIAL;
    EXPECT_EQ(osdConvertTemperatureToSelectedUnit(41), 106);
}

TEST_F(OsdTest, TestGpsElements)
{
    // given
    osdElementConfigMutable()->item_pos[OSD_GPS_SATS] = OSD_POS(2, 4) | OSD_PROFILE_1_FLAG;

    sensorsSet(SENSOR_GPS);
    osdAnalyzeActiveElements();
    
    // when
    simulationGpsHealthy = false;
    gpsSol.numSat = 0;

    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    // Sat indicator should blink and show "NC"
    for (int i = 0; i < 15; i++) {
        // Blinking should happen at 5Hz
        simulationTime += 0.2e6;
        osdRefresh(simulationTime);

        if (i % 2 == 0) {
            displayPortTestBufferSubstring(2, 4, "%c%cNC", SYM_SAT_L, SYM_SAT_R);
        } else {
            displayPortTestBufferIsEmpty();
        }
    }

    // when
    simulationGpsHealthy = true;
    gpsSol.numSat = 0;

    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    // Sat indicator should blink and show "0"
    for (int i = 0; i < 15; i++) {
        // Blinking should happen at 5Hz
        simulationTime += 0.2e6;
        osdRefresh(simulationTime);

        if (i % 2 == 0) {
            displayPortTestBufferSubstring(2, 4, "%c%c 0", SYM_SAT_L, SYM_SAT_R);
        } else {
            displayPortTestBufferIsEmpty();
        }
    }

    // when
    simulationGpsHealthy = true;
    gpsSol.numSat = 10;

    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    // Sat indicator should show "10" without flashing
    for (int i = 0; i < 15; i++) {
        // Blinking should happen at 5Hz
        simulationTime += 0.2e6;
        osdRefresh(simulationTime);

        displayPortTestBufferSubstring(2, 4, "%c%c10", SYM_SAT_L, SYM_SAT_R);
    }
}

// STUBS
extern "C" {
    bool featureIsEnabled(uint32_t f) { return simulationFeatureFlags & f; }

    void beeperConfirmationBeeps(uint8_t) {}

    bool isModeActivationConditionPresent(boxId_e) {
        return false;
    }

    bool IS_RC_MODE_ACTIVE(boxId_e) {
        return false;
    }

    uint32_t micros() {
        return simulationTime;
    }

    uint32_t millis() {
        return micros() / 1000;
    }

    bool isBeeperOn() {
        return false;
    }

    bool airmodeIsEnabled() {
        return false;
    }

    uint8_t getCurrentPidProfileIndex() {
        return 0;
    }

    uint8_t getCurrentControlRateProfileIndex() {
        return 0;
    }

    batteryState_e getBatteryState() {
        return simulationBatteryState;
    }

    uint8_t getBatteryCellCount() {
        return simulationBatteryCellCount;
    }

    uint16_t getBatteryVoltage() {
        return simulationBatteryVoltage;
    }

    uint16_t getBatteryAverageCellVoltage() {
        return simulationBatteryVoltage / simulationBatteryCellCount;
    }

    int32_t getAmperage() {
        return simulationBatteryAmperage;
    }

    int32_t getMAhDrawn() {
        return simulationMahDrawn;
    }

    int32_t getEstimatedAltitudeCm() {
        return simulationAltitude;
    }

    int32_t getEstimatedVario() {
        return simulationVerticalSpeed;
    }

    int32_t blackboxGetLogNumber() {
        return 0;
    }

    bool isBlackboxDeviceWorking() {
        return true;
    }

    bool isBlackboxDeviceFull() {
        return false;
    }

    bool isSerialTransmitBufferEmpty(const serialPort_t *) {
        return false;
    }

    void serialWrite(serialPort_t *, uint8_t) {}

    bool cmsDisplayPortRegister(displayPort_t *) {
        return false;
    }

    uint16_t getRssi(void) { return rssi; }

    uint8_t getRssiPercent(void) { return scaleRange(rssi, 0, RSSI_MAX_VALUE, 0, 100); }

    uint16_t rxGetLinkQuality(void) { return LINK_QUALITY_MAX_VALUE; }

    uint16_t getCoreTemperatureCelsius(void) { return simulationCoreTemperature; }

    bool isFlipOverAfterCrashActive(void) { return false; }

    float pidItermAccelerator(void) { return 1.0; }
    uint8_t getMotorCount(void){ return 4; }
    bool areMotorsRunning(void){ return true; }
    bool pidOsdAntiGravityActive(void) { return false; }
    bool failsafeIsActive(void) { return false; }
    bool gpsRescueIsConfigured(void) { return false; }
    bool gpsIsHealthy(void) { return simulationGpsHealthy; }
    int8_t calculateThrottlePercent(void) { return 0; }
    uint32_t persistentObjectRead(persistentObjectId_e) { return 0; }
    void persistentObjectWrite(persistentObjectId_e, uint32_t) {}
    bool isUpright(void) { return true; }
    float getMotorOutputLow(void) { return 1000.0; }
    float getMotorOutputHigh(void) { return 2047.0; }
    void ignoreTaskShortExecTime(void) {}
    void ignoreTaskStateTime(void) {}
}
