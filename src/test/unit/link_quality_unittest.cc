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

    #include "common/crc.h"
    #include "common/printf.h"
    #include "common/streambuf.h"
    #include "common/time.h"
    #include "common/utils.h"

    #include "config/config.h"

    #include "drivers/osd_symbols.h"
    #include "drivers/persistent.h"
    #include "drivers/serial.h"
    #include "drivers/system.h"

    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"

    #include "io/beeper.h"
    #include "io/gps.h"
    #include "io/serial.h"

    #include "osd/osd.h"
    #include "osd/osd_elements.h"
    #include "osd/osd_warnings.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "rx/rx.h"

    #include "sensors/battery.h"

    attitudeEulerAngles_t attitude;
    float rMat[3][3];

    pidProfile_t *currentPidProfile;
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint8_t GPS_numSat;
    uint16_t GPS_distanceToHome;
    int16_t GPS_directionToHome;
    uint32_t GPS_distanceFlownInCm;
    int32_t GPS_coord[2];
    gpsSolutionData_t gpsSol;
    float motor[8];
    acc_t acc;
    float accAverage[XYZ_AXIS_COUNT];

    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);
    PG_REGISTER(imuConfig_t, imuConfig, PG_IMU_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

    timeUs_t simulationTime = 0;

    void osdRefresh(timeUs_t currentTimeUs);
    uint16_t updateLinkQualitySamples(uint16_t value);
#define LINK_QUALITY_SAMPLE_COUNT 16
}

/* #define DEBUG_OSD */

#include "unittest_macros.h"
#include "unittest_displayport.h"
#include "gtest/gtest.h"

extern "C" {
    PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);

    boxBitmask_t rcModeActivationMask;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode = 0;

    uint16_t updateLinkQualitySamples(uint16_t value);

    extern uint16_t applyRxChannelRangeConfiguraton(int sample, const rxChannelRangeConfig_t *range);
}
void setDefaultSimulationState()
{

    setLinkQualityDirect(LINK_QUALITY_MAX_VALUE);

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
        displayPortTestBufferSubstring(2, 2, "  --- STATS ---");
    }
}

/*
 * Tests initialisation of the OSD and the power on splash screen.
 */
TEST(LQTest, TestInit)
{
    // given
    // display port is initialised
    displayPortTestInit();

    // and
    // default state values are set
    setDefaultSimulationState();

    // and
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
 * Tests the Tests the OSD_LINK_QUALITY element updateLinkQualitySamples  default LQ_SOURCE_NONE
 */
TEST(LQTest, TestElement_LQ_SOURCE_NONE_SAMPLES)
{
    // given


    linkQualitySource = LQ_SOURCE_NONE;

    osdElementConfigMutable()->item_pos[OSD_LINK_QUALITY] = OSD_POS(8, 1) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->link_quality_alarm = 0;

    osdAnalyzeActiveElements();

    // when samples populated 100%
    for (int x = 0; x < LINK_QUALITY_SAMPLE_COUNT; x++) {
        setLinkQualityDirect(updateLinkQualitySamples(LINK_QUALITY_MAX_VALUE));
    }


    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(8, 1, "%c9", SYM_LINK_QUALITY);

    // when  updateLinkQualitySamples used   50% rounds to 4
    for (int x = 0; x < LINK_QUALITY_SAMPLE_COUNT; x++) {
        setLinkQualityDirect(updateLinkQualitySamples(LINK_QUALITY_MAX_VALUE));
        setLinkQualityDirect(updateLinkQualitySamples(0));
    }

    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(8, 1, "%c4", SYM_LINK_QUALITY);

}
/*
 * Tests the Tests the OSD_LINK_QUALITY element values  default LQ_SOURCE_NONE
 */
TEST(LQTest, TestElement_LQ_SOURCE_NONE_VALUES)
{
    // given


    linkQualitySource = LQ_SOURCE_NONE;

    osdElementConfigMutable()->item_pos[OSD_LINK_QUALITY] = OSD_POS(8, 1) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->link_quality_alarm = 0;

    osdAnalyzeActiveElements();
    // when LINK_QUALITY_MAX_VALUE to 1 by 10%
    uint16_t testscale = 0;
    for (int testdigit = 10; testdigit > 0; testdigit--) {
        testscale = testdigit * 102.3;
        setLinkQualityDirect(testscale);
        displayClearScreen(&testDisplayPort);
        osdRefresh(simulationTime);
#ifdef DEBUG_OSD
        printf("%d %d\n",testscale, testdigit);
        displayPortTestPrint();
#endif
        // then
        if (testdigit >= 10) {
            displayPortTestBufferSubstring(8, 1,"%c9", SYM_LINK_QUALITY);
        }else{
            displayPortTestBufferSubstring(8, 1,"%c%1d", SYM_LINK_QUALITY, testdigit - 1);
        }
    }
}
/*
 * Tests the OSD_LINK_QUALITY element LQ RX_PROTOCOL_CRSF.
 */
TEST(LQTest, TestElementLQ_PROTOCOL_CRSF_VALUES)
{
    // given
    linkQualitySource = LQ_SOURCE_RX_PROTOCOL_CRSF;

    osdElementConfigMutable()->item_pos[OSD_LINK_QUALITY] = OSD_POS(8, 1) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->link_quality_alarm = 0;

    osdAnalyzeActiveElements();

    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // crsf setLinkQualityDirect 0-300;

    for (uint8_t x = 0; x <= 99; x++) {
        for (uint8_t m = 0; m <= 4; m++) {
            // when x scaled
            setLinkQualityDirect(x);
            rxSetRfMode(m);
            // then rxGetLinkQuality Osd should be x
            // and RfMode should be m
            displayClearScreen(&testDisplayPort);
            osdRefresh(simulationTime);
                displayPortTestBufferSubstring(8, 1, "%c%1d:%2d", SYM_LINK_QUALITY, m, x);
            }
        }
    }
/*
 * Tests the LQ Alarms
 *
*/
TEST(LQTest, TestLQAlarm)
{
    // given
    // default state is set
    setDefaultSimulationState();

    linkQualitySource = LQ_SOURCE_NONE;

    // and
    // the following OSD elements are visible

    osdElementConfigMutable()->item_pos[OSD_LINK_QUALITY] = OSD_POS(8, 1)  | OSD_PROFILE_1_FLAG;

    // and
    // this set of alarm values

    osdConfigMutable()->link_quality_alarm = 80;
    stateFlags |= GPS_FIX | GPS_FIX_HOME;

    osdAnalyzeActiveElements();

    // and
    // using the metric unit system
    osdConfigMutable()->units = UNIT_METRIC;

    // when
    // the craft is armed
    doTestArm(false);

    for (int x = 0; x < LINK_QUALITY_SAMPLE_COUNT; x++) {
        setLinkQualityDirect(updateLinkQualitySamples(LINK_QUALITY_MAX_VALUE));
    }

    // then
    // no elements should flash as all values are out of alarm range
    for (int i = 0; i < 30; i++) {
        // Check for visibility every 100ms, elements should always be visible
        simulationTime += 0.1e6;
        osdRefresh(simulationTime);

#ifdef DEBUG_OSD
        printf("%d\n", i);
#endif
        displayPortTestBufferSubstring(8,  1, "%c9", SYM_LINK_QUALITY);

    }

    setLinkQualityDirect(512);
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
        if (i % 2 == 0) {
            displayPortTestBufferSubstring(8,  1, "%c5", SYM_LINK_QUALITY);
        } else {
            displayPortTestBufferIsEmpty();
        }
    }

    doTestDisarm();
    simulationTime += 60e6;
    osdRefresh(simulationTime);
}

// STUBS
extern "C" {

    uint32_t micros() {
        return simulationTime;
    }

    uint32_t microsISR() {
        return micros();
    }

    uint32_t millis() {
        return micros() / 1000;
    }

    bool featureIsEnabled(uint32_t) { return true; }
    void beeperConfirmationBeeps(uint8_t) {}
    bool isBeeperOn() { return false; }
    uint8_t getCurrentPidProfileIndex() { return 0; }
    uint8_t getCurrentControlRateProfileIndex() { return 0; }
    batteryState_e getBatteryState() { return BATTERY_OK; }
    uint8_t getBatteryCellCount() { return 4; }
    uint16_t getBatteryVoltage() { return 1680; }
    uint16_t getBatteryAverageCellVoltage() { return  420; }
    int32_t getAmperage() { return 0; }
    int32_t getMAhDrawn() { return 0; }
    int32_t getEstimatedAltitudeCm() { return 0; }
    int32_t getEstimatedVario() { return 0; }
    int32_t blackboxGetLogNumber() { return 0; }
    bool isBlackboxDeviceWorking() { return true; }
    bool isBlackboxDeviceFull() { return false; }
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
    const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e ) {return NULL;}
    bool telemetryCheckRxPortShared(const serialPortConfig_t *) {return false;}
    bool cmsDisplayPortRegister(displayPort_t *) { return false; }
    uint16_t getCoreTemperatureCelsius(void) { return 0; }
    bool isFlipOverAfterCrashActive(void) { return false; }
    float pidItermAccelerator(void) { return 1.0; }
    uint8_t getMotorCount(void){ return 4; }
    bool areMotorsRunning(void){ return true; }
    bool pidOsdAntiGravityActive(void) { return false; }
    bool failsafeIsActive(void) { return false; }
    bool gpsIsHealthy(void) { return true; }
    bool gpsRescueIsConfigured(void) { return false; }
    int8_t calculateThrottlePercent(void) { return 0; }
    uint32_t persistentObjectRead(persistentObjectId_e) { return 0; }
    void persistentObjectWrite(persistentObjectId_e, uint32_t) {}
    void failsafeOnRxSuspend(uint32_t ) {}
    void failsafeOnRxResume(void) {}
    void featureDisableImmediate(uint32_t) { }
    bool rxMspFrameComplete(void) { return false; }
    bool isPPMDataBeingReceived(void) { return false; }
    bool isPWMDataBeingReceived(void) { return false; }
    void resetPPMDataReceivedState(void){ }
    void failsafeOnValidDataReceived(void) { }
    void failsafeOnValidDataFailed(void) { }
    void pinioBoxTaskControl(void) { }
    bool taskUpdateRxMainInProgress() { return true; }

    void rxPwmInit(rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(rxRuntimeState);
        UNUSED(callback);
    }

    bool sbusInit(rxConfig_t *initialRxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(initialRxConfig);
        UNUSED(rxRuntimeState);
        UNUSED(callback);
        return true;
    }

    bool spektrumInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(rxConfig);
        UNUSED(rxRuntimeState);
        UNUSED(callback);
        return true;
    }

    bool sumdInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(rxConfig);
        UNUSED(rxRuntimeState);
        UNUSED(callback);
        return true;
    }

    bool sumhInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(rxConfig);
        UNUSED(rxRuntimeState);
        UNUSED(callback);
        return true;
    }

    bool crsfRxInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback);

    bool jetiExBusInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(rxConfig);
        UNUSED(rxRuntimeState);
        UNUSED(callback);
        return true;
    }

    bool ibusInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(rxConfig);
        UNUSED(rxRuntimeState);
        UNUSED(callback);
        return true;
    }

    bool xBusInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(rxConfig);
        UNUSED(rxRuntimeState);
        UNUSED(callback);
        return true;
    }

    bool rxMspInit(rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, rcReadRawDataFnPtr *callback)
    {
        UNUSED(rxConfig);
        UNUSED(rxRuntimeState);
        UNUSED(callback);
        return true;
    }

    float pt1FilterGain(float f_cut, float dT)
    {
        UNUSED(f_cut);
        UNUSED(dT);
        return 0.0;
    }

    void pt1FilterInit(pt1Filter_t *filter, float k)
    {
        UNUSED(filter);
        UNUSED(k);
    }

    float pt1FilterApply(pt1Filter_t *filter, float input)
    {
        UNUSED(filter);
        UNUSED(input);
        return 0.0;
    }

    bool isUpright(void) { return true; }

    float getMotorOutputLow(void) { return 1000.0; }

    float getMotorOutputHigh(void) { return 2047.0; }

    void ignoreTaskShortExecTime(void) {}
    void ignoreTaskStateTime(void) {}
}
