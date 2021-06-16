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
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "cms/cms.h"

#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/stack_check.h"
#include "drivers/transponder_ir.h"
#include "drivers/usb_io.h"
#include "drivers/vtx_common.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/dispatch.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/piniobox.h"
#include "io/serial.h"
#include "io/transponder_ir.h"
#include "io/vtx_tramp.h" // Will be gone
#include "io/rcdevice_cam.h"
#include "io/usb_cdc_hid.h"
#include "io/vtx.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/rangefinder.h"

#include "telemetry/telemetry.h"
#include "telemetry/crsf.h"

#ifdef USE_BST
#include "i2c_bst.h"
#endif

#ifdef USE_USB_CDC_HID
//TODO: Make it platform independent in the future
#ifdef STM32F4
#include "vcpf4/usbd_cdc_vcp.h"
#include "usbd_hid_core.h"
#elif defined(STM32F7)
#include "usbd_cdc_interface.h"
#include "usbd_hid.h"
#endif
#endif

#include "tasks.h"

static void taskMain(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

#ifdef USE_SDCARD
    afatfs_poll();
#endif
}

static void taskHandleSerial(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

#if defined(USE_VCP)
    DEBUG_SET(DEBUG_USB, 0, usbCableIsInserted());
    DEBUG_SET(DEBUG_USB, 1, usbVcpIsConnected());
#endif

#ifdef USE_CLI
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }
#endif
    bool evaluateMspData = ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA;
    mspSerialProcess(evaluateMspData, mspFcProcessCommand, mspFcProcessReply);
}

static void taskBatteryAlerts(timeUs_t currentTimeUs)
{
    if (!ARMING_FLAG(ARMED)) {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }
    batteryUpdateStates(currentTimeUs);
    batteryUpdateAlarms();
}

#ifdef USE_ACC
static void taskUpdateAccelerometer(timeUs_t currentTimeUs)
{
    accUpdate(currentTimeUs, &accelerometerConfigMutable()->accelerometerTrims);
}
#endif

static enum {
    CHECK, PROCESS, MODES, UPDATE
} rxState = CHECK;

bool taskUpdateRxMainInProgress()
{
    return (rxState != CHECK);
}

static void taskUpdateRxMain(timeUs_t currentTimeUs)
{
    // Where we are using a state machine call ignoreTaskStateTime() for all states bar one
    if (rxState != MODES) {
        ignoreTaskStateTime();
    }

    switch (rxState) {
    default:
    case CHECK:
        rxState = PROCESS;
        break;

    case PROCESS:
        if (!processRx(currentTimeUs)) {
            rxState = CHECK;
            break;
        }
        rxState = MODES;
        break;

    case MODES:
        processRxModes(currentTimeUs);
        rxState = UPDATE;
        break;

    case UPDATE:
        // updateRcCommands sets rcCommand, which is needed by updateAltHoldState and updateSonarAltHoldState
        updateRcCommands();
        updateArmingStatus();

#ifdef USE_USB_CDC_HID
        if (!ARMING_FLAG(ARMED)) {
            sendRcDataToHid();
        }
#endif
        rxState = CHECK;
        break;
    }
}


#ifdef USE_BARO
static void taskUpdateBaro(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (sensors(SENSOR_BARO)) {
        const uint32_t newDeadline = baroUpdate();
        if (newDeadline != 0) {
            rescheduleTask(TASK_SELF, newDeadline);
        }
    }
}
#endif

#if defined(USE_RANGEFINDER)
void taskUpdateRangefinder(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!sensors(SENSOR_RANGEFINDER)) {
        return;
    }

    rangefinderUpdate();

    rangefinderProcess(getCosTiltAngle());
}
#endif

#if defined(USE_BARO) || defined(USE_GPS)
static void taskCalculateAltitude(timeUs_t currentTimeUs)
{
    calculateEstimatedAltitude(currentTimeUs);
}
#endif // USE_BARO || USE_GPS

#ifdef USE_TELEMETRY
static void taskTelemetry(timeUs_t currentTimeUs)
{
    if (!cliMode && featureIsEnabled(FEATURE_TELEMETRY)) {
        subTaskTelemetryPollSensors(currentTimeUs);

        telemetryProcess(currentTimeUs);
    }
}
#endif

#ifdef USE_CAMERA_CONTROL
static void taskCameraControl(uint32_t currentTime)
{
    if (ARMING_FLAG(ARMED)) {
        return;
    }

    cameraControlProcess(currentTime);
}
#endif

void tasksInit(void)
{
    schedulerInit();

    setTaskEnabled(TASK_MAIN, true);

    setTaskEnabled(TASK_SERIAL, true);
    rescheduleTask(TASK_SERIAL, TASK_PERIOD_HZ(serialConfig()->serial_update_rate_hz));

    const bool useBatteryVoltage = batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
    setTaskEnabled(TASK_BATTERY_VOLTAGE, useBatteryVoltage);

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    // If vbat motor output compensation is used, use fast vbat samplingTime
    if (isSagCompensationConfigured()) {
        rescheduleTask(TASK_BATTERY_VOLTAGE, TASK_PERIOD_HZ(FAST_VOLTAGE_TASK_FREQ_HZ));
    }
#endif

    const bool useBatteryCurrent = batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
    setTaskEnabled(TASK_BATTERY_CURRENT, useBatteryCurrent);
    const bool useBatteryAlerts = batteryConfig()->useVBatAlerts || batteryConfig()->useConsumptionAlerts || featureIsEnabled(FEATURE_OSD);
    setTaskEnabled(TASK_BATTERY_ALERTS, (useBatteryVoltage || useBatteryCurrent) && useBatteryAlerts);

#ifdef USE_STACK_CHECK
    setTaskEnabled(TASK_STACK_CHECK, true);
#endif

    if (sensors(SENSOR_GYRO)) {
        rescheduleTask(TASK_GYRO, gyro.sampleLooptime);
        rescheduleTask(TASK_FILTER, gyro.targetLooptime);
        rescheduleTask(TASK_PID, gyro.targetLooptime);
        setTaskEnabled(TASK_GYRO, true);
        setTaskEnabled(TASK_FILTER, true);
        setTaskEnabled(TASK_PID, true);
        schedulerEnableGyro();
    }

#if defined(USE_ACC)
    if (sensors(SENSOR_ACC) && acc.sampleRateHz) {
        setTaskEnabled(TASK_ACCEL, true);
        rescheduleTask(TASK_ACCEL, TASK_PERIOD_HZ(acc.sampleRateHz));
        setTaskEnabled(TASK_ATTITUDE, true);
    }
#endif

#ifdef USE_RANGEFINDER
    if (sensors(SENSOR_RANGEFINDER)) {
        setTaskEnabled(TASK_RANGEFINDER, featureIsEnabled(FEATURE_RANGEFINDER));
    }
#endif

    setTaskEnabled(TASK_RX, true);

    setTaskEnabled(TASK_DISPATCH, dispatchIsEnabled());

#ifdef USE_BEEPER
    setTaskEnabled(TASK_BEEPER, true);
#endif

#ifdef USE_GPS
    setTaskEnabled(TASK_GPS, featureIsEnabled(FEATURE_GPS));
#endif

#ifdef USE_MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#endif

#ifdef USE_BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif

#if defined(USE_BARO) || defined(USE_GPS)
    setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || featureIsEnabled(FEATURE_GPS));
#endif

#ifdef USE_DASHBOARD
    setTaskEnabled(TASK_DASHBOARD, featureIsEnabled(FEATURE_DASHBOARD));
#endif

#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        setTaskEnabled(TASK_TELEMETRY, true);
        if (rxRuntimeState.serialrxProvider == SERIALRX_JETIEXBUS) {
            // Reschedule telemetry to 500hz for Jeti Exbus
            rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
        } else if (rxRuntimeState.serialrxProvider == SERIALRX_CRSF) {
            // Reschedule telemetry to 500hz, 2ms for CRSF
            rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
        }
    }
#endif

#ifdef USE_LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, featureIsEnabled(FEATURE_LED_STRIP));
#endif

#ifdef USE_TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, featureIsEnabled(FEATURE_TRANSPONDER));
#endif

#ifdef USE_OSD
    rescheduleTask(TASK_OSD, TASK_PERIOD_HZ(osdConfig()->task_frequency));
    setTaskEnabled(TASK_OSD, featureIsEnabled(FEATURE_OSD) && osdGetDisplayPort(NULL));
#endif

#ifdef USE_BST
    setTaskEnabled(TASK_BST_MASTER_PROCESS, true);
#endif

#ifdef USE_ESC_SENSOR
    setTaskEnabled(TASK_ESC_SENSOR, featureIsEnabled(FEATURE_ESC_SENSOR));
#endif

#ifdef USE_ADC_INTERNAL
    setTaskEnabled(TASK_ADC_INTERNAL, true);
#endif

#ifdef USE_PINIOBOX
    pinioBoxTaskControl();
#endif

#ifdef USE_CMS
#ifdef USE_MSP_DISPLAYPORT
    setTaskEnabled(TASK_CMS, true);
#else
    setTaskEnabled(TASK_CMS, featureIsEnabled(FEATURE_OSD) || featureIsEnabled(FEATURE_DASHBOARD));
#endif
#endif

#ifdef USE_VTX_CONTROL
#if defined(USE_VTX_RTC6705) || defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
    setTaskEnabled(TASK_VTXCTRL, true);
#endif
#endif

#ifdef USE_CAMERA_CONTROL
    setTaskEnabled(TASK_CAMCTRL, true);
#endif

#ifdef USE_RCDEVICE
    setTaskEnabled(TASK_RCDEVICE, rcdeviceIsEnabled());
#endif

#ifdef USE_CRSF_V3
    const bool useCRSF = rxRuntimeState.serialrxProvider == SERIALRX_CRSF;
    setTaskEnabled(TASK_SPEED_NEGOTIATION, useCRSF);
#endif
}

#define DEFINE_TASK(taskNameParam, subTaskNameParam, checkFuncParam, taskFuncParam, desiredPeriodParam, staticPriorityParam) {  \
    .taskName = taskNameParam, \
    .subTaskName = subTaskNameParam, \
    .checkFunc = checkFuncParam, \
    .taskFunc = taskFuncParam, \
    .desiredPeriodUs = desiredPeriodParam, \
    .staticPriority = staticPriorityParam \
}

task_t tasks[TASK_COUNT] = {
    [TASK_SYSTEM] = DEFINE_TASK("SYSTEM", "LOAD", NULL, taskSystemLoad, TASK_PERIOD_HZ(10), TASK_PRIORITY_MEDIUM_HIGH),
    [TASK_MAIN] = DEFINE_TASK("SYSTEM", "UPDATE", NULL, taskMain, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM_HIGH),
    [TASK_SERIAL] = DEFINE_TASK("SERIAL", NULL, NULL, taskHandleSerial, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW), // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
    [TASK_BATTERY_ALERTS] = DEFINE_TASK("BATTERY_ALERTS", NULL, NULL, taskBatteryAlerts, TASK_PERIOD_HZ(5), TASK_PRIORITY_MEDIUM),
    [TASK_BATTERY_VOLTAGE] = DEFINE_TASK("BATTERY_VOLTAGE", NULL, NULL, batteryUpdateVoltage, TASK_PERIOD_HZ(SLOW_VOLTAGE_TASK_FREQ_HZ), TASK_PRIORITY_MEDIUM), // Freq may be updated in tasksInit
    [TASK_BATTERY_CURRENT] = DEFINE_TASK("BATTERY_CURRENT", NULL, NULL, batteryUpdateCurrentMeter, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM),

#ifdef USE_TRANSPONDER
    [TASK_TRANSPONDER] = DEFINE_TASK("TRANSPONDER", NULL, NULL, transponderUpdate, TASK_PERIOD_HZ(250), TASK_PRIORITY_LOW),
#endif

#ifdef USE_STACK_CHECK
    [TASK_STACK_CHECK] = DEFINE_TASK("STACKCHECK", NULL, NULL, taskStackCheck, TASK_PERIOD_HZ(10), TASK_PRIORITY_IDLE),
#endif

    [TASK_GYRO] = DEFINE_TASK("GYRO", NULL, NULL, taskGyroSample, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME),
    [TASK_FILTER] = DEFINE_TASK("FILTER", NULL, NULL, taskFiltering, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME),
    [TASK_PID] = DEFINE_TASK("PID", NULL, NULL, taskMainPidLoop, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME),
#ifdef USE_ACC
    [TASK_ACCEL] = DEFINE_TASK("ACC", NULL, NULL, taskUpdateAccelerometer, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM),
    [TASK_ATTITUDE] = DEFINE_TASK("ATTITUDE", NULL, NULL, imuUpdateAttitude, TASK_PERIOD_HZ(100), TASK_PRIORITY_MEDIUM),
#endif
    [TASK_RX] = DEFINE_TASK("RX", NULL, rxUpdateCheck, taskUpdateRxMain, TASK_PERIOD_HZ(33), TASK_PRIORITY_HIGH), // If event-based scheduling doesn't work, fallback to periodic scheduling
    [TASK_DISPATCH] = DEFINE_TASK("DISPATCH", NULL, NULL, dispatchProcess, TASK_PERIOD_HZ(1000), TASK_PRIORITY_HIGH),

#ifdef USE_BEEPER
    [TASK_BEEPER] = DEFINE_TASK("BEEPER", NULL, NULL, beeperUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_GPS
    [TASK_GPS] = DEFINE_TASK("GPS", NULL, NULL, gpsUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_MEDIUM), // Required to prevent buffer overruns if running at 115200 baud (115 bytes / period < 256 bytes buffer)
#endif

#ifdef USE_MAG
    [TASK_COMPASS] = DEFINE_TASK("COMPASS", NULL, NULL, compassUpdate,TASK_PERIOD_HZ(10), TASK_PRIORITY_LOW),
#endif

#ifdef USE_BARO
    [TASK_BARO] = DEFINE_TASK("BARO", NULL, NULL, taskUpdateBaro, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
#endif

#if defined(USE_BARO) || defined(USE_GPS)
    [TASK_ALTITUDE] = DEFINE_TASK("ALTITUDE", NULL, NULL, taskCalculateAltitude, TASK_PERIOD_HZ(40), TASK_PRIORITY_LOW),
#endif

#ifdef USE_DASHBOARD
    [TASK_DASHBOARD] = DEFINE_TASK("DASHBOARD", NULL, NULL, dashboardUpdate, TASK_PERIOD_HZ(10), TASK_PRIORITY_LOW),
#endif

#ifdef USE_OSD
    [TASK_OSD] = DEFINE_TASK("OSD", NULL, NULL, osdUpdate, TASK_PERIOD_HZ(OSD_TASK_FREQUENCY_DEFAULT), TASK_PRIORITY_LOW),
#endif

#ifdef USE_TELEMETRY
    [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", NULL, NULL, taskTelemetry, TASK_PERIOD_HZ(250), TASK_PRIORITY_LOW),
#endif

#ifdef USE_LED_STRIP
    [TASK_LEDSTRIP] = DEFINE_TASK("LEDSTRIP", NULL, NULL, ledStripUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_BST
    [TASK_BST_MASTER_PROCESS] = DEFINE_TASK("BST_MASTER_PROCESS", NULL, NULL, taskBstMasterProcess, TASK_PERIOD_HZ(50), TASK_PRIORITY_IDLE),
#endif

#ifdef USE_ESC_SENSOR
    [TASK_ESC_SENSOR] = DEFINE_TASK("ESC_SENSOR", NULL, NULL, escSensorProcess, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_CMS
    [TASK_CMS] = DEFINE_TASK("CMS", NULL, NULL, cmsHandler, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
#endif

#ifdef USE_VTX_CONTROL
    [TASK_VTXCTRL] = DEFINE_TASK("VTXCTRL", NULL, NULL, vtxUpdate, TASK_PERIOD_HZ(5), TASK_PRIORITY_IDLE),
#endif

#ifdef USE_RCDEVICE
    [TASK_RCDEVICE] = DEFINE_TASK("RCDEVICE", NULL, NULL, rcdeviceUpdate, TASK_PERIOD_HZ(20), TASK_PRIORITY_MEDIUM),
#endif

#ifdef USE_CAMERA_CONTROL
    [TASK_CAMCTRL] = DEFINE_TASK("CAMCTRL", NULL, NULL, taskCameraControl, TASK_PERIOD_HZ(5), TASK_PRIORITY_IDLE),
#endif

#ifdef USE_ADC_INTERNAL
    [TASK_ADC_INTERNAL] = DEFINE_TASK("ADCINTERNAL", NULL, NULL, adcInternalProcess, TASK_PERIOD_HZ(1), TASK_PRIORITY_IDLE),
#endif

#ifdef USE_PINIOBOX
    [TASK_PINIOBOX] = DEFINE_TASK("PINIOBOX", NULL, NULL, pinioBoxUpdate, TASK_PERIOD_HZ(20), TASK_PRIORITY_IDLE),
#endif

#ifdef USE_RANGEFINDER
    [TASK_RANGEFINDER] = DEFINE_TASK("RANGEFINDER", NULL, NULL, taskUpdateRangefinder, TASK_PERIOD_HZ(10), TASK_PRIORITY_IDLE),
#endif

#ifdef USE_CRSF_V3
    [TASK_SPEED_NEGOTIATION] = DEFINE_TASK("SPEED_NEGOTIATION", NULL, NULL, speedNegotiationProcess, TASK_PERIOD_HZ(100), TASK_PRIORITY_IDLE),
#endif
};

task_t *getTask(unsigned taskId)
{
    return &tasks[taskId];
}
