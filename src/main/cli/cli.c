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
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "platform.h"

// FIXME remove this for targets that don't need a CLI.  Perhaps use a no-op macro when USE_CLI is not enabled
// signal that we're in cli mode
bool cliMode = false;

#ifdef USE_CLI

#include "blackbox/blackbox.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cli/settings.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/printf_serial.h"
#include "common/strtol.h"
#include "common/time.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/simplified_tuning.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/adc.h"
#include "drivers/buf_writer.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/pwm_output_dshot_shared.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/display.h"
#include "drivers/dma.h"
#include "drivers/flash.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/light_led.h"
#include "drivers/motor.h"
#include "drivers/rangefinder/rangefinder_hcsr04.h"
#include "drivers/resource.h"
#include "drivers/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/sound_beeper.h"
#include "drivers/stack_check.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir.h"
#include "drivers/usb_msc.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "fc/board_info.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/servos.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/transponder_ir.h"
#include "io/usb_msc.h"
#include "io/vtx_control.h"
#include "io/vtx.h"

#include "msp/msp.h"
#include "msp/msp_box.h"
#include "msp/msp_protocol.h"

#include "osd/osd.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/board.h"
#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"
#include "pg/gyrodev.h"
#include "pg/max7456.h"
#include "pg/mco.h"
#include "pg/motor.h"
#include "pg/pinio.h"
#include "pg/pin_pull_up_down.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/rx_pwm.h"
#include "pg/rx_spi_cc2500.h"
#include "pg/serial_uart.h"
#include "pg/sdio.h"
#include "pg/timerio.h"
#include "pg/timerup.h"
#include "pg/usb.h"
#include "pg/vtx_table.h"

#include "rx/rx_bind.h"
#include "rx/rx_spi.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/sensors.h"

#include "telemetry/frsky_hub.h"
#include "telemetry/telemetry.h"

#include "cli.h"

static serialPort_t *cliPort = NULL;

#ifdef STM32F1
#define CLI_IN_BUFFER_SIZE 128
#else
// Space required to set array parameters
#define CLI_IN_BUFFER_SIZE 256
#endif
#define CLI_OUT_BUFFER_SIZE 64

static bufWriter_t *cliWriter = NULL;
static bufWriter_t *cliErrorWriter = NULL;
static uint8_t cliWriteBuffer[sizeof(*cliWriter) + CLI_OUT_BUFFER_SIZE];

static char cliBuffer[CLI_IN_BUFFER_SIZE];
static uint32_t bufferIndex = 0;

static bool configIsInCopy = false;

#define CURRENT_PROFILE_INDEX -1
static int8_t pidProfileIndexToUse = CURRENT_PROFILE_INDEX;
static int8_t rateProfileIndexToUse = CURRENT_PROFILE_INDEX;

#ifdef USE_CLI_BATCH
static bool commandBatchActive = false;
static bool commandBatchError = false;
#endif

#if defined(USE_BOARD_INFO)
static bool boardInformationUpdated = false;
#if defined(USE_SIGNATURE)
static bool signatureUpdated = false;
#endif
#endif // USE_BOARD_INFO

static const char* const emptyName = "-";
static const char* const emptyString = "";

#if !defined(USE_CUSTOM_DEFAULTS)
#define CUSTOM_DEFAULTS_START ((char*)0)
#define CUSTOM_DEFAULTS_END ((char *)0)
#else
extern char __custom_defaults_start;
extern char __custom_defaults_end;
#define CUSTOM_DEFAULTS_START (&__custom_defaults_start)
#define CUSTOM_DEFAULTS_END (&__custom_defaults_end)

static bool processingCustomDefaults = false;
static char cliBufferTemp[CLI_IN_BUFFER_SIZE];

#define CUSTOM_DEFAULTS_START_PREFIX ("# " FC_FIRMWARE_NAME)
#define CUSTOM_DEFAULTS_MANUFACTURER_ID_PREFIX "# config: manufacturer_id: "
#define CUSTOM_DEFAULTS_BOARD_NAME_PREFIX ", board_name: "
#define CUSTOM_DEFAULTS_CHANGESET_ID_PREFIX ", version: "
#define CUSTOM_DEFAULTS_DATE_PREFIX ", date: "

#define MAX_CHANGESET_ID_LENGTH 8
#define MAX_DATE_LENGTH 20

static bool customDefaultsHeaderParsed = false;
static bool customDefaultsFound = false;
static char customDefaultsManufacturerId[MAX_MANUFACTURER_ID_LENGTH + 1] = { 0 };
static char customDefaultsBoardName[MAX_BOARD_NAME_LENGTH + 1] = { 0 };
static char customDefaultsChangesetId[MAX_CHANGESET_ID_LENGTH + 1] = { 0 };
static char customDefaultsDate[MAX_DATE_LENGTH + 1] = { 0 };
#endif

#if defined(USE_CUSTOM_DEFAULTS_ADDRESS)
static char __attribute__ ((section(".custom_defaults_start_address"))) *customDefaultsStart = CUSTOM_DEFAULTS_START;
static char __attribute__ ((section(".custom_defaults_end_address"))) *customDefaultsEnd = CUSTOM_DEFAULTS_END;
#endif

#ifndef USE_QUAD_MIXER_ONLY
// sync this with mixerMode_e
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", "QUADX1234", NULL
};
#endif

// sync this with features_e
static const char * const featureNames[] = {
    "RX_PPM", "", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "",
    "RANGEFINDER", "TELEMETRY", "", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DISPLAY", "OSD",
    "", "CHANNEL_FORWARDING", "TRANSPONDER", "AIRMODE",
    "", "", "RX_SPI", "", "ESC_SENSOR", "ANTI_GRAVITY", "DYNAMIC_FILTER", NULL
};

// sync this with rxFailsafeChannelMode_e
static const char rxFailsafeModeCharacters[] = "ahs";

static const rxFailsafeChannelMode_e rxFailsafeModesTable[RX_FAILSAFE_TYPE_COUNT][RX_FAILSAFE_MODE_COUNT] = {
    { RX_FAILSAFE_MODE_AUTO, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_SET },
    { RX_FAILSAFE_MODE_INVALID, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_SET }
};

#if defined(USE_SENSOR_NAMES)
// sync this with sensors_e
static const char *const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "RANGEFINDER", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG | SENSOR_RANGEFINDER)

static const char * const *sensorHardwareNames[] = {
    lookupTableGyroHardware, lookupTableAccHardware, lookupTableBaroHardware, lookupTableMagHardware, lookupTableRangefinderHardware
};
#endif // USE_SENSOR_NAMES

// Needs to be aligned with mcuTypeId_e
static const char *mcuTypeNames[] = {
    "SIMULATOR",
    "F103",
    "F303",
    "F40X",
    "F411",
    "F446",
    "F722",
    "F745",
    "F746",
    "F765",
    "H750",
    "H743 (Rev Unknown)",
    "H743 (Rev.Y)",
    "H743 (Rev.X)",
    "H743 (Rev.V)",
    "H7A3",
    "H723/H725",
    "G474",
};

static const char *configurationStates[] = { "UNCONFIGURED", "CUSTOM DEFAULTS", "CONFIGURED" };

typedef enum dumpFlags_e {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_RATES = (1 << 2),
    DUMP_ALL = (1 << 3),
    DO_DIFF = (1 << 4),
    SHOW_DEFAULTS = (1 << 5),
    HIDE_UNUSED = (1 << 6),
    HARDWARE_ONLY = (1 << 7),
    BARE = (1 << 8),
} dumpFlags_t;

typedef bool printFn(dumpFlags_t dumpMask, bool equalsDefault, const char *format, ...);

typedef enum {
    REBOOT_TARGET_FIRMWARE,
    REBOOT_TARGET_BOOTLOADER_ROM,
    REBOOT_TARGET_BOOTLOADER_FLASH,
} rebootTarget_e;

typedef struct serialPassthroughPort_s {
    int id;
    uint32_t baud;
    unsigned mode;
    serialPort_t *port;
} serialPassthroughPort_t;

static void cliWriterFlushInternal(bufWriter_t *writer)
{
    if (writer) {
        bufWriterFlush(writer);
    }
}

static void cliPrintInternal(bufWriter_t *writer, const char *str)
{
    if (writer) {
        while (*str) {
            bufWriterAppend(writer, *str++);
        }
        cliWriterFlushInternal(writer);
    }
}

static void cliWriterFlush()
{
    cliWriterFlushInternal(cliWriter);
}

void cliPrint(const char *str)
{
    cliPrintInternal(cliWriter, str);
}

void cliPrintLinefeed(void)
{
    cliPrint("\r\n");
}

void cliPrintLine(const char *str)
{
    cliPrint(str);
    cliPrintLinefeed();
}

#ifdef MINIMAL_CLI
#define cliPrintHashLine(str)
#else
static void cliPrintHashLine(const char *str)
{
    cliPrint("\r\n# ");
    cliPrintLine(str);
}
#endif

static void cliPutp(void *p, char ch)
{
    bufWriterAppend(p, ch);
}

static void cliPrintfva(const char *format, va_list va)
{
    if (cliWriter) {
        tfp_format(cliWriter, cliPutp, format, va);
        cliWriterFlush();
    }
}

static bool cliDumpPrintLinef(dumpFlags_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if (!((dumpMask & DO_DIFF) && equalsDefault)) {
        va_list va;
        va_start(va, format);
        cliPrintfva(format, va);
        va_end(va);
        cliPrintLinefeed();
        return true;
    } else {
        return false;
    }
}

static void cliWrite(uint8_t ch)
{
    if (cliWriter) {
        bufWriterAppend(cliWriter, ch);
    }
}

static bool cliDefaultPrintLinef(dumpFlags_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if ((dumpMask & SHOW_DEFAULTS) && !equalsDefault) {
        cliWrite('#');

        va_list va;
        va_start(va, format);
        cliPrintfva(format, va);
        va_end(va);
        cliPrintLinefeed();
        return true;
    } else {
        return false;
    }
}

void cliPrintf(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintfva(format, va);
    va_end(va);
}


void cliPrintLinef(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintfva(format, va);
    va_end(va);
    cliPrintLinefeed();
}

static void cliPrintErrorVa(const char *cmdName, const char *format, va_list va)
{
    if (cliErrorWriter) {
        cliPrintInternal(cliErrorWriter, "###ERROR IN ");
        cliPrintInternal(cliErrorWriter, cmdName);
        cliPrintInternal(cliErrorWriter, ": ");

        tfp_format(cliErrorWriter, cliPutp, format, va);
        va_end(va);

        cliPrintInternal(cliErrorWriter, "###");
    }

#ifdef USE_CLI_BATCH
    if (commandBatchActive) {
        commandBatchError = true;
    }
#endif
}

static void cliPrintError(const char *cmdName, const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintErrorVa(cmdName, format, va);

    if (!cliWriter) {
        // Supply our own linefeed in case we are printing inside a custom defaults operation
        // TODO: Fix this by rewriting the entire CLI to have self contained line feeds
        // instead of expecting the directly following command to supply the line feed.
        cliPrintInternal(cliErrorWriter, "\r\n");
    }
}

static void cliPrintErrorLinef(const char *cmdName, const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintErrorVa(cmdName, format, va);
    cliPrintInternal(cliErrorWriter, "\r\n");
}

static void getMinMax(const clivalue_t *var, int *min, int *max)
{
    switch (var->type & VALUE_TYPE_MASK) {
    case VAR_UINT8:
    case VAR_UINT16:
        *min = var->config.minmaxUnsigned.min;
        *max = var->config.minmaxUnsigned.max;

        break;
    default:
        *min = var->config.minmax.min;
        *max = var->config.minmax.max;

        break;
    }
}

static void printValuePointer(const char *cmdName, const clivalue_t *var, const void *valuePointer, bool full)
{
    if ((var->type & VALUE_MODE_MASK) == MODE_ARRAY) {
        for (int i = 0; i < var->config.array.length; i++) {
            switch (var->type & VALUE_TYPE_MASK) {
            default:
            case VAR_UINT8:
                // uint8_t array
                cliPrintf("%d", ((uint8_t *)valuePointer)[i]);
                break;

            case VAR_INT8:
                // int8_t array
                cliPrintf("%d", ((int8_t *)valuePointer)[i]);
                break;

            case VAR_UINT16:
                // uin16_t array
                cliPrintf("%d", ((uint16_t *)valuePointer)[i]);
                break;

            case VAR_INT16:
                // int16_t array
                cliPrintf("%d", ((int16_t *)valuePointer)[i]);
                break;

            case VAR_UINT32:
                // uin32_t array
                cliPrintf("%u", ((uint32_t *)valuePointer)[i]);
                break;
            }

            if (i < var->config.array.length - 1) {
                cliPrint(",");
            }
        }
    } else {
        int value = 0;

        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            value = *(uint8_t *)valuePointer;

            break;
        case VAR_INT8:
            value = *(int8_t *)valuePointer;

            break;
        case VAR_UINT16:
            value = *(uint16_t *)valuePointer;

            break;
        case VAR_INT16:
            value = *(int16_t *)valuePointer;

            break;
        case VAR_UINT32:
            value = *(uint32_t *)valuePointer;

            break;
        }

        bool valueIsCorrupted = false;
        switch (var->type & VALUE_MODE_MASK) {
        case MODE_DIRECT:
            if ((var->type & VALUE_TYPE_MASK) == VAR_UINT32) {
                cliPrintf("%u", (uint32_t)value);
                if ((uint32_t)value > var->config.u32Max) {
                    valueIsCorrupted = true;
                } else if (full) {
                    cliPrintf(" 0 %u", var->config.u32Max);
                }
            } else {
                int min;
                int max;
                getMinMax(var, &min, &max);

                cliPrintf("%d", value);
                if ((value < min) || (value > max)) {
                    valueIsCorrupted = true;
                } else if (full) {
                    cliPrintf(" %d %d", min, max);
                }
            }
            break;
        case MODE_LOOKUP:
            if (value < lookupTables[var->config.lookup.tableIndex].valueCount) {
                cliPrint(lookupTables[var->config.lookup.tableIndex].values[value]);
            } else {
                valueIsCorrupted = true;
            }
            break;
        case MODE_BITSET:
            if (value & 1 << var->config.bitpos) {
                cliPrintf("ON");
            } else {
                cliPrintf("OFF");
            }
            break;
        case MODE_STRING:
            cliPrintf("%s", (strlen((char *)valuePointer) == 0) ? "-" : (char *)valuePointer);
            break;
        }

        if (valueIsCorrupted) {
            cliPrintLinefeed();
            cliPrintError(cmdName, "CORRUPTED CONFIG: %s = %d", var->name, value);
        }
    }
}


static bool valuePtrEqualsDefault(const clivalue_t *var, const void *ptr, const void *ptrDefault)
{
    bool result = true;
    int elementCount = 1;
    uint32_t mask = 0xffffffff;

    if ((var->type & VALUE_MODE_MASK) == MODE_ARRAY) {
        elementCount = var->config.array.length;
    }
    if ((var->type & VALUE_MODE_MASK) == MODE_BITSET) {
        mask = 1 << var->config.bitpos;
    }
    for (int i = 0; i < elementCount; i++) {
        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            result = result && (((uint8_t *)ptr)[i] & mask) == (((uint8_t *)ptrDefault)[i] & mask);
            break;

        case VAR_INT8:
            result = result && ((int8_t *)ptr)[i] == ((int8_t *)ptrDefault)[i];
            break;

        case VAR_UINT16:
            result = result && (((uint16_t *)ptr)[i] & mask) == (((uint16_t *)ptrDefault)[i] & mask);
            break;
        case VAR_INT16:
            result = result && ((int16_t *)ptr)[i] == ((int16_t *)ptrDefault)[i];
            break;
        case VAR_UINT32:
            result = result && (((uint32_t *)ptr)[i] & mask) == (((uint32_t *)ptrDefault)[i] & mask);
            break;
        }
    }

    return result;
}

static const char *cliPrintSectionHeading(dumpFlags_t dumpMask, bool outputFlag, const char *headingStr)
{
    if (headingStr && (!(dumpMask & DO_DIFF) || outputFlag)) {
        cliPrintHashLine(headingStr);
        return NULL;
    } else {
        return headingStr;
    }
}

static void backupPgConfig(const pgRegistry_t *pg)
{
    memcpy(pg->copy, pg->address, pg->size);
}

static void restorePgConfig(const pgRegistry_t *pg)
{
    memcpy(pg->address, pg->copy, pg->size);
}

static void backupConfigs(void)
{
    if (configIsInCopy) {
        return;
    }

    // make copies of configs to do differencing
    PG_FOREACH(pg) {
        backupPgConfig(pg);
    }

    configIsInCopy = true;
}

static void restoreConfigs(void)
{
    if (!configIsInCopy) {
        return;
    }

    PG_FOREACH(pg) {
        restorePgConfig(pg);
    }

    configIsInCopy = false;
}

#if defined(USE_RESOURCE_MGMT) || defined(USE_TIMER_MGMT)
static bool isReadingConfigFromCopy()
{
    return configIsInCopy;
}
#endif

static bool isWritingConfigToCopy()
{
    return configIsInCopy
#if defined(USE_CUSTOM_DEFAULTS)
        && !processingCustomDefaults
#endif
        ;
}

#if defined(USE_CUSTOM_DEFAULTS)
static bool cliProcessCustomDefaults(bool quiet);
#endif

static void backupAndResetConfigs(const bool useCustomDefaults)
{
    backupConfigs();

    // reset all configs to defaults to do differencing
    resetConfig();

#if defined(USE_CUSTOM_DEFAULTS)
    if (useCustomDefaults) {
        if (!cliProcessCustomDefaults(true)) {
            cliPrintLine("###WARNING: NO CUSTOM DEFAULTS FOUND###");
        }
    }
#else
    UNUSED(useCustomDefaults);
#endif
}

static uint8_t getPidProfileIndexToUse()
{
    return pidProfileIndexToUse == CURRENT_PROFILE_INDEX ? getCurrentPidProfileIndex() : pidProfileIndexToUse;
}

static uint8_t getRateProfileIndexToUse()
{
    return rateProfileIndexToUse == CURRENT_PROFILE_INDEX ? getCurrentControlRateProfileIndex() : rateProfileIndexToUse;
}


static uint16_t getValueOffset(const clivalue_t *value)
{
    switch (value->type & VALUE_SECTION_MASK) {
    case MASTER_VALUE:
    case HARDWARE_VALUE:
        return value->offset;
    case PROFILE_VALUE:
        return value->offset + sizeof(pidProfile_t) * getPidProfileIndexToUse();
    case PROFILE_RATE_VALUE:
        return value->offset + sizeof(controlRateConfig_t) * getRateProfileIndexToUse();
    }
    return 0;
}

STATIC_UNIT_TESTED void *cliGetValuePointer(const clivalue_t *value)
{
    const pgRegistry_t* rec = pgFind(value->pgn);
    if (isWritingConfigToCopy()) {
        return CONST_CAST(void *, rec->copy + getValueOffset(value));
    } else {
        return CONST_CAST(void *, rec->address + getValueOffset(value));
    }
}

static const char *dumpPgValue(const char *cmdName, const clivalue_t *value, dumpFlags_t dumpMask, const char *headingStr)
{
    const pgRegistry_t *pg = pgFind(value->pgn);
#ifdef DEBUG
    if (!pg) {
        cliPrintLinef("VALUE %s ERROR", value->name);
        return headingStr; // if it's not found, the pgn shouldn't be in the value table!
    }
#endif

    const char *format = "set %s = ";
    const char *defaultFormat = "#set %s = ";
    const int valueOffset = getValueOffset(value);
    const bool equalsDefault = valuePtrEqualsDefault(value, pg->copy + valueOffset, pg->address + valueOffset);

    headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
    if (((dumpMask & DO_DIFF) == 0) || !equalsDefault) {
        if (dumpMask & SHOW_DEFAULTS && !equalsDefault) {
            cliPrintf(defaultFormat, value->name);
            printValuePointer(cmdName, value, (uint8_t*)pg->address + valueOffset, false);
            cliPrintLinefeed();
        }
        cliPrintf(format, value->name);
        printValuePointer(cmdName, value, pg->copy + valueOffset, false);
        cliPrintLinefeed();
    }
    return headingStr;
}

static void dumpAllValues(const char *cmdName, uint16_t valueSection, dumpFlags_t dumpMask, const char *headingStr)
{
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);

    for (uint32_t i = 0; i < valueTableEntryCount; i++) {
        const clivalue_t *value = &valueTable[i];
        cliWriterFlush();
        if ((value->type & VALUE_SECTION_MASK) == valueSection || ((valueSection == MASTER_VALUE) && (value->type & VALUE_SECTION_MASK) == HARDWARE_VALUE)) {
            headingStr = dumpPgValue(cmdName, value, dumpMask, headingStr);
        }
    }
}

static void cliPrintVar(const char *cmdName, const clivalue_t *var, bool full)
{
    const void *ptr = cliGetValuePointer(var);

    printValuePointer(cmdName, var, ptr, full);
}

static void cliPrintVarRange(const clivalue_t *var)
{
    switch (var->type & VALUE_MODE_MASK) {
    case (MODE_DIRECT): {
        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT32:
            cliPrintLinef("Allowed range: 0 - %u", var->config.u32Max);

            break;
        case VAR_UINT8:
        case VAR_UINT16:
            cliPrintLinef("Allowed range: %d - %d", var->config.minmaxUnsigned.min, var->config.minmaxUnsigned.max);

            break;
        default:
            cliPrintLinef("Allowed range: %d - %d", var->config.minmax.min, var->config.minmax.max);

            break;
        }
    }
    break;
    case (MODE_LOOKUP): {
        const lookupTableEntry_t *tableEntry = &lookupTables[var->config.lookup.tableIndex];
        cliPrint("Allowed values: ");
        bool firstEntry = true;
        for (unsigned i = 0; i < tableEntry->valueCount; i++) {
            if (tableEntry->values[i]) {
                if (!firstEntry) {
                    cliPrint(", ");
                }
                cliPrintf("%s", tableEntry->values[i]);
                firstEntry = false;
            }
        }
        cliPrintLinefeed();
    }
    break;
    case (MODE_ARRAY): {
        cliPrintLinef("Array length: %d", var->config.array.length);
    }
    break;
    case (MODE_STRING): {
        cliPrintLinef("String length: %d - %d", var->config.string.minlength, var->config.string.maxlength);
    }
    break;
    case (MODE_BITSET): {
        cliPrintLinef("Allowed values: OFF, ON");
    }
    break;
    }
}

static void cliSetVar(const clivalue_t *var, const uint32_t value)
{
    void *ptr = cliGetValuePointer(var);
    uint32_t workValue;
    uint32_t mask;

    if ((var->type & VALUE_MODE_MASK) == MODE_BITSET) {
        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            mask = (1 << var->config.bitpos) & 0xff;
            if (value) {
                workValue = *(uint8_t *)ptr | mask;
            } else {
                workValue = *(uint8_t *)ptr & ~mask;
            }
            *(uint8_t *)ptr = workValue;
            break;

        case VAR_UINT16:
            mask = (1 << var->config.bitpos) & 0xffff;
            if (value) {
                workValue = *(uint16_t *)ptr | mask;
            } else {
                workValue = *(uint16_t *)ptr & ~mask;
            }
            *(uint16_t *)ptr = workValue;
            break;

        case VAR_UINT32:
            mask = 1 << var->config.bitpos;
            if (value) {
                workValue = *(uint32_t *)ptr | mask;
            } else {
                workValue = *(uint32_t *)ptr & ~mask;
            }
            *(uint32_t *)ptr = workValue;
            break;
        }
    } else {
        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            *(uint8_t *)ptr = value;
            break;

        case VAR_INT8:
            *(int8_t *)ptr = value;
            break;

        case VAR_UINT16:
            *(uint16_t *)ptr = value;
            break;

        case VAR_INT16:
            *(int16_t *)ptr = value;
            break;

        case VAR_UINT32:
            *(uint32_t *)ptr = value;
            break;
        }
    }
}

#if defined(USE_RESOURCE_MGMT) && !defined(MINIMAL_CLI)
static void cliRepeat(char ch, uint8_t len)
{
    if (cliWriter) {
        for (int i = 0; i < len; i++) {
            bufWriterAppend(cliWriter, ch);
        }
        cliPrintLinefeed();
    }
}
#endif

static void cliPrompt(void)
{
#if defined(USE_CUSTOM_DEFAULTS) && defined(DEBUG_CUSTOM_DEFAULTS)
    if (processingCustomDefaults) {
        cliPrint("\r\nd: #");
    } else
#endif
    {
        cliPrint("\r\n# ");
    }
}

static void cliShowParseError(const char *cmdName)
{
    cliPrintErrorLinef(cmdName, "PARSING FAILED");
}

static void cliShowInvalidArgumentCountError(const char *cmdName)
{
    cliPrintErrorLinef(cmdName, "INVALID ARGUMENT COUNT", cmdName);
}

static void cliShowArgumentRangeError(const char *cmdName, char *name, int min, int max)
{
    if (name) {
        cliPrintErrorLinef(cmdName, "%s NOT BETWEEN %d AND %d", name, min, max);
    } else {
        cliPrintErrorLinef(cmdName, "ARGUMENT OUT OF RANGE");
    }
}

static const char *nextArg(const char *currentArg)
{
    const char *ptr = strchr(currentArg, ' ');
    while (ptr && *ptr == ' ') {
        ptr++;
    }

    return ptr;
}

static const char *processChannelRangeArgs(const char *ptr, channelRange_t *range, uint8_t *validArgumentCount)
{
    for (uint32_t argIndex = 0; argIndex < 2; argIndex++) {
        ptr = nextArg(ptr);
        if (ptr) {
            int val = atoi(ptr);
            val = CHANNEL_VALUE_TO_STEP(val);
            if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
                if (argIndex == 0) {
                    range->startStep = val;
                } else {
                    range->endStep = val;
                }
                (*validArgumentCount)++;
            }
        }
    }

    return ptr;
}

// Check if a string's length is zero
static bool isEmpty(const char *string)
{
    return (string == NULL || *string == '\0') ? true : false;
}

static void printRxFailsafe(dumpFlags_t dumpMask, const rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs, const rxFailsafeChannelConfig_t *defaultRxFailsafeChannelConfigs, const char *headingStr)
{
    // print out rxConfig failsafe settings
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
        const rxFailsafeChannelConfig_t *channelFailsafeConfig = &rxFailsafeChannelConfigs[channel];
        const rxFailsafeChannelConfig_t *defaultChannelFailsafeConfig = &defaultRxFailsafeChannelConfigs[channel];
        const bool equalsDefault = !memcmp(channelFailsafeConfig, defaultChannelFailsafeConfig, sizeof(*channelFailsafeConfig));
        const bool requireValue = channelFailsafeConfig->mode == RX_FAILSAFE_MODE_SET;
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        if (requireValue) {
            const char *format = "rxfail %u %c %d";
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[defaultChannelFailsafeConfig->mode],
                RXFAIL_STEP_TO_CHANNEL_VALUE(defaultChannelFailsafeConfig->step)
            );
            cliDumpPrintLinef(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[channelFailsafeConfig->mode],
                RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step)
            );
        } else {
            const char *format = "rxfail %u %c";
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[defaultChannelFailsafeConfig->mode]
            );
            cliDumpPrintLinef(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[channelFailsafeConfig->mode]
            );
        }
    }
}

static void cliRxFailsafe(const char *cmdName, char *cmdline)
{
    uint8_t channel;
    char buf[3];

    if (isEmpty(cmdline)) {
        // print out rxConfig failsafe settings
        for (channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
            cliRxFailsafe(cmdName, itoa(channel, buf, 10));
        }
    } else {
        const char *ptr = cmdline;
        channel = atoi(ptr++);
        if ((channel < MAX_SUPPORTED_RC_CHANNEL_COUNT)) {

            rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigsMutable(channel);

            const rxFailsafeChannelType_e type = (channel < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_TYPE_FLIGHT : RX_FAILSAFE_TYPE_AUX;
            rxFailsafeChannelMode_e mode = channelFailsafeConfig->mode;
            bool requireValue = channelFailsafeConfig->mode == RX_FAILSAFE_MODE_SET;

            ptr = nextArg(ptr);
            if (ptr) {
                const char *p = strchr(rxFailsafeModeCharacters, *(ptr));
                if (p) {
                    const uint8_t requestedMode = p - rxFailsafeModeCharacters;
                    mode = rxFailsafeModesTable[type][requestedMode];
                } else {
                    mode = RX_FAILSAFE_MODE_INVALID;
                }
                if (mode == RX_FAILSAFE_MODE_INVALID) {
                    cliShowParseError(cmdName);
                    return;
                }

                requireValue = mode == RX_FAILSAFE_MODE_SET;

                ptr = nextArg(ptr);
                if (ptr) {
                    if (!requireValue) {
                        cliShowParseError(cmdName);
                        return;
                    }
                    uint16_t value = atoi(ptr);
                    value = CHANNEL_VALUE_TO_RXFAIL_STEP(value);
                    if (value > MAX_RXFAIL_RANGE_STEP) {
                        cliPrintErrorLinef(cmdName, "value out of range: %d", value);
                        return;
                    }

                    channelFailsafeConfig->step = value;
                } else if (requireValue) {
                    cliShowInvalidArgumentCountError(cmdName);
                    return;
                }
                channelFailsafeConfig->mode = mode;
            }

            char modeCharacter = rxFailsafeModeCharacters[channelFailsafeConfig->mode];

            // double use of cliPrintf below
            // 1. acknowledge interpretation on command,
            // 2. query current setting on single item,

            if (requireValue) {
                cliPrintLinef("rxfail %u %c %d",
                    channel,
                    modeCharacter,
                    RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step)
                );
            } else {
                cliPrintLinef("rxfail %u %c",
                    channel,
                    modeCharacter
                );
            }
        } else {
            cliShowArgumentRangeError(cmdName, "CHANNEL", 0, MAX_SUPPORTED_RC_CHANNEL_COUNT - 1);
        }
    }
}

static void printAux(dumpFlags_t dumpMask, const modeActivationCondition_t *modeActivationConditions, const modeActivationCondition_t *defaultModeActivationConditions, const char *headingStr)
{
    const char *format = "aux %u %u %u %u %u %u %u";
    // print out aux channel settings
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = &modeActivationConditions[i];
        bool equalsDefault = false;
        if (defaultModeActivationConditions) {
            const modeActivationCondition_t *macDefault = &defaultModeActivationConditions[i];
            equalsDefault = !isModeActivationConditionConfigured(mac, macDefault);
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            const box_t *box = findBoxByBoxId(macDefault->modeId);
            const box_t *linkedTo = findBoxByBoxId(macDefault->linkedTo);
            if (box) {
                cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                    i,
                    box->permanentId,
                    macDefault->auxChannelIndex,
                    MODE_STEP_TO_CHANNEL_VALUE(macDefault->range.startStep),
                    MODE_STEP_TO_CHANNEL_VALUE(macDefault->range.endStep),
                    macDefault->modeLogic,
                    linkedTo ? linkedTo->permanentId : 0
                );
            }
        }
        const box_t *box = findBoxByBoxId(mac->modeId);
        const box_t *linkedTo = findBoxByBoxId(mac->linkedTo);
        if (box) {
            cliDumpPrintLinef(dumpMask, equalsDefault, format,
                i,
                box->permanentId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep),
                mac->modeLogic,
                linkedTo ? linkedTo->permanentId : 0
            );
        }
    }
}

static void cliAux(const char *cmdName, char *cmdline)
{
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printAux(DUMP_MASTER, modeActivationConditions(0), NULL, NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            uint8_t validArgumentCount = 0;
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                const box_t *box = findBoxByPermanentId(val);
                if (box) {
                    mac->modeId = box->boxId;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    mac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &mac->range, &validArgumentCount);
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val == MODELOGIC_OR || val == MODELOGIC_AND) {
                    mac->modeLogic = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                const box_t *box = findBoxByPermanentId(val);
                if (box) {
                    mac->linkedTo = box->boxId;
                    validArgumentCount++;
                }
            }
            if (validArgumentCount == 4) { // for backwards compatibility
                mac->modeLogic = MODELOGIC_OR;
                mac->linkedTo = 0;
            } else if (validArgumentCount == 5) { // for backwards compatibility
                mac->linkedTo = 0;
            } else if (validArgumentCount != 6) {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
            analyzeModeActivationConditions();
            cliPrintLinef( "aux %u %u %u %u %u %u %u",
                i,
                findBoxByBoxId(mac->modeId)->permanentId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep),
                mac->modeLogic,
                findBoxByBoxId(mac->linkedTo)->permanentId
            );
        } else {
            cliShowArgumentRangeError(cmdName, "INDEX", 0, MAX_MODE_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}

static void printSerial(dumpFlags_t dumpMask, const serialConfig_t *serialConfig, const serialConfig_t *serialConfigDefault, const char *headingStr)
{
    const char *format = "serial %d %d %ld %ld %ld %ld";
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < SERIAL_PORT_COUNT; i++) {
        if (!serialIsPortAvailable(serialConfig->portConfigs[i].identifier)) {
            continue;
        };
        bool equalsDefault = false;
        if (serialConfigDefault) {
            equalsDefault = !memcmp(&serialConfig->portConfigs[i], &serialConfigDefault->portConfigs[i], sizeof(serialConfig->portConfigs[i]));
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                serialConfigDefault->portConfigs[i].identifier,
                serialConfigDefault->portConfigs[i].functionMask,
                baudRates[serialConfigDefault->portConfigs[i].msp_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].gps_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].telemetry_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].blackbox_baudrateIndex]
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            serialConfig->portConfigs[i].identifier,
            serialConfig->portConfigs[i].functionMask,
            baudRates[serialConfig->portConfigs[i].msp_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].gps_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].telemetry_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].blackbox_baudrateIndex]
            );
    }
}

static void cliSerial(const char *cmdName, char *cmdline)
{
    const char *format = "serial %d %d %ld %ld %ld %ld";
    if (isEmpty(cmdline)) {
        printSerial(DUMP_MASTER, serialConfig(), NULL, NULL);
        return;
    }
    serialPortConfig_t portConfig;
    memset(&portConfig, 0 , sizeof(portConfig));


    uint8_t validArgumentCount = 0;

    const char *ptr = cmdline;

    int val = atoi(ptr++);
    serialPortConfig_t *currentConfig = serialFindPortConfigurationMutable(val);

    if (currentConfig) {
        portConfig.identifier = val;
        validArgumentCount++;
    }

    ptr = nextArg(ptr);
    if (ptr) {
        val = strtoul(ptr, NULL, 10);
        portConfig.functionMask = val;
        validArgumentCount++;
    }

    for (int i = 0; i < 4; i ++) {
        ptr = nextArg(ptr);
        if (!ptr) {
            break;
        }

        val = atoi(ptr);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch (i) {
        case 0:
            if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_1000000) {
                continue;
            }
            portConfig.msp_baudrateIndex = baudRateIndex;
            break;
        case 1:
            if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                continue;
            }
            portConfig.gps_baudrateIndex = baudRateIndex;
            break;
        case 2:
            if (baudRateIndex != BAUD_AUTO && baudRateIndex > BAUD_115200) {
                continue;
            }
            portConfig.telemetry_baudrateIndex = baudRateIndex;
            break;
        case 3:
            if (baudRateIndex < BAUD_19200 || baudRateIndex > BAUD_2470000) {
                continue;
            }
            portConfig.blackbox_baudrateIndex = baudRateIndex;
            break;
        }

        validArgumentCount++;
    }

    if (validArgumentCount < 6) {
        cliShowInvalidArgumentCountError(cmdName);
        return;
    }

    memcpy(currentConfig, &portConfig, sizeof(portConfig));

    cliDumpPrintLinef(0, false, format,
        portConfig.identifier,
        portConfig.functionMask,
        baudRates[portConfig.msp_baudrateIndex],
        baudRates[portConfig.gps_baudrateIndex],
        baudRates[portConfig.telemetry_baudrateIndex],
        baudRates[portConfig.blackbox_baudrateIndex]
        );

}

#if defined(USE_SERIAL_PASSTHROUGH)
static void cbCtrlLine(void *context, uint16_t ctrl)
{
#ifdef USE_PINIO
    int contextValue = (int)(long)context;
    if (contextValue) {
        pinioSet(contextValue - 1, !(ctrl & CTRL_LINE_STATE_DTR));
    } else
#endif /* USE_PINIO */
    UNUSED(context);

    if (!(ctrl & CTRL_LINE_STATE_DTR)) {
        systemReset();
    }
}

static int cliParseSerialMode(const char *tok)
{
    int mode = 0;

    if (strcasestr(tok, "rx")) {
        mode |= MODE_RX;
    }
    if (strcasestr(tok, "tx")) {
        mode |= MODE_TX;
    }

    return mode;
}

static void cliSerialPassthrough(const char *cmdName, char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliShowInvalidArgumentCountError(cmdName);
        return;
    }

    serialPassthroughPort_t ports[2] = { {SERIAL_PORT_NONE, 0, 0, NULL}, {cliPort->identifier, 0, 0, cliPort} };
    bool enableBaudCb = false;
    int port1PinioDtr = 0;
    bool port1ResetOnDtr = false;
    bool escSensorPassthrough = false;
    char *saveptr;
    char* tok = strtok_r(cmdline, " ", &saveptr);
    int index = 0;

    while (tok != NULL) {
        switch (index) {
        case 0:
            if (strcasestr(tok, "esc_sensor")) {
                escSensorPassthrough = true;
                const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
                ports[0].id = portConfig->identifier;
            } else {
                ports[0].id = atoi(tok);
            }
            break;
        case 1:
            ports[0].baud = atoi(tok);
            break;
        case 2:
            ports[0].mode = cliParseSerialMode(tok);
            break;
        case 3:
            if (strncasecmp(tok, "reset", strlen(tok)) == 0) {
                port1ResetOnDtr = true;
#ifdef USE_PINIO
            } else if (strncasecmp(tok, "none", strlen(tok)) == 0) {
                port1PinioDtr = 0;
            } else {
                port1PinioDtr = atoi(tok);
                if (port1PinioDtr < 0 || port1PinioDtr > PINIO_COUNT) {
                    cliPrintLinef("Invalid PinIO number %d", port1PinioDtr);
                    return ;
                }
#endif /* USE_PINIO */
            }
            break;
        case 4:
            ports[1].id = atoi(tok);
            ports[1].port = NULL;
            break;
        case 5:
            ports[1].baud = atoi(tok);
            break;
        case 6:
            ports[1].mode = cliParseSerialMode(tok);
            break;
        }
        index++;
        tok = strtok_r(NULL, " ", &saveptr);
    }

    // Port checks
    if (ports[0].id == ports[1].id) {
        cliPrintLinef("Port1 and port2 are same");
        return ;
    }

    for (int i = 0; i < 2; i++) {
        if (findSerialPortIndexByIdentifier(ports[i].id) == -1) {
            cliPrintLinef("Invalid port%d %d", i + 1, ports[i].id);
            return ;
        } else {
            cliPrintLinef("Port%d: %d ", i + 1, ports[i].id);
        }
    }

    if (ports[0].baud == 0 && ports[1].id == SERIAL_PORT_USB_VCP) {
        enableBaudCb = true;
    }

    for (int i = 0; i < 2; i++) {
        serialPort_t **port = &(ports[i].port);
        if (*port != NULL) {
            continue;
        }

        int portIndex = i + 1;
        serialPortUsage_t *portUsage = findSerialPortUsageByIdentifier(ports[i].id);
        if (!portUsage || portUsage->serialPort == NULL) {
            bool isUseDefaultBaud = false;
            if (ports[i].baud == 0) {
                // Set default baud
                ports[i].baud = 57600;
                isUseDefaultBaud = true;
            }

            if (!ports[i].mode) {
                ports[i].mode = MODE_RXTX;
            }

            *port = openSerialPort(ports[i].id, FUNCTION_NONE, NULL, NULL,
                                            ports[i].baud, ports[i].mode,
                                            SERIAL_NOT_INVERTED);
            if (!*port) {
                cliPrintLinef("Port%d could not be opened.", portIndex);
                return;
            }

            if (isUseDefaultBaud) {
                cliPrintf("Port%d opened, default baud = %d.\r\n", portIndex, ports[i].baud);
            } else {
                cliPrintf("Port%d opened, baud = %d.\r\n", portIndex, ports[i].baud);
            }
        } else {
            *port = portUsage->serialPort;
            // If the user supplied a mode, override the port's mode, otherwise
            // leave the mode unchanged. serialPassthrough() handles one-way ports.
            // Set the baud rate if specified
            if (ports[i].baud) {
                cliPrintf("Port%d is already open, setting baud = %d.\r\n", portIndex, ports[i].baud);
                serialSetBaudRate(*port, ports[i].baud);
            } else {
                cliPrintf("Port%d is already open, baud = %d.\r\n", portIndex, (*port)->baudRate);
            }

            if (ports[i].mode && (*port)->mode != ports[i].mode) {
                cliPrintf("Port%d mode changed from %d to %d.\r\n",
                    portIndex, (*port)->mode, ports[i].mode);
                serialSetMode(*port, ports[i].mode);
            }

            // If this port has a rx callback associated we need to remove it now.
            // Otherwise no data will be pushed in the serial port buffer!
            if ((*port)->rxCallback) {
                (*port)->rxCallback = NULL;
            }
        }
    }

    // If no baud rate is specified allow to be set via USB
    if (enableBaudCb) {
        cliPrintLine("Port1 baud rate change over USB enabled.");
        // Register the right side baud rate setting routine with the left side which allows setting of the UART
        // baud rate over USB without setting it using the serialpassthrough command
        serialSetBaudRateCb(ports[0].port, serialSetBaudRate, ports[1].port);
    }

    char *resetMessage = "";
    if (port1ResetOnDtr && ports[1].id == SERIAL_PORT_USB_VCP) {
        resetMessage = "or drop DTR ";
    }

    cliPrintLinef("Forwarding, power cycle %sto exit.", resetMessage);

    if ((ports[1].id == SERIAL_PORT_USB_VCP) && (port1ResetOnDtr
#ifdef USE_PINIO
        || port1PinioDtr
#endif /* USE_PINIO */
        )) {
        // Register control line state callback
        serialSetCtrlLineStateCb(ports[0].port, cbCtrlLine, (void *)(intptr_t)(port1PinioDtr));
    }

// XXX Review ESC pass through under refactored motor handling
#ifdef USE_PWM_OUTPUT
    if (escSensorPassthrough) {
        // pwmDisableMotors();
        motorDisable();
        delay(5);
        unsigned motorsCount = getMotorCount();
        for (unsigned i = 0; i < motorsCount; i++) {
            const ioTag_t tag = motorConfig()->dev.ioTags[i];
            if (tag) {
                const timerHardware_t *timerHardware = timerGetByTag(tag);
                if (timerHardware) {
                    IO_t io = IOGetByTag(tag);
                    IOInit(io, OWNER_MOTOR, 0);
                    IOConfigGPIO(io, IOCFG_OUT_PP);
                    if (timerHardware->output & TIMER_OUTPUT_INVERTED) {
                        IOLo(io);
                    } else {
                        IOHi(io);
                    }
                }
            }
        }
    }
#endif

    serialPassthrough(ports[0].port, ports[1].port, NULL, NULL);
}
#endif

static void printAdjustmentRange(dumpFlags_t dumpMask, const adjustmentRange_t *adjustmentRanges, const adjustmentRange_t *defaultAdjustmentRanges, const char *headingStr)
{
    const char *format = "adjrange %u 0 %u %u %u %u %u %u %u";
    // print out adjustment ranges channel settings
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
        const adjustmentRange_t *ar = &adjustmentRanges[i];
        bool equalsDefault = false;
        if (defaultAdjustmentRanges) {
            const adjustmentRange_t *arDefault = &defaultAdjustmentRanges[i];
            equalsDefault = !memcmp(ar, arDefault, sizeof(*ar));
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                arDefault->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(arDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(arDefault->range.endStep),
                arDefault->adjustmentConfig,
                arDefault->auxSwitchChannelIndex,
                arDefault->adjustmentCenter,
                arDefault->adjustmentScale
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            ar->auxChannelIndex,
            MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
            ar->adjustmentConfig,
            ar->auxSwitchChannelIndex,
            ar->adjustmentCenter,
            ar->adjustmentScale
        );
    }
}

static void cliAdjustmentRange(const char *cmdName, char *cmdline)
{
    const char *format = "adjrange %u 0 %u %u %u %u %u %u %u";
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printAdjustmentRange(DUMP_MASTER, adjustmentRanges(0), NULL, NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = adjustmentRangesMutable(i);
            uint8_t validArgumentCount = 0;

            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                // Was: slot
                // Keeping the parameter to retain backwards compatibility for the command format.
                validArgumentCount++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }

            ptr = processChannelRangeArgs(ptr, &ar->range, &validArgumentCount);

            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < ADJUSTMENT_FUNCTION_COUNT) {
                    ar->adjustmentConfig = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxSwitchChannelIndex = val;
                    validArgumentCount++;
                }
            }

            if (validArgumentCount != 6) {
                memset(ar, 0, sizeof(adjustmentRange_t));
                cliShowInvalidArgumentCountError(cmdName);
                return;
            }

            // Optional arguments
            ar->adjustmentCenter = 0;
            ar->adjustmentScale = 0;

            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                ar->adjustmentCenter = val;
                validArgumentCount++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                ar->adjustmentScale = val;
                validArgumentCount++;
            }

            activeAdjustmentRangeReset();

            cliDumpPrintLinef(0, false, format,
                i,
                ar->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
                ar->adjustmentConfig,
                ar->auxSwitchChannelIndex,
                ar->adjustmentCenter,
                ar->adjustmentScale
            );

        } else {
            cliShowArgumentRangeError(cmdName, "INDEX", 0, MAX_ADJUSTMENT_RANGE_COUNT - 1);
        }
    }
}

#ifndef USE_QUAD_MIXER_ONLY
static void printMotorMix(dumpFlags_t dumpMask, const motorMixer_t *customMotorMixer, const motorMixer_t *defaultCustomMotorMixer, const char *headingStr)
{
    const char *format = "mmix %d %s %s %s %s";
    char buf0[FTOA_BUFFER_LENGTH];
    char buf1[FTOA_BUFFER_LENGTH];
    char buf2[FTOA_BUFFER_LENGTH];
    char buf3[FTOA_BUFFER_LENGTH];
    for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (customMotorMixer[i].throttle == 0.0f)
            break;
        const float thr = customMotorMixer[i].throttle;
        const float roll = customMotorMixer[i].roll;
        const float pitch = customMotorMixer[i].pitch;
        const float yaw = customMotorMixer[i].yaw;
        bool equalsDefault = false;
        if (defaultCustomMotorMixer) {
            const float thrDefault = defaultCustomMotorMixer[i].throttle;
            const float rollDefault = defaultCustomMotorMixer[i].roll;
            const float pitchDefault = defaultCustomMotorMixer[i].pitch;
            const float yawDefault = defaultCustomMotorMixer[i].yaw;
            const bool equalsDefault = thr == thrDefault && roll == rollDefault && pitch == pitchDefault && yaw == yawDefault;

            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                ftoa(thrDefault, buf0),
                ftoa(rollDefault, buf1),
                ftoa(pitchDefault, buf2),
                ftoa(yawDefault, buf3));
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            ftoa(thr, buf0),
            ftoa(roll, buf1),
            ftoa(pitch, buf2),
            ftoa(yaw, buf3));
    }
}
#endif // USE_QUAD_MIXER_ONLY

static void cliMotorMix(const char *cmdName, char *cmdline)
{
#ifdef USE_QUAD_MIXER_ONLY
    UNUSED(cmdName);
    UNUSED(cmdline);
#else
    int check = 0;
    uint8_t len;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printMotorMix(DUMP_MASTER, customMotorMixer(0), NULL, NULL);
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            customMotorMixerMutable(i)->throttle = 0.0f;
        }
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = nextArg(cmdline);
        if (ptr) {
            len = strlen(ptr);
            for (uint32_t i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrintErrorLinef(cmdName, "INVALID NAME");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i, customMotorMixerMutable(0));
                    cliPrintLinef("Loaded %s", mixerNames[i]);
                    cliMotorMix(cmdName, "");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        uint32_t i = atoi(ptr); // get motor number
        if (i < MAX_SUPPORTED_MOTORS) {
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->throttle = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->roll = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->pitch = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->yaw = fastA2F(ptr);
                check++;
            }
            if (check != 4) {
                cliShowInvalidArgumentCountError(cmdName);
            } else {
                printMotorMix(DUMP_MASTER, customMotorMixer(0), NULL, NULL);
            }
        } else {
            cliShowArgumentRangeError(cmdName, "INDEX", 0, MAX_SUPPORTED_MOTORS - 1);
        }
    }
#endif
}

static void printRxRange(dumpFlags_t dumpMask, const rxChannelRangeConfig_t *channelRangeConfigs, const rxChannelRangeConfig_t *defaultChannelRangeConfigs, const char *headingStr)
{
    const char *format = "rxrange %u %u %u";
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        bool equalsDefault = false;
        if (defaultChannelRangeConfigs) {
            equalsDefault = !memcmp(&channelRangeConfigs[i], &defaultChannelRangeConfigs[i], sizeof(channelRangeConfigs[i]));
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                defaultChannelRangeConfigs[i].min,
                defaultChannelRangeConfigs[i].max
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            channelRangeConfigs[i].min,
            channelRangeConfigs[i].max
        );
    }
}

static void cliRxRange(const char *cmdName, char *cmdline)
{
    const char *format = "rxrange %u %u %u";
    int i, validArgumentCount = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printRxRange(DUMP_MASTER, rxChannelRangeConfigs(0), NULL, NULL);
    } else if (strcasecmp(cmdline, "reset") == 0) {
        resetAllRxChannelRangeConfigurations(rxChannelRangeConfigsMutable(0));
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i >= 0 && i < NON_AUX_CHANNEL_COUNT) {
            int rangeMin = PWM_PULSE_MIN, rangeMax = PWM_PULSE_MAX;

            ptr = nextArg(ptr);
            if (ptr) {
                rangeMin = atoi(ptr);
                validArgumentCount++;
            }

            ptr = nextArg(ptr);
            if (ptr) {
                rangeMax = atoi(ptr);
                validArgumentCount++;
            }

            if (validArgumentCount != 2) {
                cliShowInvalidArgumentCountError(cmdName);
            } else if (rangeMin < PWM_PULSE_MIN || rangeMin > PWM_PULSE_MAX || rangeMax < PWM_PULSE_MIN || rangeMax > PWM_PULSE_MAX) {
                cliShowArgumentRangeError(cmdName, "range min/max", PWM_PULSE_MIN, PWM_PULSE_MAX);
            } else {
                rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(i);
                channelRangeConfig->min = rangeMin;
                channelRangeConfig->max = rangeMax;
                cliDumpPrintLinef(0, false, format,
                    i,
                    channelRangeConfig->min,
                    channelRangeConfig->max
                );

            }
        } else {
            cliShowArgumentRangeError(cmdName, "CHANNEL", 0, NON_AUX_CHANNEL_COUNT - 1);
        }
    }
}

#ifdef USE_LED_STRIP_STATUS_MODE
static void printLed(dumpFlags_t dumpMask, const ledConfig_t *ledConfigs, const ledConfig_t *defaultLedConfigs, const char *headingStr)
{
    const char *format = "led %u %s";
    char ledConfigBuffer[20];
    char ledConfigDefaultBuffer[20];
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
        ledConfig_t ledConfig = ledConfigs[i];
        generateLedConfig(&ledConfig, ledConfigBuffer, sizeof(ledConfigBuffer));
        bool equalsDefault = false;
        if (defaultLedConfigs) {
            ledConfig_t ledConfigDefault = defaultLedConfigs[i];
            equalsDefault = ledConfig == ledConfigDefault;
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            generateLedConfig(&ledConfigDefault, ledConfigDefaultBuffer, sizeof(ledConfigDefaultBuffer));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i, ledConfigDefaultBuffer);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i, ledConfigBuffer);
    }
}

static void cliLed(const char *cmdName, char *cmdline)
{
    const char *format = "led %u %s";
    char ledConfigBuffer[20];
    int i;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printLed(DUMP_MASTER, ledStripStatusModeConfig()->ledConfigs, NULL, NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i >= 0 && i < LED_MAX_STRIP_LENGTH) {
            ptr = nextArg(cmdline);
            if (parseLedStripConfig(i, ptr)) {
                generateLedConfig((ledConfig_t *)&ledStripStatusModeConfig()->ledConfigs[i], ledConfigBuffer, sizeof(ledConfigBuffer));
                cliDumpPrintLinef(0, false, format, i, ledConfigBuffer);
            } else {
                cliShowParseError(cmdName);
            }
        } else {
            cliShowArgumentRangeError(cmdName, "INDEX", 0, LED_MAX_STRIP_LENGTH - 1);
        }
    }
}

static void printColor(dumpFlags_t dumpMask, const hsvColor_t *colors, const hsvColor_t *defaultColors, const char *headingStr)
{
    const char *format = "color %u %d,%u,%u";
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
        const hsvColor_t *color = &colors[i];
        bool equalsDefault = false;
        if (defaultColors) {
            const hsvColor_t *colorDefault = &defaultColors[i];
            equalsDefault = !memcmp(color, colorDefault, sizeof(*color));
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,colorDefault->h, colorDefault->s, colorDefault->v);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i, color->h, color->s, color->v);
    }
}

static void cliColor(const char *cmdName, char *cmdline)
{
    const char *format = "color %u %d,%u,%u";
    if (isEmpty(cmdline)) {
        printColor(DUMP_MASTER, ledStripStatusModeConfig()->colors, NULL, NULL);
    } else {
        const char *ptr = cmdline;
        const int i = atoi(ptr);
        if (i < LED_CONFIGURABLE_COLOR_COUNT) {
            ptr = nextArg(cmdline);
            if (parseColor(i, ptr)) {
                const hsvColor_t *color = &ledStripStatusModeConfig()->colors[i];
                cliDumpPrintLinef(0, false, format, i, color->h, color->s, color->v);
            } else {
                cliShowParseError(cmdName);
            }
        } else {
            cliShowArgumentRangeError(cmdName, "INDEX", 0, LED_CONFIGURABLE_COLOR_COUNT - 1);
        }
    }
}

static void printModeColor(dumpFlags_t dumpMask, const ledStripStatusModeConfig_t *ledStripStatusModeConfig, const ledStripStatusModeConfig_t *defaultLedStripConfig, const char *headingStr)
{
    const char *format = "mode_color %u %u %u";
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < LED_MODE_COUNT; i++) {
        for (uint32_t j = 0; j < LED_DIRECTION_COUNT; j++) {
            int colorIndex = ledStripStatusModeConfig->modeColors[i].color[j];
            bool equalsDefault = false;
            if (defaultLedStripConfig) {
                int colorIndexDefault = defaultLedStripConfig->modeColors[i].color[j];
                equalsDefault = colorIndex == colorIndexDefault;
                headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, i, j, colorIndexDefault);
            }
            cliDumpPrintLinef(dumpMask, equalsDefault, format, i, j, colorIndex);
        }
    }

    for (uint32_t j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
        const int colorIndex = ledStripStatusModeConfig->specialColors.color[j];
        bool equalsDefault = false;
        if (defaultLedStripConfig) {
            const int colorIndexDefault = defaultLedStripConfig->specialColors.color[j];
            equalsDefault = colorIndex == colorIndexDefault;
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndexDefault);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndex);
    }

    const int ledStripAuxChannel = ledStripStatusModeConfig->ledstrip_aux_channel;
    bool equalsDefault = false;
    if (defaultLedStripConfig) {
        const int ledStripAuxChannelDefault = defaultLedStripConfig->ledstrip_aux_channel;
        equalsDefault = ledStripAuxChannel == ledStripAuxChannelDefault;
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        cliDefaultPrintLinef(dumpMask, equalsDefault, format, LED_AUX_CHANNEL, 0, ledStripAuxChannelDefault);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault, format, LED_AUX_CHANNEL, 0, ledStripAuxChannel);
}

static void cliModeColor(const char *cmdName, char *cmdline)
{
    if (isEmpty(cmdline)) {
        printModeColor(DUMP_MASTER, ledStripStatusModeConfig(), NULL, NULL);
    } else {
        enum {MODE = 0, FUNCTION, COLOR, ARGS_COUNT};
        int args[ARGS_COUNT];
        int argNo = 0;
        char *saveptr;
        const char* ptr = strtok_r(cmdline, " ", &saveptr);
        while (ptr && argNo < ARGS_COUNT) {
            args[argNo++] = atoi(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || argNo != ARGS_COUNT) {
            cliShowInvalidArgumentCountError(cmdName);
            return;
        }

        int modeIdx  = args[MODE];
        int funIdx = args[FUNCTION];
        int color = args[COLOR];
        if (!setModeColor(modeIdx, funIdx, color)) {
            cliShowParseError(cmdName);
            return;
        }
        // values are validated
        cliPrintLinef("mode_color %u %u %u", modeIdx, funIdx, color);
    }
}
#endif

#ifdef USE_SERVOS
static void printServo(dumpFlags_t dumpMask, const servoParam_t *servoParams, const servoParam_t *defaultServoParams, const char *headingStr)
{
    // print out servo settings
    const char *format = "servo %u %d %d %d %d %d";
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const servoParam_t *servoConf = &servoParams[i];
        bool equalsDefault = false;
        if (defaultServoParams) {
            const servoParam_t *defaultServoConf = &defaultServoParams[i];
            equalsDefault = !memcmp(servoConf, defaultServoConf, sizeof(*servoConf));
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                defaultServoConf->min,
                defaultServoConf->max,
                defaultServoConf->middle,
                defaultServoConf->rate,
                defaultServoConf->forwardFromChannel
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            servoConf->min,
            servoConf->max,
            servoConf->middle,
            servoConf->rate,
            servoConf->forwardFromChannel
        );
    }
    // print servo directions
    for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const char *format = "smix reverse %d %d r";
        const servoParam_t *servoConf = &servoParams[i];
        const servoParam_t *servoConfDefault = &defaultServoParams[i];
        if (defaultServoParams) {
            bool equalsDefault = servoConf->reversedSources == servoConfDefault->reversedSources;
            for (uint32_t channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                equalsDefault = ~(servoConf->reversedSources ^ servoConfDefault->reversedSources) & (1 << channel);
                if (servoConfDefault->reversedSources & (1 << channel)) {
                    cliDefaultPrintLinef(dumpMask, equalsDefault, format, i , channel);
                }
                if (servoConf->reversedSources & (1 << channel)) {
                    cliDumpPrintLinef(dumpMask, equalsDefault, format, i , channel);
                }
            }
        } else {
            for (uint32_t channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                if (servoConf->reversedSources & (1 << channel)) {
                    cliDumpPrintLinef(dumpMask, true, format, i , channel);
                }
            }
        }
    }
}

static void cliServo(const char *cmdName, char *cmdline)
{
    const char *format = "servo %u %d %d %d %d %d";
    enum { SERVO_ARGUMENT_COUNT = 6 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    servoParam_t *servo;

    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        printServo(DUMP_MASTER, servoParams(0), NULL, NULL);
    } else {
        int validArgumentCount = 0;

        ptr = cmdline;

        // Command line is integers (possibly negative) separated by spaces, no other characters allowed.

        // If command line doesn't fit the format, don't modify the config
        while (*ptr) {
            if (*ptr == '-' || (*ptr >= '0' && *ptr <= '9')) {
                if (validArgumentCount >= SERVO_ARGUMENT_COUNT) {
                    cliShowInvalidArgumentCountError(cmdName);
                    return;
                }

                arguments[validArgumentCount++] = atoi(ptr);

                do {
                    ptr++;
                } while (*ptr >= '0' && *ptr <= '9');
            } else if (*ptr == ' ') {
                ptr++;
            } else {
                cliShowParseError(cmdName);
                return;
            }
        }

        enum {INDEX = 0, MIN, MAX, MIDDLE, RATE, FORWARD};

        i = arguments[INDEX];

        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if (validArgumentCount != SERVO_ARGUMENT_COUNT || i < 0 || i >= MAX_SUPPORTED_SERVOS) {
            cliShowInvalidArgumentCountError(cmdName);
            return;
        }

        servo = servoParamsMutable(i);

        if (
            arguments[MIN] < PWM_PULSE_MIN || arguments[MIN] > PWM_PULSE_MAX ||
            arguments[MAX] < PWM_PULSE_MIN || arguments[MAX] > PWM_PULSE_MAX ||
            arguments[MIDDLE] < arguments[MIN] || arguments[MIDDLE] > arguments[MAX] ||
            arguments[MIN] > arguments[MAX] || arguments[MAX] < arguments[MIN] ||
            arguments[RATE] < -100 || arguments[RATE] > 100 ||
            arguments[FORWARD] >= MAX_SUPPORTED_RC_CHANNEL_COUNT
        ) {
            cliShowArgumentRangeError(cmdName, NULL, 0, 0);
            return;
        }

        servo->min = arguments[MIN];
        servo->max = arguments[MAX];
        servo->middle = arguments[MIDDLE];
        servo->rate = arguments[RATE];
        servo->forwardFromChannel = arguments[FORWARD];

        cliDumpPrintLinef(0, false, format,
            i,
            servo->min,
            servo->max,
            servo->middle,
            servo->rate,
            servo->forwardFromChannel
        );

    }
}
#endif

#ifdef USE_SERVOS
static void printServoMix(dumpFlags_t dumpMask, const servoMixer_t *customServoMixers, const servoMixer_t *defaultCustomServoMixers, const char *headingStr)
{
    const char *format = "smix %d %d %d %d %d %d %d %d";
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; i < MAX_SERVO_RULES; i++) {
        const servoMixer_t customServoMixer = customServoMixers[i];
        if (customServoMixer.rate == 0) {
            break;
        }

        bool equalsDefault = false;
        if (defaultCustomServoMixers) {
            servoMixer_t customServoMixerDefault = defaultCustomServoMixers[i];
            equalsDefault = !memcmp(&customServoMixer, &customServoMixerDefault, sizeof(customServoMixer));

            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                customServoMixerDefault.targetChannel,
                customServoMixerDefault.inputSource,
                customServoMixerDefault.rate,
                customServoMixerDefault.speed,
                customServoMixerDefault.min,
                customServoMixerDefault.max,
                customServoMixerDefault.box
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            customServoMixer.targetChannel,
            customServoMixer.inputSource,
            customServoMixer.rate,
            customServoMixer.speed,
            customServoMixer.min,
            customServoMixer.max,
            customServoMixer.box
        );
    }
}

static void cliServoMix(const char *cmdName, char *cmdline)
{
    int args[8], check = 0;
    int len = strlen(cmdline);

    if (len == 0) {
        printServoMix(DUMP_MASTER, customServoMixers(0), NULL, NULL);
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        memset(customServoMixers_array(), 0, sizeof(*customServoMixers_array()));
        for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoParamsMutable(i)->reversedSources = 0;
        }
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        const char *ptr = nextArg(cmdline);
        if (ptr) {
            len = strlen(ptr);
            for (uint32_t i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrintErrorLinef(cmdName, "INVALID NAME");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    servoMixerLoadMix(i);
                    cliPrintLinef("Loaded %s", mixerNames[i]);
                    cliServoMix(cmdName, "");
                    break;
                }
            }
        }
    } else if (strncasecmp(cmdline, "reverse", 7) == 0) {
        enum {SERVO = 0, INPUT, REVERSE, ARGS_COUNT};
        char *ptr = strchr(cmdline, ' ');

        if (ptr == NULL) {
            cliPrintf("s");
            for (uint32_t inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                cliPrintf("\ti%d", inputSource);
            cliPrintLinefeed();

            for (uint32_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
                cliPrintf("%d", servoIndex);
                for (uint32_t inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++) {
                    cliPrintf("\t%s  ", (servoParams(servoIndex)->reversedSources & (1 << inputSource)) ? "r" : "n");
                }
                cliPrintLinefeed();
            }
            return;
        }

        char *saveptr;
        ptr = strtok_r(ptr, " ", &saveptr);
        while (ptr != NULL && check < ARGS_COUNT - 1) {
            args[check++] = atoi(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr == NULL || check != ARGS_COUNT - 1) {
            cliShowInvalidArgumentCountError(cmdName);
            return;
        }

        if (args[SERVO] >= 0 && args[SERVO] < MAX_SUPPORTED_SERVOS
                && args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT
                && (*ptr == 'r' || *ptr == 'n')) {
            if (*ptr == 'r') {
                servoParamsMutable(args[SERVO])->reversedSources |= 1 << args[INPUT];
            } else {
                servoParamsMutable(args[SERVO])->reversedSources &= ~(1 << args[INPUT]);
            }
        } else {
            cliShowArgumentRangeError(cmdName, "servo", 0, MAX_SUPPORTED_SERVOS);
            return;
        }

        cliServoMix(cmdName, "reverse");
    } else {
        enum {RULE = 0, TARGET, INPUT, RATE, SPEED, MIN, MAX, BOX, ARGS_COUNT};
        char *saveptr;
        char *ptr = strtok_r(cmdline, " ", &saveptr);
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = atoi(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || check != ARGS_COUNT) {
            cliShowInvalidArgumentCountError(cmdName);
            return;
        }

        int32_t i = args[RULE];
        if (i >= 0 && i < MAX_SERVO_RULES &&
            args[TARGET] >= 0 && args[TARGET] < MAX_SUPPORTED_SERVOS &&
            args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT &&
            args[RATE] >= -100 && args[RATE] <= 100 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED &&
            args[MIN] >= 0 && args[MIN] <= 100 &&
            args[MAX] >= 0 && args[MAX] <= 100 && args[MIN] < args[MAX] &&
            args[BOX] >= 0 && args[BOX] <= MAX_SERVO_BOXES) {
            customServoMixersMutable(i)->targetChannel = args[TARGET];
            customServoMixersMutable(i)->inputSource = args[INPUT];
            customServoMixersMutable(i)->rate = args[RATE];
            customServoMixersMutable(i)->speed = args[SPEED];
            customServoMixersMutable(i)->min = args[MIN];
            customServoMixersMutable(i)->max = args[MAX];
            customServoMixersMutable(i)->box = args[BOX];
            cliServoMix(cmdName, "");
        } else {
            cliShowArgumentRangeError(cmdName, NULL, 0, 0);
        }
    }
}
#endif

#ifdef USE_SDCARD

static void cliWriteBytes(const uint8_t *buffer, int count)
{
    while (count > 0) {
        cliWrite(*buffer);
        buffer++;
        count--;
    }
}

static void cliSdInfo(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    cliPrint("SD card: ");

    if (sdcardConfig()->mode == SDCARD_MODE_NONE) {
        cliPrintLine("Not configured");

        return;
    }

    if (!sdcard_isInserted()) {
        cliPrintLine("None inserted");
        return;
    }

    if (!sdcard_isFunctional() || !sdcard_isInitialized()) {
        cliPrintLine("Startup failed");
        return;
    }

    const sdcardMetadata_t *metadata = sdcard_getMetadata();

    cliPrintf("Manufacturer 0x%x, %ukB, %02d/%04d, v%d.%d, '",
        metadata->manufacturerID,
        metadata->numBlocks / 2, /* One block is half a kB */
        metadata->productionMonth,
        metadata->productionYear,
        metadata->productRevisionMajor,
        metadata->productRevisionMinor
    );

    cliWriteBytes((uint8_t*)metadata->productName, sizeof(metadata->productName));

    cliPrint("'\r\n" "Filesystem: ");

    switch (afatfs_getFilesystemState()) {
    case AFATFS_FILESYSTEM_STATE_READY:
        cliPrint("Ready");
        break;
    case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
        cliPrint("Initializing");
        break;
    case AFATFS_FILESYSTEM_STATE_UNKNOWN:
    case AFATFS_FILESYSTEM_STATE_FATAL:
        cliPrint("Fatal");

        switch (afatfs_getLastError()) {
        case AFATFS_ERROR_BAD_MBR:
            cliPrint(" - no FAT MBR partitions");
            break;
        case AFATFS_ERROR_BAD_FILESYSTEM_HEADER:
            cliPrint(" - bad FAT header");
            break;
        case AFATFS_ERROR_GENERIC:
        case AFATFS_ERROR_NONE:
            ; // Nothing more detailed to print
            break;
        }
        break;
    }
    cliPrintLinefeed();
}

#endif

#ifdef USE_FLASH_CHIP

static void cliFlashInfo(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    const flashGeometry_t *layout = flashGetGeometry();

    cliPrintLinef("Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u",
            layout->sectors, layout->sectorSize, layout->pagesPerSector, layout->pageSize, layout->totalSize);

    for (uint8_t index = 0; index < FLASH_MAX_PARTITIONS; index++) {
        const flashPartition_t *partition;
        if (index == 0) {
            cliPrintLine("Paritions:");
        }
        partition = flashPartitionFindByIndex(index);
        if (!partition) {
            break;
        }
        cliPrintLinef("  %d: %s %u %u", index, flashPartitionGetTypeName(partition->type), partition->startSector, partition->endSector);
    }
#ifdef USE_FLASHFS
    const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);

    cliPrintLinef("FlashFS size=%u, usedSize=%u",
            FLASH_PARTITION_SECTOR_COUNT(flashPartition) * layout->sectorSize,
            flashfsGetOffset()
    );
#endif
}


static void cliFlashErase(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    if (!flashfsIsSupported()) {
        return;
    }

#ifndef MINIMAL_CLI
    uint32_t i = 0;
    cliPrintLine("Erasing, please wait ... ");
#else
    cliPrintLine("Erasing,");
#endif

    cliWriterFlush();
    flashfsEraseCompletely();

    while (!flashfsIsReady()) {
#ifndef MINIMAL_CLI
        cliPrintf(".");
        if (i++ > 120) {
            i=0;
            cliPrintLinefeed();
        }

        cliWriterFlush();
#endif
        delay(100);
    }
    beeper(BEEPER_BLACKBOX_ERASE);
    cliPrintLinefeed();
    cliPrintLine("Done.");
}

#ifdef USE_FLASH_TOOLS

static void cliFlashVerify(const char *cmdName, char *cmdline)
{
    UNUSED(cmdline);

    cliPrintLine("Verifying");
    if (flashfsVerifyEntireFlash()) {
        cliPrintLine("Success");
    } else {
        cliPrintErrorLinef(cmdName, "Failed");
    }
}

static void cliFlashWrite(const char *cmdName, char *cmdline)
{
    const uint32_t address = atoi(cmdline);
    const char *text = strchr(cmdline, ' ');

    if (!text) {
        cliShowInvalidArgumentCountError(cmdName);
    } else {
        flashfsSeekAbs(address);
        flashfsWrite((uint8_t*)text, strlen(text), true);
        flashfsFlushSync();

        cliPrintLinef("Wrote %u bytes at %u.", strlen(text), address);
    }
}

static void cliFlashRead(const char *cmdName, char *cmdline)
{
    uint32_t address = atoi(cmdline);

    const char *nextArg = strchr(cmdline, ' ');

    if (!nextArg) {
        cliShowInvalidArgumentCountError(cmdName);
    } else {
        uint32_t length = atoi(nextArg);

        cliPrintLinef("Reading %u bytes at %u:", length, address);

        uint8_t buffer[32];
        while (length > 0) {
            int bytesRead = flashfsReadAbs(address, buffer, length < sizeof(buffer) ? length : sizeof(buffer));

            for (int i = 0; i < bytesRead; i++) {
                cliWrite(buffer[i]);
            }

            length -= bytesRead;
            address += bytesRead;

            if (bytesRead == 0) {
                //Assume we reached the end of the volume or something fatal happened
                break;
            }
        }
        cliPrintLinefeed();
    }
}

#endif
#endif

#ifdef USE_VTX_CONTROL
static void printVtx(dumpFlags_t dumpMask, const vtxConfig_t *vtxConfig, const vtxConfig_t *vtxConfigDefault, const char *headingStr)
{
    // print out vtx channel settings
    const char *format = "vtx %u %u %u %u %u %u %u";
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    bool equalsDefault = false;
    for (uint32_t i = 0; i < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; i++) {
        const vtxChannelActivationCondition_t *cac = &vtxConfig->vtxChannelActivationConditions[i];
        if (vtxConfigDefault) {
            const vtxChannelActivationCondition_t *cacDefault = &vtxConfigDefault->vtxChannelActivationConditions[i];
            equalsDefault = !memcmp(cac, cacDefault, sizeof(*cac));
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                cacDefault->auxChannelIndex,
                cacDefault->band,
                cacDefault->channel,
                cacDefault->power,
                MODE_STEP_TO_CHANNEL_VALUE(cacDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(cacDefault->range.endStep)
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            cac->auxChannelIndex,
            cac->band,
            cac->channel,
            cac->power,
            MODE_STEP_TO_CHANNEL_VALUE(cac->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(cac->range.endStep)
        );
    }
}

static void cliVtx(const char *cmdName, char *cmdline)
{
    const char *format = "vtx %u %u %u %u %u %u %u";
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printVtx(DUMP_MASTER, vtxConfig(), NULL, NULL);
    } else {
#ifdef USE_VTX_TABLE
        const uint8_t maxBandIndex = vtxTableConfig()->bands;
        const uint8_t maxChannelIndex = vtxTableConfig()->channels;
        const uint8_t maxPowerIndex = vtxTableConfig()->powerLevels;
#else
        const uint8_t maxBandIndex = VTX_TABLE_MAX_BANDS;
        const uint8_t maxChannelIndex = VTX_TABLE_MAX_CHANNELS;
        const uint8_t maxPowerIndex = VTX_TABLE_MAX_POWER_LEVELS;
#endif
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT) {
            vtxChannelActivationCondition_t *cac = &vtxConfigMutable()->vtxChannelActivationConditions[i];
            uint8_t validArgumentCount = 0;
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    cac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val <= maxBandIndex) {
                    cac->band = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val <= maxChannelIndex) {
                    cac->channel = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val <= maxPowerIndex) {
                    cac->power= val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &cac->range, &validArgumentCount);

            if (validArgumentCount != 6) {
                memset(cac, 0, sizeof(vtxChannelActivationCondition_t));
                cliShowInvalidArgumentCountError(cmdName);
            } else {
                cliDumpPrintLinef(0, false, format,
                    i,
                    cac->auxChannelIndex,
                    cac->band,
                    cac->channel,
                    cac->power,
                    MODE_STEP_TO_CHANNEL_VALUE(cac->range.startStep),
                    MODE_STEP_TO_CHANNEL_VALUE(cac->range.endStep)
                );
            }
        } else {
            cliShowArgumentRangeError(cmdName, "INDEX", 0, MAX_CHANNEL_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}

#endif // VTX_CONTROL

#ifdef USE_VTX_TABLE

static char *formatVtxTableBandFrequency(const bool isFactory, const uint16_t *frequency, int channels)
{
    static char freqbuf[5 * VTX_TABLE_MAX_CHANNELS + 8 + 1];
    char freqtmp[5 + 1];
    freqbuf[0] = 0;
    strcat(freqbuf, isFactory ? " FACTORY" : " CUSTOM ");
    for (int channel = 0; channel < channels; channel++) {
        tfp_sprintf(freqtmp, " %4d", frequency[channel]);
        strcat(freqbuf, freqtmp);
    }
    return freqbuf;
}

static const char *printVtxTableBand(dumpFlags_t dumpMask, int band, const vtxTableConfig_t *currentConfig, const vtxTableConfig_t *defaultConfig, const char *headingStr)
{
    char *fmt = "vtxtable band %d %s %c%s";
    bool equalsDefault = false;

    if (defaultConfig) {
        equalsDefault = true;
        if (strcasecmp(currentConfig->bandNames[band], defaultConfig->bandNames[band])) {
            equalsDefault = false;
        }
        if (currentConfig->bandLetters[band] != defaultConfig->bandLetters[band]) {
            equalsDefault = false;
        }
        for (int channel = 0; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
            if (currentConfig->frequency[band][channel] != defaultConfig->frequency[band][channel]) {
                equalsDefault = false;
              }
        }
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        char *freqbuf = formatVtxTableBandFrequency(defaultConfig->isFactoryBand[band], defaultConfig->frequency[band], defaultConfig->channels);
        cliDefaultPrintLinef(dumpMask, equalsDefault, fmt, band + 1, defaultConfig->bandNames[band], defaultConfig->bandLetters[band], freqbuf);
    }

    char *freqbuf = formatVtxTableBandFrequency(currentConfig->isFactoryBand[band], currentConfig->frequency[band], currentConfig->channels);
    cliDumpPrintLinef(dumpMask, equalsDefault, fmt, band + 1, currentConfig->bandNames[band], currentConfig->bandLetters[band], freqbuf);
    return headingStr;
}

static char *formatVtxTablePowerValues(const uint16_t *levels, int count)
{
    // (max 4 digit + 1 space) per level
    static char pwrbuf[5 * VTX_TABLE_MAX_POWER_LEVELS + 1];
    char pwrtmp[5 + 1];
    pwrbuf[0] = 0;
    for (int pwrindex = 0; pwrindex < count; pwrindex++) {
        tfp_sprintf(pwrtmp, " %d", levels[pwrindex]);
        strcat(pwrbuf, pwrtmp);
    }
    return pwrbuf;
}

static const char *printVtxTablePowerValues(dumpFlags_t dumpMask, const vtxTableConfig_t *currentConfig, const vtxTableConfig_t *defaultConfig, const char *headingStr)
{
    char *fmt = "vtxtable powervalues%s";
    bool equalsDefault = false;
    if (defaultConfig) {
        equalsDefault = true;
        for (int pwrindex = 0; pwrindex < VTX_TABLE_MAX_POWER_LEVELS; pwrindex++) {
            if (defaultConfig->powerValues[pwrindex] != currentConfig->powerValues[pwrindex]) {
                equalsDefault = false;
            }
        }
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        char *pwrbuf = formatVtxTablePowerValues(defaultConfig->powerValues, VTX_TABLE_MAX_POWER_LEVELS);
        cliDefaultPrintLinef(dumpMask, equalsDefault, fmt, pwrbuf);
    }

    char *pwrbuf = formatVtxTablePowerValues(currentConfig->powerValues, currentConfig->powerLevels);
    cliDumpPrintLinef(dumpMask, equalsDefault, fmt, pwrbuf);
    return headingStr;
}

static char *formatVtxTablePowerLabels(const char labels[VTX_TABLE_MAX_POWER_LEVELS][VTX_TABLE_POWER_LABEL_LENGTH + 1], int count)
{
    static char pwrbuf[(VTX_TABLE_POWER_LABEL_LENGTH + 1) * VTX_TABLE_MAX_POWER_LEVELS + 1];
    char pwrtmp[(VTX_TABLE_POWER_LABEL_LENGTH + 1) + 1];
    pwrbuf[0] = 0;
    for (int pwrindex = 0; pwrindex < count; pwrindex++) {
        strcat(pwrbuf, " ");
        strcpy(pwrtmp, labels[pwrindex]);
        // trim trailing space
        char *sp;
        while ((sp = strchr(pwrtmp, ' '))) {
            *sp = 0;
        }
        strcat(pwrbuf, pwrtmp);
    }
    return pwrbuf;
}

static const char *printVtxTablePowerLabels(dumpFlags_t dumpMask, const vtxTableConfig_t *currentConfig, const vtxTableConfig_t *defaultConfig, const char *headingStr)
{
    char *fmt = "vtxtable powerlabels%s";
    bool equalsDefault = false;
    if (defaultConfig) {
        equalsDefault = true;
        for (int pwrindex = 0; pwrindex < VTX_TABLE_MAX_POWER_LEVELS; pwrindex++) {
            if (strcasecmp(defaultConfig->powerLabels[pwrindex], currentConfig->powerLabels[pwrindex])) {
                equalsDefault = false;
            }
        }
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        char *pwrbuf = formatVtxTablePowerLabels(defaultConfig->powerLabels, VTX_TABLE_MAX_POWER_LEVELS);
        cliDefaultPrintLinef(dumpMask, equalsDefault, fmt, pwrbuf);
    }

    char *pwrbuf = formatVtxTablePowerLabels(currentConfig->powerLabels, currentConfig->powerLevels);
    cliDumpPrintLinef(dumpMask, equalsDefault, fmt, pwrbuf);
    return headingStr;
}

static void printVtxTable(dumpFlags_t dumpMask, const vtxTableConfig_t *currentConfig, const vtxTableConfig_t *defaultConfig, const char *headingStr)
{
    bool equalsDefault;
    char *fmt;

    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);

    // bands
    equalsDefault = false;
    fmt = "vtxtable bands %d";
    if (defaultConfig) {
        equalsDefault = (defaultConfig->bands == currentConfig->bands);
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        cliDefaultPrintLinef(dumpMask, equalsDefault, fmt, defaultConfig->bands);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault, fmt, currentConfig->bands);

    // channels
    equalsDefault = false;
    fmt = "vtxtable channels %d";
    if (defaultConfig) {
        equalsDefault = (defaultConfig->channels == currentConfig->channels);
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        cliDefaultPrintLinef(dumpMask, equalsDefault, fmt, defaultConfig->channels);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault, fmt, currentConfig->channels);

    // band

    for (int band = 0; band < currentConfig->bands; band++) {
        headingStr = printVtxTableBand(dumpMask, band, currentConfig, defaultConfig, headingStr);
    }

    // powerlevels

    equalsDefault = false;
    fmt = "vtxtable powerlevels %d";
    if (defaultConfig) {
        equalsDefault = (defaultConfig->powerLevels == currentConfig->powerLevels);
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        cliDefaultPrintLinef(dumpMask, equalsDefault, fmt, defaultConfig->powerLevels);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault, fmt, currentConfig->powerLevels);

    // powervalues

    // powerlabels
    headingStr = printVtxTablePowerValues(dumpMask, currentConfig, defaultConfig, headingStr);
    headingStr = printVtxTablePowerLabels(dumpMask, currentConfig, defaultConfig, headingStr);
}

static void cliVtxTable(const char *cmdName, char *cmdline)
{
    char *tok;
    char *saveptr;

    // Band number or nothing
    tok  = strtok_r(cmdline, " ", &saveptr);

    if (!tok) {
        printVtxTable(DUMP_MASTER | HIDE_UNUSED, vtxTableConfigMutable(), NULL, NULL);
        return;
    }

    if (strcasecmp(tok, "bands") == 0) {
        tok = strtok_r(NULL, " ", &saveptr);
        int bands = atoi(tok);
        if (bands < 0 || bands > VTX_TABLE_MAX_BANDS) {
            cliShowArgumentRangeError(cmdName, "BAND COUNT", 0, VTX_TABLE_MAX_BANDS);
            return;
        }
        if (bands < vtxTableConfigMutable()->bands) {
            for (int i = bands; i < vtxTableConfigMutable()->bands; i++) {
                vtxTableConfigClearBand(vtxTableConfigMutable(), i);
             }
        }
        vtxTableConfigMutable()->bands = bands;

    } else if (strcasecmp(tok, "channels") == 0) {
        tok = strtok_r(NULL, " ", &saveptr);

        int channels = atoi(tok);
        if (channels < 0 || channels > VTX_TABLE_MAX_CHANNELS) {
            cliShowArgumentRangeError(cmdName, "CHANNEL COUNT", 0, VTX_TABLE_MAX_CHANNELS);
            return;
        }
        if (channels < vtxTableConfigMutable()->channels) {
            for (int i = 0; i < VTX_TABLE_MAX_BANDS; i++) {
                vtxTableConfigClearChannels(vtxTableConfigMutable(), i, channels);
            }
        }
        vtxTableConfigMutable()->channels = channels;

    } else if (strcasecmp(tok, "powerlevels") == 0) {
        // Number of power levels
        tok = strtok_r(NULL, " ", &saveptr);
        if (tok) {
            int levels = atoi(tok);
            if (levels < 0 || levels > VTX_TABLE_MAX_POWER_LEVELS) {
                cliShowArgumentRangeError(cmdName, "POWER LEVEL COUNT", 0, VTX_TABLE_MAX_POWER_LEVELS);
            } else {
                if (levels < vtxTableConfigMutable()->powerLevels) {
                    vtxTableConfigClearPowerValues(vtxTableConfigMutable(), levels);
                    vtxTableConfigClearPowerLabels(vtxTableConfigMutable(), levels);
                }
                vtxTableConfigMutable()->powerLevels = levels;
            }
        } else {
            // XXX Show current level count?
        }
        return;

    } else if (strcasecmp(tok, "powervalues") == 0) {
        // Power values
        uint16_t power[VTX_TABLE_MAX_POWER_LEVELS];
        int count;
        int levels = vtxTableConfigMutable()->powerLevels;

        memset(power, 0, sizeof(power));

        for (count = 0; count < levels && (tok = strtok_r(NULL, " ", &saveptr)); count++) {
            int value = atoi(tok);
            power[count] = value;
        }

        // Check remaining tokens

        if (count < levels) {
            cliPrintErrorLinef(cmdName, "NOT ENOUGH VALUES (EXPECTED %d)", levels);
            return;
        } else if ((tok = strtok_r(NULL, " ", &saveptr))) {
            cliPrintErrorLinef(cmdName, "TOO MANY VALUES (EXPECTED %d)", levels);
            return;
        }

        for (int i = 0; i < VTX_TABLE_MAX_POWER_LEVELS; i++) {
            vtxTableConfigMutable()->powerValues[i] = power[i];
        }

    } else if (strcasecmp(tok, "powerlabels") == 0) {
        // Power labels
        char label[VTX_TABLE_MAX_POWER_LEVELS][VTX_TABLE_POWER_LABEL_LENGTH + 1];
        int levels = vtxTableConfigMutable()->powerLevels;
        int count;
        for (count = 0; count < levels && (tok = strtok_r(NULL, " ", &saveptr)); count++) {
            strncpy(label[count], tok, VTX_TABLE_POWER_LABEL_LENGTH);
            for (unsigned i = 0; i < strlen(label[count]); i++) {
                label[count][i] = toupper(label[count][i]);
            }
        }

        // Check remaining tokens

        if (count < levels) {
            cliPrintErrorLinef(cmdName, "NOT ENOUGH LABELS (EXPECTED %d)", levels);
            return;
        } else if ((tok = strtok_r(NULL, " ", &saveptr))) {
            cliPrintErrorLinef(cmdName, "TOO MANY LABELS (EXPECTED %d)", levels);
            return;
        }

        for (int i = 0; i < count; i++) {
            vtxTableStrncpyWithPad(vtxTableConfigMutable()->powerLabels[i], label[i], VTX_TABLE_POWER_LABEL_LENGTH);
        }
    } else if (strcasecmp(tok, "band") == 0) {

        int bands = vtxTableConfigMutable()->bands;

        tok = strtok_r(NULL, " ", &saveptr);
        if (!tok) {
            return;
        }

        int band = atoi(tok);
        --band;

        if (band < 0 || band >= bands) {
            cliShowArgumentRangeError(cmdName, "BAND NUMBER", 1, bands);
            return;
        }

        // Band name
        tok  = strtok_r(NULL, " ", &saveptr);

        if (!tok) {
            return;
        }

        char bandname[VTX_TABLE_BAND_NAME_LENGTH + 1];
        memset(bandname, 0, VTX_TABLE_BAND_NAME_LENGTH + 1);
        strncpy(bandname, tok, VTX_TABLE_BAND_NAME_LENGTH);
        for (unsigned i = 0; i < strlen(bandname); i++) {
            bandname[i] = toupper(bandname[i]);
        }

        // Band letter
        tok  = strtok_r(NULL, " ", &saveptr);

        if (!tok) {
            return;
        }

        char bandletter = toupper(tok[0]);

        uint16_t bandfreq[VTX_TABLE_MAX_CHANNELS];
        int channel = 0;
        int channels = vtxTableConfigMutable()->channels;
        bool isFactory = false;

        for (channel = 0; channel <  channels && (tok  = strtok_r(NULL, " ", &saveptr)); channel++) {
            if (channel == 0 && !isdigit(tok[0])) {
                channel -= 1;
                if (strcasecmp(tok, "FACTORY") == 0) {
                    isFactory = true;
                } else if (strcasecmp(tok, "CUSTOM") == 0) {
                    isFactory = false;
                } else {
                    cliPrintErrorLinef(cmdName, "INVALID FACTORY FLAG %s (EXPECTED FACTORY OR CUSTOM)", tok);
                    return;
                }
            }
            int freq = atoi(tok);
            if (freq < 0) {
                cliPrintErrorLinef(cmdName, "INVALID FREQUENCY %s", tok);
                return;
            }
            bandfreq[channel] = freq;
        }

        if (channel < channels) {
            cliPrintErrorLinef(cmdName, "NOT ENOUGH FREQUENCIES (EXPECTED %d)", channels);
            return;
        } else if ((tok = strtok_r(NULL, " ", &saveptr))) {
            cliPrintErrorLinef(cmdName, "TOO MANY FREQUENCIES (EXPECTED %d)", channels);
            return;
        }

        vtxTableStrncpyWithPad(vtxTableConfigMutable()->bandNames[band], bandname, VTX_TABLE_BAND_NAME_LENGTH);
        vtxTableConfigMutable()->bandLetters[band] = bandletter;

        for (int i = 0; i < channel; i++) {
            vtxTableConfigMutable()->frequency[band][i] = bandfreq[i];
        }
        vtxTableConfigMutable()->isFactoryBand[band] = isFactory;
    } else {
        // Bad subcommand
        cliPrintErrorLinef(cmdName, "INVALID SUBCOMMAND %s", tok);
    }
}

static void cliVtxInfo(const char *cmdName, char *cmdline)
{
    UNUSED(cmdline);

    // Display the available power levels
    uint16_t levels[VTX_TABLE_MAX_POWER_LEVELS];
    uint16_t powers[VTX_TABLE_MAX_POWER_LEVELS];
    vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        uint8_t level_count = vtxCommonGetVTXPowerLevels(vtxDevice, levels, powers);

        if (level_count) {
            for (int i = 0; i < level_count; i++) {
                cliPrintLinef("level %d dBm, power %d mW", levels[i], powers[i]);
            }
        } else {
            cliPrintErrorLinef(cmdName, "NO POWER VALUES DEFINED");
        }
    } else {
        cliPrintErrorLinef(cmdName, "NO VTX");
    }
}
#endif // USE_VTX_TABLE

#ifdef USE_SIMPLIFIED_TUNING
static void cliApplySimplifiedTuning(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    applySimplifiedTuning(currentPidProfile);
    cliPrintLine("Applied tuning based on simplified tuning settings.");
}
#endif

static void printName(dumpFlags_t dumpMask, const pilotConfig_t *pilotConfig)
{
    const bool equalsDefault = strlen(pilotConfig->name) == 0;
    cliDumpPrintLinef(dumpMask, equalsDefault, "\r\n# name: %s", equalsDefault ? emptyName : pilotConfig->name);
}

#if defined(USE_BOARD_INFO)

#define ERROR_MESSAGE "%s CANNOT BE CHANGED. CURRENT VALUE: '%s'"

static void printBoardName(dumpFlags_t dumpMask)
{
    if (!(dumpMask & DO_DIFF) || strlen(getBoardName())) {
        cliPrintLinef("board_name %s", getBoardName());
    }
}

static void cliBoardName(const char *cmdName, char *cmdline)
{
    const unsigned int len = strlen(cmdline);
    const char *boardName = getBoardName();
    if (len > 0 && strlen(boardName) != 0 && boardInformationIsSet() && (len != strlen(boardName) || strncmp(boardName, cmdline, len))) {
        cliPrintErrorLinef(cmdName, ERROR_MESSAGE, "BOARD_NAME", boardName);
    } else {
        if (len > 0 && !configIsInCopy && setBoardName(cmdline)) {
            boardInformationUpdated = true;

            cliPrintHashLine("Set board_name.");
        }
        printBoardName(DUMP_ALL);
    }
}

static void printManufacturerId(dumpFlags_t dumpMask)
{
    if (!(dumpMask & DO_DIFF) || strlen(getManufacturerId())) {
        cliPrintLinef("manufacturer_id %s", getManufacturerId());
    }
}

static void cliManufacturerId(const char *cmdName, char *cmdline)
{
    const unsigned int len = strlen(cmdline);
    const char *manufacturerId = getManufacturerId();
    if (len > 0 && boardInformationIsSet() && strlen(manufacturerId) != 0 && (len != strlen(manufacturerId) || strncmp(manufacturerId, cmdline, len))) {
        cliPrintErrorLinef(cmdName, ERROR_MESSAGE, "MANUFACTURER_ID", manufacturerId);
    } else {
        if (len > 0 && !configIsInCopy && setManufacturerId(cmdline)) {
            boardInformationUpdated = true;

            cliPrintHashLine("Set manufacturer_id.");
        }
        printManufacturerId(DUMP_ALL);
    }
}

#if defined(USE_SIGNATURE)
static void writeSignature(char *signatureStr, uint8_t *signature)
{
    for (unsigned i = 0; i < SIGNATURE_LENGTH; i++) {
        tfp_sprintf(&signatureStr[2 * i], "%02x", signature[i]);
    }
}

static void cliSignature(const char *cmdName, char *cmdline)
{
    const int len = strlen(cmdline);

    uint8_t signature[SIGNATURE_LENGTH] = {0};
    if (len > 0) {
        if (len != 2 * SIGNATURE_LENGTH) {
            cliPrintErrorLinef(cmdName, "INVALID LENGTH: %d (EXPECTED: %d)", len, 2 * SIGNATURE_LENGTH);

            return;
        }

#define BLOCK_SIZE 2
        for (unsigned i = 0; i < SIGNATURE_LENGTH; i++) {
            char temp[BLOCK_SIZE + 1];
            strncpy(temp, &cmdline[i * BLOCK_SIZE], BLOCK_SIZE);
            temp[BLOCK_SIZE] = '\0';
            char *end;
            unsigned result = strtoul(temp, &end, 16);
            if (end == &temp[BLOCK_SIZE]) {
                signature[i] = result;
            } else {
                cliPrintErrorLinef(cmdName, "INVALID CHARACTER FOUND: %c", end[0]);

                return;
            }
        }
#undef BLOCK_SIZE
    }

    char signatureStr[SIGNATURE_LENGTH * 2 + 1] = {0};
    if (len > 0 && signatureIsSet() && memcmp(signature, getSignature(), SIGNATURE_LENGTH)) {
        writeSignature(signatureStr, getSignature());
        cliPrintErrorLinef(cmdName, ERROR_MESSAGE, "SIGNATURE", signatureStr);
    } else {
        if (len > 0 && !configIsInCopy && setSignature(signature)) {
            signatureUpdated = true;

            writeSignature(signatureStr, getSignature());

            cliPrintHashLine("Set signature.");
        } else if (signatureUpdated || signatureIsSet()) {
            writeSignature(signatureStr, getSignature());
        }

        cliPrintLinef("signature %s", signatureStr);
    }
}
#endif

#undef ERROR_MESSAGE

#endif // USE_BOARD_INFO

static void cliMcuId(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    cliPrintLinef("mcu_id %08x%08x%08x", U_ID_0, U_ID_1, U_ID_2);
}

static void printFeature(dumpFlags_t dumpMask, const uint32_t mask, const uint32_t defaultMask, const char *headingStr)
{
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (uint32_t i = 0; featureNames[i]; i++) { // disabled features first
        if (strcmp(featureNames[i], emptyString) != 0) { //Skip unused
            const char *format = "feature -%s";
            const bool equalsDefault = (~defaultMask | mask) & (1 << i);
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDefaultPrintLinef(dumpMask, (defaultMask | ~mask) & (1 << i), format, featureNames[i]);
            cliDumpPrintLinef(dumpMask, equalsDefault, format, featureNames[i]);
        }
    }
    for (uint32_t i = 0; featureNames[i]; i++) {  // enabled features
        if (strcmp(featureNames[i], emptyString) != 0) { //Skip unused
            const char *format = "feature %s";
            if (defaultMask & (1 << i)) {
                cliDefaultPrintLinef(dumpMask, (~defaultMask | mask) & (1 << i), format, featureNames[i]);
            }
            if (mask & (1 << i)) {
                const bool equalsDefault = (defaultMask | ~mask) & (1 << i);
                headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
                cliDumpPrintLinef(dumpMask, equalsDefault, format, featureNames[i]);
            }
        }
    }
}

static void cliFeature(const char *cmdName, char *cmdline)
{
    uint32_t len = strlen(cmdline);
    const uint32_t mask = featureConfig()->enabledFeatures;
    if (len == 0) {
        cliPrint("Enabled: ");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                break;
            }
            if (mask & (1 << i)) {
                cliPrintf("%s ", featureNames[i]);
            }
        }
        cliPrintLinefeed();
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available:");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (strcmp(featureNames[i], emptyString) != 0) //Skip unused
                cliPrintf(" %s", featureNames[i]);
        }
        cliPrintLinefeed();
        return;
    } else {
        uint32_t feature;

        bool remove = false;
        if (cmdline[0] == '-') {
            // remove feature
            remove = true;
            cmdline++; // skip over -
            len--;
        }

        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                cliPrintErrorLinef(cmdName, "INVALID NAME");
                break;
            }

            if (strncasecmp(cmdline, featureNames[i], len) == 0) {
                feature = 1 << i;
#ifndef USE_GPS
                if (feature & FEATURE_GPS) {
                    cliPrintLine("unavailable");
                    break;
                }
#endif
#ifndef USE_RANGEFINDER
                if (feature & FEATURE_RANGEFINDER) {
                    cliPrintLine("unavailable");
                    break;
                }
#endif
                if (remove) {
                    featureConfigClear(feature);
                    cliPrint("Disabled");
                } else {
                    featureConfigSet(feature);
                    cliPrint("Enabled");
                }
                cliPrintLinef(" %s", featureNames[i]);
                break;
            }
        }
    }
}

#if defined(USE_BEEPER)
static void printBeeper(dumpFlags_t dumpMask, const uint32_t offFlags, const uint32_t offFlagsDefault, const char *name, const uint32_t allowedFlags, const char *headingStr)
{
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    const uint8_t beeperCount = beeperTableEntryCount();
    for (int32_t i = 0; i < beeperCount - 1; i++) {
        if (beeperModeMaskForTableIndex(i) & allowedFlags) {
            const char *formatOff = "%s -%s";
            const char *formatOn = "%s %s";
            const uint32_t beeperModeMask = beeperModeMaskForTableIndex(i);
            cliDefaultPrintLinef(dumpMask, ~(offFlags ^ offFlagsDefault) & beeperModeMask, offFlags & beeperModeMask ? formatOn : formatOff, name, beeperNameForTableIndex(i));
            const bool equalsDefault = ~(offFlags ^ offFlagsDefault) & beeperModeMask;
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            cliDumpPrintLinef(dumpMask, equalsDefault, offFlags & beeperModeMask ? formatOff : formatOn, name, beeperNameForTableIndex(i));
        }
    }
}

static void processBeeperCommand(const char *cmdName, char *cmdline, uint32_t *offFlags, const uint32_t allowedFlags)
{
    uint32_t len = strlen(cmdline);
    uint8_t beeperCount = beeperTableEntryCount();

    if (len == 0) {
        cliPrintf("Disabled:");
        for (int32_t i = 0; ; i++) {
            if (i == beeperCount - 1) {
                if (*offFlags == 0)
                    cliPrint("  none");
                break;
            }

            if (beeperModeMaskForTableIndex(i) & *offFlags)
                cliPrintf("  %s", beeperNameForTableIndex(i));
        }
        cliPrintLinefeed();
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available:");
        for (uint32_t i = 0; i < beeperCount; i++) {
            if (beeperModeMaskForTableIndex(i) & allowedFlags) {
                cliPrintf(" %s", beeperNameForTableIndex(i));
            }
        }
        cliPrintLinefeed();
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            remove = true;     // this is for beeper OFF condition
            cmdline++;
            len--;
        }

        for (uint32_t i = 0; ; i++) {
            if (i == beeperCount) {
                cliPrintErrorLinef(cmdName, "INVALID NAME");
                break;
            }
            if (strncasecmp(cmdline, beeperNameForTableIndex(i), len) == 0 && beeperModeMaskForTableIndex(i) & (allowedFlags | BEEPER_GET_FLAG(BEEPER_ALL))) {
                if (remove) { // beeper off
                    if (i == BEEPER_ALL - 1) {
                        *offFlags = allowedFlags;
                    } else {
                        *offFlags |= beeperModeMaskForTableIndex(i);
                    }
                    cliPrint("Disabled");
                }
                else { // beeper on
                    if (i == BEEPER_ALL - 1) {
                        *offFlags = 0;
                    } else {
                        *offFlags &= ~beeperModeMaskForTableIndex(i);
                    }
                    cliPrint("Enabled");
                }
            cliPrintLinef(" %s", beeperNameForTableIndex(i));
            break;
            }
        }
    }
}

#if defined(USE_DSHOT)
static void cliBeacon(const char *cmdName, char *cmdline)
{
    processBeeperCommand(cmdName, cmdline, &(beeperConfigMutable()->dshotBeaconOffFlags), DSHOT_BEACON_ALLOWED_MODES);
}
#endif

static void cliBeeper(const char *cmdName, char *cmdline)
{
    processBeeperCommand(cmdName, cmdline, &(beeperConfigMutable()->beeper_off_flags), BEEPER_ALLOWED_MODES);
}
#endif

#if defined(USE_RX_BIND)
static void cliRxBind(const char *cmdName, char *cmdline)
{
    UNUSED(cmdline);
    if (!startRxBind()) {
        cliPrintErrorLinef(cmdName, "Not supported.");
    } else {
        cliPrintLinef("Binding...");
    }
}
#endif

static void printMap(dumpFlags_t dumpMask, const rxConfig_t *rxConfig, const rxConfig_t *defaultRxConfig, const char *headingStr)
{
    bool equalsDefault = true;
    char buf[16];
    char bufDefault[16];
    uint32_t i;

    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
        buf[rxConfig->rcmap[i]] = rcChannelLetters[i];
        if (defaultRxConfig) {
            bufDefault[defaultRxConfig->rcmap[i]] = rcChannelLetters[i];
            equalsDefault = equalsDefault && (rxConfig->rcmap[i] == defaultRxConfig->rcmap[i]);
        }
    }
    buf[i] = '\0';

    headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
    const char *formatMap = "map %s";
    if (defaultRxConfig) {
        bufDefault[i] = '\0';
        cliDefaultPrintLinef(dumpMask, equalsDefault, formatMap, bufDefault);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault, formatMap, buf);
}


static void cliMap(const char *cmdName, char *cmdline)
{
    uint32_t i;
    char buf[RX_MAPPABLE_CHANNEL_COUNT + 1];

    uint32_t len = strlen(cmdline);
    if (len == RX_MAPPABLE_CHANNEL_COUNT) {

        for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
            buf[i] = toupper((unsigned char)cmdline[i]);
        }
        buf[i] = '\0';

        for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
            buf[i] = toupper((unsigned char)cmdline[i]);

            if (strchr(rcChannelLetters, buf[i]) && !strchr(buf + i + 1, buf[i]))
                continue;

            cliShowParseError(cmdName);
            return;
        }
        parseRcChannels(buf, rxConfigMutable());
    } else if (len > 0) {
        cliShowInvalidArgumentCountError(cmdName);
        return;
    }

    for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
        buf[rxConfig()->rcmap[i]] = rcChannelLetters[i];
    }

    buf[i] = '\0';
    cliPrintLinef("map %s", buf);
}

static char *skipSpace(char *buffer)
{
    while (*(buffer) == ' ') {
        buffer++;
    }

    return buffer;
}

static char *checkCommand(char *cmdline, const char *command)
{
    if (!strncasecmp(cmdline, command, strlen(command))   // command names match
        && (isspace((unsigned)cmdline[strlen(command)]) || cmdline[strlen(command)] == 0)) {
        return skipSpace(cmdline + strlen(command) + 1);
    } else {
        return 0;
    }
}

static void cliRebootEx(rebootTarget_e rebootTarget)
{
    cliPrint("\r\nRebooting");
    cliWriterFlush();
    waitForSerialPortToFinishTransmitting(cliPort);
    motorShutdown();

    switch (rebootTarget) {
    case REBOOT_TARGET_BOOTLOADER_ROM:
        systemResetToBootloader(BOOTLOADER_REQUEST_ROM);

        break;
#if defined(USE_FLASH_BOOT_LOADER)
    case REBOOT_TARGET_BOOTLOADER_FLASH:
        systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);

        break;
#endif
    case REBOOT_TARGET_FIRMWARE:
    default:
        systemReset();

        break;
    }
}

static void cliReboot(void)
{
    cliRebootEx(REBOOT_TARGET_FIRMWARE);
}

static void cliBootloader(const char *cmdName, char *cmdline)
{
    rebootTarget_e rebootTarget;
    if (
#if !defined(USE_FLASH_BOOT_LOADER)
        isEmpty(cmdline) ||
#endif
        strncasecmp(cmdline, "rom", 3) == 0) {
        rebootTarget = REBOOT_TARGET_BOOTLOADER_ROM;

        cliPrintHashLine("restarting in ROM bootloader mode");
#if defined(USE_FLASH_BOOT_LOADER)
    } else if (isEmpty(cmdline) || strncasecmp(cmdline, "flash", 5) == 0) {
        rebootTarget = REBOOT_TARGET_BOOTLOADER_FLASH;

        cliPrintHashLine("restarting in flash bootloader mode");
#endif
    } else {
        cliPrintErrorLinef(cmdName, "Invalid option");

        return;
    }

    cliRebootEx(rebootTarget);
}

static void cliExit(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    cliPrintHashLine("leaving CLI mode, unsaved changes lost");
    cliWriterFlush();

    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = false;
    // incase a motor was left running during motortest, clear it here
    mixerResetDisarmedMotors();
    cliReboot();
}

#ifdef USE_GPS
static void cliGpsPassthrough(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    gpsEnablePassthrough(cliPort);
}
#endif

#if defined(USE_GYRO_REGISTER_DUMP) && !defined(SIMULATOR_BUILD)
static void cliPrintGyroRegisters(uint8_t whichSensor)
{
    cliPrintLinef("# WHO_AM_I    0x%X", gyroReadRegister(whichSensor, MPU_RA_WHO_AM_I));
    cliPrintLinef("# CONFIG      0x%X", gyroReadRegister(whichSensor, MPU_RA_CONFIG));
    cliPrintLinef("# GYRO_CONFIG 0x%X", gyroReadRegister(whichSensor, MPU_RA_GYRO_CONFIG));
}

static void cliDumpGyroRegisters(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

#ifdef USE_MULTI_GYRO
    if ((gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_1) || (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_BOTH)) {
        cliPrintLinef("\r\n# Gyro 1");
        cliPrintGyroRegisters(GYRO_CONFIG_USE_GYRO_1);
    }
    if ((gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_2) || (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_BOTH)) {
        cliPrintLinef("\r\n# Gyro 2");
        cliPrintGyroRegisters(GYRO_CONFIG_USE_GYRO_2);
    }
#else
    cliPrintGyroRegisters(GYRO_CONFIG_USE_GYRO_1);
#endif
}
#endif


static int parseOutputIndex(const char *cmdName, char *pch, bool allowAllEscs) {
    int outputIndex = atoi(pch);
    if ((outputIndex >= 0) && (outputIndex < getMotorCount())) {
        cliPrintLinef("Using output %d.", outputIndex);
    } else if (allowAllEscs && outputIndex == ALL_MOTORS) {
        cliPrintLinef("Using all outputs.");
    } else {
        cliPrintErrorLinef(cmdName, "INVALID OUTPUT NUMBER. RANGE: 0 - %d.", getMotorCount() - 1);

        return -1;
    }

    return outputIndex;
}

#if defined(USE_DSHOT)
#if defined(USE_ESC_SENSOR) && defined(USE_ESC_SENSOR_INFO)

#define ESC_INFO_KISS_V1_EXPECTED_FRAME_SIZE 15
#define ESC_INFO_KISS_V2_EXPECTED_FRAME_SIZE 21
#define ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE 64

enum {
    ESC_INFO_KISS_V1,
    ESC_INFO_KISS_V2,
    ESC_INFO_BLHELI32
};

#define ESC_INFO_VERSION_POSITION 12

static void printEscInfo(const char *cmdName, const uint8_t *escInfoBuffer, uint8_t bytesRead)
{
    bool escInfoReceived = false;
    if (bytesRead > ESC_INFO_VERSION_POSITION) {
        uint8_t escInfoVersion;
        uint8_t frameLength;
        if (escInfoBuffer[ESC_INFO_VERSION_POSITION] == 254) {
            escInfoVersion = ESC_INFO_BLHELI32;
            frameLength = ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE;
        } else if (escInfoBuffer[ESC_INFO_VERSION_POSITION] == 255) {
            escInfoVersion = ESC_INFO_KISS_V2;
            frameLength = ESC_INFO_KISS_V2_EXPECTED_FRAME_SIZE;
        } else {
            escInfoVersion = ESC_INFO_KISS_V1;
            frameLength = ESC_INFO_KISS_V1_EXPECTED_FRAME_SIZE;
        }

        if (bytesRead == frameLength) {
            escInfoReceived = true;

            if (calculateCrc8(escInfoBuffer, frameLength - 1) == escInfoBuffer[frameLength - 1]) {
                uint8_t firmwareVersion = 0;
                uint8_t firmwareSubVersion = 0;
                uint8_t escType = 0;
                switch (escInfoVersion) {
                case ESC_INFO_KISS_V1:
                    firmwareVersion = escInfoBuffer[12];
                    firmwareSubVersion = (escInfoBuffer[13] & 0x1f) + 97;
                    escType = (escInfoBuffer[13] & 0xe0) >> 5;

                    break;
                case ESC_INFO_KISS_V2:
                    firmwareVersion = escInfoBuffer[13];
                    firmwareSubVersion = escInfoBuffer[14];
                    escType = escInfoBuffer[15];

                    break;
                case ESC_INFO_BLHELI32:
                    firmwareVersion = escInfoBuffer[13];
                    firmwareSubVersion = escInfoBuffer[14];
                    escType = escInfoBuffer[15];

                    break;
                }

                cliPrint("ESC Type: ");
                switch (escInfoVersion) {
                case ESC_INFO_KISS_V1:
                case ESC_INFO_KISS_V2:
                    switch (escType) {
                    case 1:
                        cliPrintLine("KISS8A");

                        break;
                    case 2:
                        cliPrintLine("KISS16A");

                        break;
                    case 3:
                        cliPrintLine("KISS24A");

                        break;
                    case 5:
                        cliPrintLine("KISS Ultralite");

                        break;
                    default:
                        cliPrintLine("unknown");

                        break;
                    }

                    break;
                case ESC_INFO_BLHELI32:
                    {
                        char *escType = (char *)(escInfoBuffer + 31);
                        escType[32] = 0;
                        cliPrintLine(escType);
                    }

                    break;
                }

                cliPrint("MCU Serial No: 0x");
                for (int i = 0; i < 12; i++) {
                    if (i && (i % 3 == 0)) {
                        cliPrint("-");
                    }
                    cliPrintf("%02x", escInfoBuffer[i]);
                }
                cliPrintLinefeed();

                switch (escInfoVersion) {
                case ESC_INFO_KISS_V1:
                case ESC_INFO_KISS_V2:
                    cliPrintLinef("Firmware Version: %d.%02d%c", firmwareVersion / 100, firmwareVersion % 100, (char)firmwareSubVersion);

                    break;
                case ESC_INFO_BLHELI32:
                    cliPrintLinef("Firmware Version: %d.%02d%", firmwareVersion, firmwareSubVersion);

                    break;
                }
                if (escInfoVersion == ESC_INFO_KISS_V2 || escInfoVersion == ESC_INFO_BLHELI32) {
                    cliPrintLinef("Rotation Direction: %s", escInfoBuffer[16] ? "reversed" : "normal");
                    cliPrintLinef("3D: %s", escInfoBuffer[17] ? "on" : "off");
                    if (escInfoVersion == ESC_INFO_BLHELI32) {
                        uint8_t setting = escInfoBuffer[18];
                        cliPrint("Low voltage Limit: ");
                        switch (setting) {
                        case 0:
                            cliPrintLine("off");

                            break;
                        case 255:
                            cliPrintLine("unsupported");

                            break;
                        default:
                            cliPrintLinef("%d.%01d", setting / 10, setting % 10);

                            break;
                        }

                        setting = escInfoBuffer[19];
                        cliPrint("Current Limit: ");
                        switch (setting) {
                        case 0:
                            cliPrintLine("off");

                            break;
                        case 255:
                            cliPrintLine("unsupported");

                            break;
                        default:
                            cliPrintLinef("%d", setting);

                            break;
                        }

                        for (int i = 0; i < 4; i++) {
                            setting = escInfoBuffer[i + 20];
                            cliPrintLinef("LED %d: %s", i, setting ? (setting == 255) ? "unsupported" : "on" : "off");
                        }
                    }
                }
            } else {
                cliPrintErrorLinef(cmdName, "CHECKSUM ERROR.");
            }
        }
    }

    if (!escInfoReceived) {
        cliPrintLine("No Info.");
    }
}

static void executeEscInfoCommand(const char *cmdName, uint8_t escIndex)
{
    cliPrintLinef("Info for ESC %d:", escIndex);

    uint8_t escInfoBuffer[ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE];

    startEscDataRead(escInfoBuffer, ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE);

    dshotCommandWrite(escIndex, getMotorCount(), DSHOT_CMD_ESC_INFO, DSHOT_CMD_TYPE_BLOCKING);

    delay(10);

    printEscInfo(cmdName, escInfoBuffer, getNumberEscBytesRead());
}
#endif // USE_ESC_SENSOR && USE_ESC_SENSOR_INFO

static void cliDshotProg(const char *cmdName, char *cmdline)
{
    if (isEmpty(cmdline) || !isMotorProtocolDshot()) {
        cliShowParseError(cmdName);

        return;
    }

    char *saveptr;
    char *pch = strtok_r(cmdline, " ", &saveptr);
    int pos = 0;
    int escIndex = 0;
    bool firstCommand = true;
    while (pch != NULL) {
        switch (pos) {
        case 0:
            escIndex = parseOutputIndex(cmdName, pch, true);
            if (escIndex == -1) {
                return;
            }

            break;
        default:
            {
                int command = atoi(pch);
                if (command >= 0 && command < DSHOT_MIN_THROTTLE) {
                    if (firstCommand) {
                        // pwmDisableMotors();
                        motorDisable();

                        firstCommand = false;
                    }

                    if (command != DSHOT_CMD_ESC_INFO) {
                        dshotCommandWrite(escIndex, getMotorCount(), command, DSHOT_CMD_TYPE_BLOCKING);
                    } else {
#if defined(USE_ESC_SENSOR) && defined(USE_ESC_SENSOR_INFO)
                        if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                            if (escIndex != ALL_MOTORS) {
                                executeEscInfoCommand(cmdName, escIndex);
                            } else {
                                for (uint8_t i = 0; i < getMotorCount(); i++) {
                                    executeEscInfoCommand(cmdName, i);
                                }
                            }
                        } else
#endif
                        {
                            cliPrintLine("Not supported.");
                        }
                    }

                    cliPrintLinef("Command Sent: %d", command);

                } else {
                    cliPrintErrorLinef(cmdName, "INVALID COMMAND. RANGE: 1 - %d.", DSHOT_MIN_THROTTLE - 1);
                }
            }

            break;
        }

        pos++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    motorEnable();
}
#endif // USE_DSHOT

#ifdef USE_ESCSERIAL
static void cliEscPassthrough(const char *cmdName, char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliShowInvalidArgumentCountError(cmdName);

        return;
    }

    char *saveptr;
    char *pch = strtok_r(cmdline, " ", &saveptr);
    int pos = 0;
    uint8_t mode = 0;
    int escIndex = 0;
    while (pch != NULL) {
        switch (pos) {
        case 0:
            if (strncasecmp(pch, "sk", strlen(pch)) == 0) {
                mode = PROTOCOL_SIMONK;
            } else if (strncasecmp(pch, "bl", strlen(pch)) == 0) {
                mode = PROTOCOL_BLHELI;
            } else if (strncasecmp(pch, "ki", strlen(pch)) == 0) {
                mode = PROTOCOL_KISS;
            } else if (strncasecmp(pch, "cc", strlen(pch)) == 0) {
                mode = PROTOCOL_KISSALL;
            } else {
                cliShowParseError(cmdName);

                return;
            }
            break;
        case 1:
            escIndex = parseOutputIndex(cmdName, pch, mode == PROTOCOL_KISS);
            if (escIndex == -1) {
                return;
            }

            break;
        default:
            cliShowInvalidArgumentCountError(cmdName);

            return;

            break;

        }
        pos++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    if (!escEnablePassthrough(cliPort, &motorConfig()->dev, escIndex, mode)) {
        cliPrintErrorLinef(cmdName, "Error starting ESC connection");
    }
}
#endif

#ifndef USE_QUAD_MIXER_ONLY
static void cliMixer(const char *cmdName, char *cmdline)
{
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrintLinef("Mixer: %s", mixerNames[mixerConfig()->mixerMode - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available:");
        for (uint32_t i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            cliPrintf(" %s", mixerNames[i]);
        }
        cliPrintLinefeed();
        return;
    }

    for (uint32_t i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            cliPrintErrorLinef(cmdName, "INVALID NAME");
            return;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            mixerConfigMutable()->mixerMode = i + 1;
            break;
        }
    }

    cliMixer(cmdName, "");
}
#endif

static void cliMotor(const char *cmdName, char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliShowInvalidArgumentCountError(cmdName);

        return;
    }

    int motorIndex = 0;
    int motorValue = 0;

    char *saveptr;
    char *pch = strtok_r(cmdline, " ", &saveptr);
    int index = 0;
    while (pch != NULL) {
        switch (index) {
        case 0:
            motorIndex = parseOutputIndex(cmdName, pch, true);
            if (motorIndex == -1) {
                return;
            }

            break;
        case 1:
            motorValue = atoi(pch);

            break;
        }
        index++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    if (index == 2) {
        if (motorValue < PWM_RANGE_MIN || motorValue > PWM_RANGE_MAX) {
            cliShowArgumentRangeError(cmdName, "VALUE", 1000, 2000);
        } else {
            uint32_t motorOutputValue = motorConvertFromExternal(motorValue);

            if (motorIndex != ALL_MOTORS) {
                motor_disarmed[motorIndex] = motorOutputValue;

                cliPrintLinef("motor %d: %d", motorIndex, motorOutputValue);
            } else  {
                for (int i = 0; i < getMotorCount(); i++) {
                    motor_disarmed[i] = motorOutputValue;
                }

                cliPrintLinef("all motors: %d", motorOutputValue);
            }
        }
    } else {
        cliShowInvalidArgumentCountError(cmdName);
    }
}

#ifndef MINIMAL_CLI
static void cliPlaySound(const char *cmdName, char *cmdline)
{
    int i;
    const char *name;
    static int lastSoundIdx = -1;

    if (isEmpty(cmdline)) {
        i = lastSoundIdx + 1;     //next sound index
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            while (true) {   //no name for index; try next one
                if (++i >= beeperTableEntryCount())
                    i = 0;   //if end then wrap around to first entry
                if ((name=beeperNameForTableIndex(i)) != NULL)
                    break;   //if name OK then play sound below
                if (i == lastSoundIdx + 1) {     //prevent infinite loop
                    cliPrintErrorLinef(cmdName, "ERROR PLAYING SOUND");
                    return;
                }
            }
        }
    } else {       //index value was given
        i = atoi(cmdline);
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            cliPrintLinef("No sound for index %d", i);
            return;
        }
    }
    lastSoundIdx = i;
    beeperSilence();
    cliPrintLinef("Playing sound %d: %s", i, name);
    beeper(beeperModeForTableIndex(i));
}
#endif

static void cliProfile(const char *cmdName, char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliPrintLinef("profile %d", getPidProfileIndexToUse());
        return;
    } else {
        const int i = atoi(cmdline);
        if (i >= 0 && i < PID_PROFILE_COUNT) {
            changePidProfile(i);
            cliProfile(cmdName, "");
        } else {
            cliPrintErrorLinef(cmdName, "PROFILE OUTSIDE OF [0..%d]", PID_PROFILE_COUNT - 1);
        }
    }
}

static void cliRateProfile(const char *cmdName, char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliPrintLinef("rateprofile %d", getRateProfileIndexToUse());
        return;
    } else {
        const int i = atoi(cmdline);
        if (i >= 0 && i < CONTROL_RATE_PROFILE_COUNT) {
            changeControlRateProfile(i);
            cliRateProfile(cmdName, "");
        } else {
            cliPrintErrorLinef(cmdName, "RATE PROFILE OUTSIDE OF [0..%d]", CONTROL_RATE_PROFILE_COUNT - 1);
        }
    }
}

static void cliDumpPidProfile(const char *cmdName, uint8_t pidProfileIndex, dumpFlags_t dumpMask)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        // Faulty values
        return;
    }

    pidProfileIndexToUse = pidProfileIndex;

    cliPrintLinefeed();
    cliProfile(cmdName, "");

    char profileStr[10];
    tfp_sprintf(profileStr, "profile %d", pidProfileIndex);
    dumpAllValues(cmdName, PROFILE_VALUE, dumpMask, profileStr);

    pidProfileIndexToUse = CURRENT_PROFILE_INDEX;
}

static void cliDumpRateProfile(const char *cmdName, uint8_t rateProfileIndex, dumpFlags_t dumpMask)
{
    if (rateProfileIndex >= CONTROL_RATE_PROFILE_COUNT) {
        // Faulty values
        return;
    }

    rateProfileIndexToUse = rateProfileIndex;

    cliPrintLinefeed();
    cliRateProfile(cmdName, "");

    char rateProfileStr[14];
    tfp_sprintf(rateProfileStr, "rateprofile %d", rateProfileIndex);
    dumpAllValues(cmdName, PROFILE_RATE_VALUE, dumpMask, rateProfileStr);

    rateProfileIndexToUse = CURRENT_PROFILE_INDEX;
}

#ifdef USE_CLI_BATCH
static void cliPrintCommandBatchWarning(const char *cmdName, const char *warning)
{
    cliPrintErrorLinef(cmdName, "ERRORS WERE DETECTED - PLEASE REVIEW BEFORE CONTINUING");
    if (warning) {
        cliPrintErrorLinef(cmdName, warning);
    }
}

static void resetCommandBatch(void)
{
    commandBatchActive = false;
    commandBatchError = false;
}

static void cliBatch(const char *cmdName, char *cmdline)
{
    if (strncasecmp(cmdline, "start", 5) == 0) {
        if (!commandBatchActive) {
            commandBatchActive = true;
            commandBatchError = false;
        }
        cliPrintLine("Command batch started");
    } else if (strncasecmp(cmdline, "end", 3) == 0) {
        if (commandBatchActive && commandBatchError) {
            cliPrintCommandBatchWarning(cmdName, NULL);
        } else {
            cliPrintLine("Command batch ended");
        }
        resetCommandBatch();
    } else {
        cliPrintErrorLinef(cmdName, "Invalid option");
    }
}
#endif

static bool prepareSave(void)
{
#if defined(USE_CUSTOM_DEFAULTS)
    if (processingCustomDefaults) {
        return true;
    }
#endif

#ifdef USE_CLI_BATCH
    if (commandBatchActive && commandBatchError) {
        return false;
    }
#endif

#if defined(USE_BOARD_INFO)
    if (boardInformationUpdated) {
        persistBoardInformation();
    }
#if defined(USE_SIGNATURE)
    if (signatureUpdated) {
        persistSignature();
    }
#endif
#endif // USE_BOARD_INFO

    return true;
}

bool tryPrepareSave(const char *cmdName)
{
    bool success = prepareSave();
#if defined(USE_CLI_BATCH)
    if (!success) {
        cliPrintCommandBatchWarning(cmdName, "PLEASE FIX ERRORS THEN 'SAVE'");
        resetCommandBatch();

        return false;
    }
#else
    UNUSED(cmdName);
    UNUSED(success);
#endif

    return true;
}

static void cliSave(const char *cmdName, char *cmdline)
{
    UNUSED(cmdline);

    if (tryPrepareSave(cmdName)) {
        writeEEPROM();
        cliPrintHashLine("saving");

        cliReboot();
    }
}

#if defined(USE_CUSTOM_DEFAULTS)
bool resetConfigToCustomDefaults(void)
{
    resetConfig();

#ifdef USE_CLI_BATCH
    commandBatchError = false;
#endif

    cliProcessCustomDefaults(true);

    return prepareSave();
}

static bool customDefaultsHasNext(const char *customDefaultsPtr)
{
    return *customDefaultsPtr && *customDefaultsPtr != 0xFF && customDefaultsPtr < customDefaultsEnd;
}

static const char *parseCustomDefaultsHeaderElement(char *dest, const char *customDefaultsPtr, const char *prefix, const char terminator, const unsigned maxLength)
{
    char *endPtr = NULL;
    unsigned len = strlen(prefix);
    if (customDefaultsPtr && customDefaultsHasNext(customDefaultsPtr) && strncmp(customDefaultsPtr, prefix, len) == 0) {
        customDefaultsPtr += len;
        endPtr = strchr(customDefaultsPtr, terminator);
    }

    if (endPtr && customDefaultsHasNext(endPtr)) {
        len = endPtr - customDefaultsPtr;
        memcpy(dest, customDefaultsPtr, MIN(len, maxLength));

        customDefaultsPtr += len;

        return customDefaultsPtr;
    }

    return NULL;
}

static void parseCustomDefaultsHeader(void)
{
    const char *customDefaultsPtr = customDefaultsStart;
    if (strncmp(customDefaultsPtr, CUSTOM_DEFAULTS_START_PREFIX, strlen(CUSTOM_DEFAULTS_START_PREFIX)) == 0) {
        customDefaultsFound = true;

        customDefaultsPtr = strchr(customDefaultsPtr, '\n');
        if (customDefaultsPtr && customDefaultsHasNext(customDefaultsPtr)) {
            customDefaultsPtr++;
        }

        customDefaultsPtr = parseCustomDefaultsHeaderElement(customDefaultsManufacturerId, customDefaultsPtr, CUSTOM_DEFAULTS_MANUFACTURER_ID_PREFIX, CUSTOM_DEFAULTS_BOARD_NAME_PREFIX[0], MAX_MANUFACTURER_ID_LENGTH);

        customDefaultsPtr = parseCustomDefaultsHeaderElement(customDefaultsBoardName, customDefaultsPtr, CUSTOM_DEFAULTS_BOARD_NAME_PREFIX, CUSTOM_DEFAULTS_CHANGESET_ID_PREFIX[0], MAX_BOARD_NAME_LENGTH);

        customDefaultsPtr = parseCustomDefaultsHeaderElement(customDefaultsChangesetId, customDefaultsPtr, CUSTOM_DEFAULTS_CHANGESET_ID_PREFIX, CUSTOM_DEFAULTS_DATE_PREFIX[0], MAX_CHANGESET_ID_LENGTH);

        customDefaultsPtr = parseCustomDefaultsHeaderElement(customDefaultsDate, customDefaultsPtr, CUSTOM_DEFAULTS_DATE_PREFIX, '\n', MAX_DATE_LENGTH);
    }

    customDefaultsHeaderParsed = true;
}

bool hasCustomDefaults(void)
{
    if (!customDefaultsHeaderParsed) {
        parseCustomDefaultsHeader();
    }

    return customDefaultsFound;
}
#endif

static void cliDefaults(const char *cmdName, char *cmdline)
{
    bool saveConfigs = true;
#if defined(USE_CUSTOM_DEFAULTS)
    bool useCustomDefaults = true;
#elif defined(USE_CUSTOM_DEFAULTS_ADDRESS)
    // Required to keep the linker from eliminating these
    if (customDefaultsStart != customDefaultsEnd) {
        delay(0);
    }
#endif

    if (isEmpty(cmdline)) {
    } else if (strncasecmp(cmdline, "nosave", 6) == 0) {
        saveConfigs = false;
#if defined(USE_CUSTOM_DEFAULTS)
    } else if (strncasecmp(cmdline, "bare", 4) == 0) {
        useCustomDefaults = false;
    } else if (strncasecmp(cmdline, "show", 4) == 0) {
        if (hasCustomDefaults()) {
            char *customDefaultsPtr = customDefaultsStart;
            while (customDefaultsHasNext(customDefaultsPtr)) {
                if (*customDefaultsPtr != '\n') {
                    cliPrintf("%c", *customDefaultsPtr++);
                } else {
                    cliPrintLinefeed();
                    customDefaultsPtr++;
                }
            }
        } else {
            cliPrintError(cmdName, "NO CUSTOM DEFAULTS FOUND");
        }

        return;
#endif
    } else {
        cliPrintError(cmdName, "INVALID OPTION");

        return;
    }

    cliPrintHashLine("resetting to defaults");

    resetConfig();

#ifdef USE_CLI_BATCH
    // Reset only the error state and allow the batch active state to remain.
    // This way if a "defaults nosave" was issued after the "batch on" we'll
    // only reset the current error state but the batch will still be active
    // for subsequent commands.
    commandBatchError = false;
#endif

#if defined(USE_CUSTOM_DEFAULTS)
    if (useCustomDefaults) {
        cliProcessCustomDefaults(false);
    }
#endif

    if (saveConfigs && tryPrepareSave(cmdName)) {
        writeUnmodifiedConfigToEEPROM();

        cliReboot();
    }
}

static void cliPrintVarDefault(const char *cmdName, const clivalue_t *value)
{
    const pgRegistry_t *pg = pgFind(value->pgn);
    if (pg) {
        const char *defaultFormat = "Default value: ";
        const int valueOffset = getValueOffset(value);
        const bool equalsDefault = valuePtrEqualsDefault(value, pg->copy + valueOffset, pg->address + valueOffset);
        if (!equalsDefault) {
            cliPrintf(defaultFormat, value->name);
            printValuePointer(cmdName, value, (uint8_t*)pg->address + valueOffset, false);
            cliPrintLinefeed();
        }
    }
}

STATIC_UNIT_TESTED void cliGet(const char *cmdName, char *cmdline)
{
    const clivalue_t *val;
    int matchedCommands = 0;

    pidProfileIndexToUse = getCurrentPidProfileIndex();
    rateProfileIndexToUse = getCurrentControlRateProfileIndex();

    backupAndResetConfigs(true);

    for (uint32_t i = 0; i < valueTableEntryCount; i++) {
        if (strcasestr(valueTable[i].name, cmdline)) {
            val = &valueTable[i];
            if (matchedCommands > 0) {
                cliPrintLinefeed();
            }
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(cmdName, val, 0);
            cliPrintLinefeed();
            switch (val->type & VALUE_SECTION_MASK) {
            case PROFILE_VALUE:
                cliProfile(cmdName, "");

                break;
            case PROFILE_RATE_VALUE:
                cliRateProfile(cmdName, "");

                break;
            default:

                break;
            }
            cliPrintVarRange(val);
            cliPrintVarDefault(cmdName, val);

            matchedCommands++;
        }
    }

    restoreConfigs();

    pidProfileIndexToUse = CURRENT_PROFILE_INDEX;
    rateProfileIndexToUse = CURRENT_PROFILE_INDEX;

    if (!matchedCommands) {
        cliPrintErrorLinef(cmdName, "INVALID NAME");
    }
}

static uint8_t getWordLength(char *bufBegin, char *bufEnd)
{
    while (*(bufEnd - 1) == ' ') {
        bufEnd--;
    }

    return bufEnd - bufBegin;
}

uint16_t cliGetSettingIndex(char *name, uint8_t length)
{
    for (uint32_t i = 0; i < valueTableEntryCount; i++) {
        const char *settingName = valueTable[i].name;

        // ensure exact match when setting to prevent setting variables with shorter names
        if (strncasecmp(name, settingName, strlen(settingName)) == 0 && length == strlen(settingName)) {
            return i;
        }
    }
    return valueTableEntryCount;
}

STATIC_UNIT_TESTED void cliSet(const char *cmdName, char *cmdline)
{
    const uint32_t len = strlen(cmdline);
    char *eqptr;

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrintLine("Current settings: ");

        for (uint32_t i = 0; i < valueTableEntryCount; i++) {
            const clivalue_t *val = &valueTable[i];
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(cmdName, val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            cliPrintLinefeed();
        }
    } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
        // has equals

        uint8_t variableNameLength = getWordLength(cmdline, eqptr);

        // skip the '=' and any ' ' characters
        eqptr++;
        eqptr = skipSpace(eqptr);

        const uint16_t index = cliGetSettingIndex(cmdline, variableNameLength);
        if (index >= valueTableEntryCount) {
            cliPrintErrorLinef(cmdName, "INVALID NAME");
            return;
        }
        const clivalue_t *val = &valueTable[index];

        bool valueChanged = false;
        int16_t value  = 0;
        switch (val->type & VALUE_MODE_MASK) {
        case MODE_DIRECT: {
                if ((val->type & VALUE_TYPE_MASK) == VAR_UINT32) {
                    uint32_t value = strtoul(eqptr, NULL, 10);

                    if (value <= val->config.u32Max) {
                        cliSetVar(val, value);
                        valueChanged = true;
                    }
                } else {
                    int value = atoi(eqptr);

                    int min;
                    int max;
                    getMinMax(val, &min, &max);

                    if (value >= min && value <= max) {
                        cliSetVar(val, value);
                        valueChanged = true;
                    }
                }
            }

            break;
        case MODE_LOOKUP:
        case MODE_BITSET: {
                int tableIndex;
                if ((val->type & VALUE_MODE_MASK) == MODE_BITSET) {
                    tableIndex = TABLE_OFF_ON;
                } else {
                    tableIndex = val->config.lookup.tableIndex;
                }
                const lookupTableEntry_t *tableEntry = &lookupTables[tableIndex];
                bool matched = false;
                for (uint32_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
                    matched = tableEntry->values[tableValueIndex] && strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

                    if (matched) {
                        value = tableValueIndex;

                        cliSetVar(val, value);
                        valueChanged = true;
                    }
                }
            }

            break;

        case MODE_ARRAY: {
                const uint8_t arrayLength = val->config.array.length;
                char *valPtr = eqptr;

                int i = 0;
                while (i < arrayLength && valPtr != NULL) {
                    // skip spaces
                    valPtr = skipSpace(valPtr);

                    // process substring starting at valPtr
                    // note: no need to copy substrings for atoi()
                    //       it stops at the first character that cannot be converted...
                    switch (val->type & VALUE_TYPE_MASK) {
                    default:
                    case VAR_UINT8:
                        {
                            // fetch data pointer
                            uint8_t *data = (uint8_t *)cliGetValuePointer(val) + i;
                            // store value
                            *data = (uint8_t)atoi((const char*) valPtr);
                        }

                        break;
                    case VAR_INT8:
                        {
                            // fetch data pointer
                            int8_t *data = (int8_t *)cliGetValuePointer(val) + i;
                            // store value
                            *data = (int8_t)atoi((const char*) valPtr);
                        }

                        break;
                    case VAR_UINT16:
                        {
                            // fetch data pointer
                            uint16_t *data = (uint16_t *)cliGetValuePointer(val) + i;
                            // store value
                            *data = (uint16_t)atoi((const char*) valPtr);
                        }

                        break;
                    case VAR_INT16:
                        {
                            // fetch data pointer
                            int16_t *data = (int16_t *)cliGetValuePointer(val) + i;
                            // store value
                            *data = (int16_t)atoi((const char*) valPtr);
                        }

                        break;
                    case VAR_UINT32:
                        {
                            // fetch data pointer
                            uint32_t *data = (uint32_t *)cliGetValuePointer(val) + i;
                            // store value
                            *data = (uint32_t)strtoul((const char*) valPtr, NULL, 10);
                       }

                        break;
                    }

                    // find next comma (or end of string)
                    valPtr = strchr(valPtr, ',') + 1;

                    i++;
                }
            }

            // mark as changed
            valueChanged = true;

            break;
        case MODE_STRING: {
                char *valPtr = eqptr;
                valPtr = skipSpace(valPtr);

                const unsigned int len = strlen(valPtr);
                const uint8_t min = val->config.string.minlength;
                const uint8_t max = val->config.string.maxlength;
                const bool updatable = ((val->config.string.flags & STRING_FLAGS_WRITEONCE) == 0 ||
                                        strlen((char *)cliGetValuePointer(val)) == 0 ||
                                        strncmp(valPtr, (char *)cliGetValuePointer(val), len) == 0);

                if (updatable && len > 0 && len <= max) {
                    memset((char *)cliGetValuePointer(val), 0, max);
                    if (len >= min && strncmp(valPtr, emptyName, len)) {
                        strncpy((char *)cliGetValuePointer(val), valPtr, len);
                    }
                    valueChanged = true;
                } else {
                    cliPrintErrorLinef(cmdName, "STRING MUST BE 1-%d CHARACTERS OR '-' FOR EMPTY", max);
                }
            }
            break;
        }

        if (valueChanged) {
            cliPrintf("%s set to ", val->name);
            cliPrintVar(cmdName, val, 0);
        } else {
            cliPrintErrorLinef(cmdName, "INVALID VALUE");
            cliPrintVarRange(val);
        }

        return;
    } else {
        // no equals, check for matching variables.
        cliGet(cmdName, cmdline);
    }
}

static const char *getMcuTypeById(mcuTypeId_e id)
{
    if (id < MCU_TYPE_UNKNOWN) {
        return mcuTypeNames[id];
    } else {
        return "UNKNOWN";
    }
}

static void cliStatus(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    // MCU type, clock, vrefint, core temperature

    cliPrintf("MCU %s Clock=%dMHz", getMcuTypeById(getMcuTypeId()), (SystemCoreClock / 1000000));

#if defined(STM32F4) || defined(STM32G4)
    // Only F4 and G4 is capable of switching between HSE/HSI (for now)
    int sysclkSource = SystemSYSCLKSource();

    const char *SYSCLKSource[] = { "HSI", "HSE", "PLLP", "PLLR" };
    const char *PLLSource[] = { "-HSI", "-HSE" };

    int pllSource;

    if (sysclkSource >= 2) {
        pllSource = SystemPLLSource();
    }

    cliPrintf(" (%s%s)", SYSCLKSource[sysclkSource], (sysclkSource < 2) ? "" : PLLSource[pllSource]);
#endif

#ifdef USE_ADC_INTERNAL
    uint16_t vrefintMv = getVrefMv();
    int16_t coretemp = getCoreTemperatureCelsius();
    cliPrintLinef(", Vref=%d.%2dV, Core temp=%ddegC", vrefintMv / 1000, (vrefintMv % 1000) / 10, coretemp);
#else
    cliPrintLinefeed();
#endif

    // Stack and config sizes and usages

    cliPrintf("Stack size: %d, Stack address: 0x%x", stackTotalSize(), stackHighMem());
#ifdef USE_STACK_CHECK
    cliPrintf(", Stack used: %d", stackUsedSize());
#endif
    cliPrintLinefeed();

    cliPrintLinef("Configuration: %s, size: %d, max available: %d", configurationStates[systemConfigMutable()->configurationState], getEEPROMConfigSize(), getEEPROMStorageSize());

    // Devices
#if defined(USE_SPI) || defined(USE_I2C)
    cliPrint("Devices detected:");
#if defined(USE_SPI)
    cliPrintf(" SPI:%d", spiGetRegisteredDeviceCount());
#if defined(USE_I2C)
    cliPrint(",");
#endif
#endif
#if defined(USE_I2C)
    cliPrintf(" I2C:%d", i2cGetRegisteredDeviceCount());
#endif
    cliPrintLinefeed();
#endif

    // Sensors
    cliPrint("Gyros detected:");
    bool found = false;
    for (unsigned pos = 0; pos < 7; pos++) {
        if (gyroConfig()->gyrosDetected & BIT(pos)) {
            if (found) {
                cliPrint(",");
            } else {
                found = true;
            }
            cliPrintf(" gyro %d", pos + 1);
        }
    }
    cliPrintLinefeed();

#if defined(USE_SENSOR_NAMES)
    const uint32_t detectedSensorsMask = sensorsMask();
    for (uint32_t i = 0; ; i++) {
        if (sensorTypeNames[i] == NULL) {
            break;
        }
        const uint32_t mask = (1 << i);
        if ((detectedSensorsMask & mask) && (mask & SENSOR_NAMES_MASK)) {
            const uint8_t sensorHardwareIndex = detectedSensors[i];
            const char *sensorHardware = sensorHardwareNames[i][sensorHardwareIndex];
            if (i) {
                cliPrint(", ");
            }
            cliPrintf("%s=%s", sensorTypeNames[i], sensorHardware);
#if defined(USE_ACC)
            if (mask == SENSOR_ACC && acc.dev.revisionCode) {
                cliPrintf(".%c", acc.dev.revisionCode);
            }
#endif
        }
    }
    cliPrintLinefeed();
#endif /* USE_SENSOR_NAMES */

#if defined(USE_OSD)
    osdDisplayPortDevice_e displayPortDeviceType;
    osdGetDisplayPort(&displayPortDeviceType);

    cliPrintLinef("OSD: %s", lookupTableOsdDisplayPortDevice[displayPortDeviceType]);
#endif

    // Uptime and wall clock

    cliPrintf("System Uptime: %d seconds", millis() / 1000);

#ifdef USE_RTC_TIME
    char buf[FORMATTED_DATE_TIME_BUFSIZE];
    dateTime_t dt;
    if (rtcGetDateTime(&dt)) {
        dateTimeFormatLocal(buf, &dt);
        cliPrintf(", Current Time: %s", buf);
    }
#endif
    cliPrintLinefeed();

    // Run status

    const int gyroRate = getTaskDeltaTimeUs(TASK_GYRO) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTimeUs(TASK_GYRO)));
    int rxRate = getCurrentRxRefreshRate();
    if (rxRate != 0) {
        rxRate = (int)(1000000.0f / ((float)rxRate));
    }
    const int systemRate = getTaskDeltaTimeUs(TASK_SYSTEM) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTimeUs(TASK_SYSTEM)));
    cliPrintLinef("CPU:%d%%, cycle time: %d, GYRO rate: %d, RX rate: %d, System rate: %d",
            constrain(getAverageSystemLoadPercent(), 0, LOAD_PERCENTAGE_ONE), getTaskDeltaTimeUs(TASK_GYRO), gyroRate, rxRate, systemRate);

    // Battery meter

    cliPrintLinef("Voltage: %d * 0.01V (%dS battery - %s)", getBatteryVoltage(), getBatteryCellCount(), getBatteryStateString());

    // Other devices and status

#ifdef USE_I2C
    const uint16_t i2cErrorCounter = i2cGetErrorCounter();
#else
    const uint16_t i2cErrorCounter = 0;
#endif
    cliPrintLinef("I2C Errors: %d", i2cErrorCounter);

#ifdef USE_SDCARD
    cliSdInfo(cmdName, "");
#endif

    cliPrint("Arming disable flags:");
    armingDisableFlags_e flags = getArmingDisableFlags();
    while (flags) {
        const int bitpos = ffs(flags) - 1;
        flags &= ~(1 << bitpos);
        cliPrintf(" %s", armingDisableFlagNames[bitpos]);
    }
    cliPrintLinefeed();
}

#if defined(USE_TASK_STATISTICS)
static void cliTasks(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);
    int maxLoadSum = 0;
    int averageLoadSum = 0;

#ifndef MINIMAL_CLI
    if (systemConfig()->task_statistics) {
#if defined(USE_LATE_TASK_STATISTICS)
        cliPrintLine("Task list             rate/hz  max/us  avg/us maxload avgload  total/ms   late    run reqd/us");
#else
        cliPrintLine("Task list             rate/hz  max/us  avg/us maxload avgload  total/ms");
#endif
    } else {
        cliPrintLine("Task list");
    }
#endif
    for (taskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        taskInfo_t taskInfo;
        getTaskInfo(taskId, &taskInfo);
        if (taskInfo.isEnabled) {
            int taskFrequency = taskInfo.averageDeltaTimeUs == 0 ? 0 : lrintf(1e6f / taskInfo.averageDeltaTimeUs);
            cliPrintf("%02d - (%15s) ", taskId, taskInfo.taskName);
            const int maxLoad = taskInfo.maxExecutionTimeUs == 0 ? 0 :(taskInfo.maxExecutionTimeUs * taskFrequency + 5000) / 1000;
            const int averageLoad = taskInfo.averageExecutionTimeUs == 0 ? 0 : (taskInfo.averageExecutionTimeUs * taskFrequency + 5000) / 1000;
            if (taskId != TASK_SERIAL) {
                maxLoadSum += maxLoad;
                averageLoadSum += averageLoad;
            }
            if (systemConfig()->task_statistics) {
#if defined(USE_LATE_TASK_STATISTICS)
                cliPrintLinef("%6d %7d %7d %4d.%1d%% %4d.%1d%% %9d %6d %6d %7d",
                        taskFrequency, taskInfo.maxExecutionTimeUs, taskInfo.averageExecutionTimeUs,
                        maxLoad/10, maxLoad%10, averageLoad/10, averageLoad%10,
                        taskInfo.totalExecutionTimeUs / 1000,
                        taskInfo.lateCount, taskInfo.runCount, taskInfo.execTime);
#else
                cliPrintLinef("%6d %7d %7d %4d.%1d%% %4d.%1d%% %9d",
                        taskFrequency, taskInfo.maxExecutionTimeUs, taskInfo.averageExecutionTimeUs,
                        maxLoad/10, maxLoad%10, averageLoad/10, averageLoad%10,
                        taskInfo.totalExecutionTimeUs / 1000);
#endif
            } else {
                cliPrintLinef("%6d", taskFrequency);
            }

            schedulerResetTaskMaxExecutionTime(taskId);
        }
    }
    if (systemConfig()->task_statistics) {
        cfCheckFuncInfo_t checkFuncInfo;
        getCheckFuncInfo(&checkFuncInfo);
        cliPrintLinef("RX Check Function %19d %7d %25d", checkFuncInfo.maxExecutionTimeUs, checkFuncInfo.averageExecutionTimeUs, checkFuncInfo.totalExecutionTimeUs / 1000);
        cliPrintLinef("Total (excluding SERIAL) %25d.%1d%% %4d.%1d%%", maxLoadSum/10, maxLoadSum%10, averageLoadSum/10, averageLoadSum%10);
        schedulerResetCheckFunctionMaxExecutionTime();
    }
}
#endif

static void printVersion(const char *cmdName, bool printBoardInfo)
{
#if !(defined(USE_CUSTOM_DEFAULTS) && defined(USE_UNIFIED_TARGET))
    UNUSED(cmdName);
    UNUSED(printBoardInfo);
#endif

    cliPrintf("# %s / %s (%s) %s %s / %s (%s) MSP API: %s",
        FC_FIRMWARE_NAME,
        targetName,
        systemConfig()->boardIdentifier,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision,
        MSP_API_VERSION_STRING
    );

#ifdef FEATURE_CUT_LEVEL
    cliPrintLinef(" / FEATURE CUT LEVEL %d", FEATURE_CUT_LEVEL);
#else
    cliPrintLinefeed();
#endif

#if defined(USE_CUSTOM_DEFAULTS)
    if (hasCustomDefaults()) {
        if (strlen(customDefaultsManufacturerId) || strlen(customDefaultsBoardName) || strlen(customDefaultsChangesetId) || strlen(customDefaultsDate)) {
            cliPrintLinef("%s%s%s%s%s%s%s%s",
                CUSTOM_DEFAULTS_MANUFACTURER_ID_PREFIX, customDefaultsManufacturerId,
                CUSTOM_DEFAULTS_BOARD_NAME_PREFIX, customDefaultsBoardName,
                CUSTOM_DEFAULTS_CHANGESET_ID_PREFIX, customDefaultsChangesetId,
                CUSTOM_DEFAULTS_DATE_PREFIX, customDefaultsDate
            );
        } else {
            cliPrintHashLine("config: YES");
        }
    } else {
#if defined(USE_UNIFIED_TARGET)
        cliPrintError(cmdName, "NO CONFIG FOUND");
#else
        cliPrintHashLine("NO CUSTOM DEFAULTS FOUND");
#endif // USE_UNIFIED_TARGET
    }
#endif // USE_CUSTOM_DEFAULTS

#if defined(USE_UNIFIED_TARGET) && defined(USE_BOARD_INFO)
    if (printBoardInfo && strlen(getManufacturerId()) && strlen(getBoardName())) {
        cliPrintLinef("# board: manufacturer_id: %s, board_name: %s", getManufacturerId(), getBoardName());
    }
#endif
}

static void cliVersion(const char *cmdName, char *cmdline)
{
    UNUSED(cmdline);

    printVersion(cmdName, true);
}

#ifdef USE_RC_SMOOTHING_FILTER
static void cliRcSmoothing(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);
    rcSmoothingFilter_t *rcSmoothingData = getRcSmoothingData();
    cliPrint("# RC Smoothing Type: ");
    if (rxConfig()->rc_smoothing_mode) {
        cliPrintLine("FILTER");
        if (rcSmoothingAutoCalculate()) {
            const uint16_t avgRxFrameUs = rcSmoothingData->averageFrameTimeUs;
            cliPrint("# Detected RX frame rate: ");
            if (avgRxFrameUs == 0) {
                cliPrintLine("NO SIGNAL");
            } else {
                cliPrintLinef("%d.%03dms", avgRxFrameUs / 1000, avgRxFrameUs % 1000);
            }
        }
        cliPrintf("# Active setpoint cutoff: %dhz ", rcSmoothingData->setpointCutoffFrequency);
        if (rcSmoothingData->setpointCutoffSetting) {
            cliPrintLine("(manual)");
        } else {
            cliPrintLine("(auto)");
        }
        cliPrintf("# Active FF cutoff: %dhz ", rcSmoothingData->feedforwardCutoffFrequency);
        if (rcSmoothingData->ffCutoffSetting) {
            cliPrintLine("(manual)");
        } else {
            cliPrintLine("(auto)");
        }
        cliPrintf("# Active throttle cutoff: %dhz ", rcSmoothingData->throttleCutoffFrequency);
        if (rcSmoothingData->ffCutoffSetting) {
            cliPrintLine("(manual)");
        } else {
            cliPrintLine("(auto)");
        }
    } else {
        cliPrintLine("OFF");
    }
}
#endif // USE_RC_SMOOTHING_FILTER

#if defined(USE_RESOURCE_MGMT)

#define MAX_RESOURCE_INDEX(x) ((x) == 0 ? 1 : (x))

typedef struct {
    const uint8_t owner;
    pgn_t pgn;
    uint8_t stride;
    uint8_t offset;
    const uint8_t maxIndex;
} cliResourceValue_t;

// Handy macros for keeping the table tidy.
// DEFS : Single entry
// DEFA : Array of uint8_t (stride = 1)
// DEFW : Wider stride case; array of structs.

#define DEFS(owner, pgn, type, member) \
    { owner, pgn, 0, offsetof(type, member), 0 }

#define DEFA(owner, pgn, type, member, max) \
    { owner, pgn, sizeof(ioTag_t), offsetof(type, member), max }

#define DEFW(owner, pgn, type, member, max) \
    { owner, pgn, sizeof(type), offsetof(type, member), max }

const cliResourceValue_t resourceTable[] = {
#ifdef USE_BEEPER
    DEFS( OWNER_BEEPER,        PG_BEEPER_DEV_CONFIG, beeperDevConfig_t, ioTag) ,
#endif
    DEFA( OWNER_MOTOR,         PG_MOTOR_CONFIG, motorConfig_t, dev.ioTags[0], MAX_SUPPORTED_MOTORS ),
#ifdef USE_SERVOS
    DEFA( OWNER_SERVO,         PG_SERVO_CONFIG, servoConfig_t, dev.ioTags[0], MAX_SUPPORTED_SERVOS ),
#endif
#if defined(USE_PPM)
    DEFS( OWNER_PPMINPUT,      PG_PPM_CONFIG, ppmConfig_t, ioTag ),
#endif
#if defined(USE_PWM)
    DEFA( OWNER_PWMINPUT,      PG_PWM_CONFIG, pwmConfig_t, ioTags[0], PWM_INPUT_PORT_COUNT ),
#endif
#ifdef USE_RANGEFINDER_HCSR04
    DEFS( OWNER_SONAR_TRIGGER, PG_SONAR_CONFIG, sonarConfig_t, triggerTag ),
    DEFS( OWNER_SONAR_ECHO,    PG_SONAR_CONFIG, sonarConfig_t, echoTag ),
#endif
#ifdef USE_LED_STRIP
    DEFS( OWNER_LED_STRIP,     PG_LED_STRIP_CONFIG, ledStripConfig_t, ioTag ),
#endif
#ifdef USE_UART
    DEFA( OWNER_SERIAL_TX,     PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagTx[0], SERIAL_PORT_MAX_INDEX ),
    DEFA( OWNER_SERIAL_RX,     PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagRx[0], SERIAL_PORT_MAX_INDEX ),
#endif
#ifdef USE_INVERTER
    DEFA( OWNER_INVERTER,      PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagInverter[0], SERIAL_PORT_MAX_INDEX ),
#endif
#ifdef USE_I2C
    DEFW( OWNER_I2C_SCL,       PG_I2C_CONFIG, i2cConfig_t, ioTagScl, I2CDEV_COUNT ),
    DEFW( OWNER_I2C_SDA,       PG_I2C_CONFIG, i2cConfig_t, ioTagSda, I2CDEV_COUNT ),
#endif
    DEFA( OWNER_LED,           PG_STATUS_LED_CONFIG, statusLedConfig_t, ioTags[0], STATUS_LED_NUMBER ),
#ifdef USE_SPEKTRUM_BIND
    DEFS( OWNER_RX_BIND,       PG_RX_CONFIG, rxConfig_t, spektrum_bind_pin_override_ioTag ),
    DEFS( OWNER_RX_BIND_PLUG,  PG_RX_CONFIG, rxConfig_t, spektrum_bind_plug_ioTag ),
#endif
#ifdef USE_TRANSPONDER
    DEFS( OWNER_TRANSPONDER,   PG_TRANSPONDER_CONFIG, transponderConfig_t, ioTag ),
#endif
#ifdef USE_SPI
    DEFW( OWNER_SPI_SCK,       PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagSck, SPIDEV_COUNT ),
    DEFW( OWNER_SPI_MISO,      PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagMiso, SPIDEV_COUNT ),
    DEFW( OWNER_SPI_MOSI,      PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagMosi, SPIDEV_COUNT ),
#endif
#ifdef USE_ESCSERIAL
    DEFS( OWNER_ESCSERIAL,     PG_ESCSERIAL_CONFIG, escSerialConfig_t, ioTag ),
#endif
#ifdef USE_CAMERA_CONTROL
    DEFS( OWNER_CAMERA_CONTROL, PG_CAMERA_CONTROL_CONFIG, cameraControlConfig_t, ioTag ),
#endif
#ifdef USE_ADC
    DEFS( OWNER_ADC_BATT,      PG_ADC_CONFIG, adcConfig_t, vbat.ioTag ),
    DEFS( OWNER_ADC_RSSI,      PG_ADC_CONFIG, adcConfig_t, rssi.ioTag ),
    DEFS( OWNER_ADC_CURR,      PG_ADC_CONFIG, adcConfig_t, current.ioTag ),
    DEFS( OWNER_ADC_EXT,       PG_ADC_CONFIG, adcConfig_t, external1.ioTag ),
#endif
#ifdef USE_BARO
    DEFS( OWNER_BARO_CS,       PG_BAROMETER_CONFIG, barometerConfig_t, baro_spi_csn ),
    DEFS( OWNER_BARO_EOC,      PG_BAROMETER_CONFIG, barometerConfig_t, baro_eoc_tag ),
    DEFS( OWNER_BARO_XCLR,     PG_BAROMETER_CONFIG, barometerConfig_t, baro_xclr_tag ),
#endif
#ifdef USE_MAG
    DEFS( OWNER_COMPASS_CS,    PG_COMPASS_CONFIG, compassConfig_t, mag_spi_csn ),
#ifdef USE_MAG_DATA_READY_SIGNAL
    DEFS( OWNER_COMPASS_EXTI,  PG_COMPASS_CONFIG, compassConfig_t, interruptTag ),
#endif
#endif
#ifdef USE_SDCARD_SPI
    DEFS( OWNER_SDCARD_CS,     PG_SDCARD_CONFIG, sdcardConfig_t, chipSelectTag ),
#endif
#ifdef USE_SDCARD
    DEFS( OWNER_SDCARD_DETECT, PG_SDCARD_CONFIG, sdcardConfig_t, cardDetectTag ),
#endif
#if defined(STM32H7) && defined(USE_SDCARD_SDIO)
    DEFS( OWNER_SDIO_CK,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, CKPin ),
    DEFS( OWNER_SDIO_CMD,      PG_SDIO_PIN_CONFIG, sdioPinConfig_t, CMDPin ),
    DEFS( OWNER_SDIO_D0,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, D0Pin ),
    DEFS( OWNER_SDIO_D1,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, D1Pin ),
    DEFS( OWNER_SDIO_D2,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, D2Pin ),
    DEFS( OWNER_SDIO_D3,       PG_SDIO_PIN_CONFIG, sdioPinConfig_t, D3Pin ),
#endif
#ifdef USE_PINIO
    DEFA( OWNER_PINIO,         PG_PINIO_CONFIG, pinioConfig_t, ioTag, PINIO_COUNT ),
#endif
#if defined(USE_USB_MSC)
    DEFS( OWNER_USB_MSC_PIN,   PG_USB_CONFIG, usbDev_t, mscButtonPin ),
#endif
#ifdef USE_FLASH_CHIP
    DEFS( OWNER_FLASH_CS,      PG_FLASH_CONFIG, flashConfig_t, csTag ),
#endif
#ifdef USE_MAX7456
    DEFS( OWNER_OSD_CS,        PG_MAX7456_CONFIG, max7456Config_t, csTag ),
#endif
#ifdef USE_RX_SPI
    DEFS( OWNER_RX_SPI_CS,     PG_RX_SPI_CONFIG, rxSpiConfig_t, csnTag ),
    DEFS( OWNER_RX_SPI_EXTI,   PG_RX_SPI_CONFIG, rxSpiConfig_t, extiIoTag ),
    DEFS( OWNER_RX_SPI_BIND,   PG_RX_SPI_CONFIG, rxSpiConfig_t, bindIoTag ),
    DEFS( OWNER_RX_SPI_LED,    PG_RX_SPI_CONFIG, rxSpiConfig_t, ledIoTag ),
#if defined(USE_RX_CC2500) && defined(USE_RX_CC2500_SPI_PA_LNA)
    DEFS( OWNER_RX_SPI_CC2500_TX_EN,   PG_RX_CC2500_SPI_CONFIG, rxCc2500SpiConfig_t, txEnIoTag ),
    DEFS( OWNER_RX_SPI_CC2500_LNA_EN,  PG_RX_CC2500_SPI_CONFIG, rxCc2500SpiConfig_t, lnaEnIoTag ),
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
    DEFS( OWNER_RX_SPI_CC2500_ANT_SEL, PG_RX_CC2500_SPI_CONFIG, rxCc2500SpiConfig_t, antSelIoTag ),
#endif
#endif
#endif
#ifdef USE_GYRO_EXTI
    DEFW( OWNER_GYRO_EXTI,     PG_GYRO_DEVICE_CONFIG, gyroDeviceConfig_t, extiTag, MAX_GYRODEV_COUNT ),
#endif
    DEFW( OWNER_GYRO_CS,       PG_GYRO_DEVICE_CONFIG, gyroDeviceConfig_t, csnTag, MAX_GYRODEV_COUNT ),
#ifdef USE_USB_DETECT
    DEFS( OWNER_USB_DETECT,    PG_USB_CONFIG, usbDev_t, detectPin ),
#endif
#ifdef USE_VTX_RTC6705
    DEFS( OWNER_VTX_POWER,     PG_VTX_IO_CONFIG, vtxIOConfig_t, powerTag ),
    DEFS( OWNER_VTX_CS,        PG_VTX_IO_CONFIG, vtxIOConfig_t, csTag ),
    DEFS( OWNER_VTX_DATA,      PG_VTX_IO_CONFIG, vtxIOConfig_t, dataTag ),
    DEFS( OWNER_VTX_CLK,       PG_VTX_IO_CONFIG, vtxIOConfig_t, clockTag ),
#endif
#ifdef USE_PIN_PULL_UP_DOWN
    DEFA( OWNER_PULLUP,        PG_PULLUP_CONFIG,   pinPullUpDownConfig_t, ioTag, PIN_PULL_UP_DOWN_COUNT ),
    DEFA( OWNER_PULLDOWN,      PG_PULLDOWN_CONFIG, pinPullUpDownConfig_t, ioTag, PIN_PULL_UP_DOWN_COUNT ),
#endif
};

#undef DEFS
#undef DEFA
#undef DEFW

static ioTag_t *getIoTag(const cliResourceValue_t value, uint8_t index)
{
    const pgRegistry_t* rec = pgFind(value.pgn);
    return CONST_CAST(ioTag_t *, rec->address + value.stride * index + value.offset);
}

static void printResource(dumpFlags_t dumpMask, const char *headingStr)
{
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (unsigned int i = 0; i < ARRAYLEN(resourceTable); i++) {
        const char* owner = ownerNames[resourceTable[i].owner];
        const pgRegistry_t* pg = pgFind(resourceTable[i].pgn);
        const void *currentConfig;
        const void *defaultConfig;
        if (isReadingConfigFromCopy()) {
            currentConfig = pg->copy;
            defaultConfig = pg->address;
        } else {
            currentConfig = pg->address;
            defaultConfig = NULL;
        }

        for (int index = 0; index < MAX_RESOURCE_INDEX(resourceTable[i].maxIndex); index++) {
            const ioTag_t ioTag = *(ioTag_t *)((const uint8_t *)currentConfig + resourceTable[i].stride * index + resourceTable[i].offset);
            ioTag_t ioTagDefault = NULL;
            if (defaultConfig) {
                ioTagDefault = *(ioTag_t *)((const uint8_t *)defaultConfig + resourceTable[i].stride * index + resourceTable[i].offset);
            }

            const bool equalsDefault = ioTag == ioTagDefault;
            const char *format = "resource %s %d %c%02d";
            const char *formatUnassigned = "resource %s %d NONE";
            headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
            if (ioTagDefault) {
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, owner, RESOURCE_INDEX(index), IO_GPIOPortIdxByTag(ioTagDefault) + 'A', IO_GPIOPinIdxByTag(ioTagDefault));
            } else if (defaultConfig) {
                cliDefaultPrintLinef(dumpMask, equalsDefault, formatUnassigned, owner, RESOURCE_INDEX(index));
            }
            if (ioTag) {
                cliDumpPrintLinef(dumpMask, equalsDefault, format, owner, RESOURCE_INDEX(index), IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag));
            } else if (!(dumpMask & HIDE_UNUSED)) {
                cliDumpPrintLinef(dumpMask, equalsDefault, formatUnassigned, owner, RESOURCE_INDEX(index));
            }
        }
    }
}

static void printResourceOwner(uint8_t owner, uint8_t index)
{
    cliPrintf("%s", ownerNames[resourceTable[owner].owner]);

    if (resourceTable[owner].maxIndex > 0) {
        cliPrintf(" %d", RESOURCE_INDEX(index));
    }
}

static void resourceCheck(uint8_t resourceIndex, uint8_t index, ioTag_t newTag)
{
    if (!newTag) {
        return;
    }

    const char * format = "\r\nNOTE: %c%02d already assigned to ";
    for (int r = 0; r < (int)ARRAYLEN(resourceTable); r++) {
        for (int i = 0; i < MAX_RESOURCE_INDEX(resourceTable[r].maxIndex); i++) {
            ioTag_t *tag = getIoTag(resourceTable[r], i);
            if (*tag == newTag) {
                bool cleared = false;
                if (r == resourceIndex) {
                    if (i == index) {
                        continue;
                    }
                    *tag = IO_TAG_NONE;
                    cleared = true;
                }

                cliPrintf(format, DEFIO_TAG_GPIOID(newTag) + 'A', DEFIO_TAG_PIN(newTag));

                printResourceOwner(r, i);

                if (cleared) {
                    cliPrintf(". ");
                    printResourceOwner(r, i);
                    cliPrintf(" disabled");
                }

                cliPrintLine(".");
            }
        }
    }
}

static bool strToPin(char *ptr, ioTag_t *tag)
{
    if (strcasecmp(ptr, "NONE") == 0) {
        *tag = IO_TAG_NONE;

        return true;
    } else {
        const unsigned port = (*ptr >= 'a') ? *ptr - 'a' : *ptr - 'A';
        if (port < 8) {
            ptr++;

            char *end;
            const long pin = strtol(ptr, &end, 10);
            if (end != ptr && pin >= 0 && pin < 16) {
                *tag = DEFIO_TAG_MAKE(port, pin);

                return true;
            }
        }
    }

    return false;
}

#ifdef USE_DMA
static void showDma(void)
{
    cliPrintLinefeed();

#ifdef MINIMAL_CLI
    cliPrintLine("DMA:");
#else
    cliPrintLine("Currently active DMA:");
    cliRepeat('-', 20);
#endif
    for (int i = 1; i <= DMA_LAST_HANDLER; i++) {
        const resourceOwner_t *owner = dmaGetOwner(i);

        cliPrintf(DMA_OUTPUT_STRING, DMA_DEVICE_NO(i), DMA_DEVICE_INDEX(i));
        if (owner->resourceIndex > 0) {
            cliPrintLinef(" %s %d", ownerNames[owner->owner], owner->resourceIndex);
        } else {
            cliPrintLinef(" %s", ownerNames[owner->owner]);
        }
    }
}
#endif

#ifdef USE_DMA_SPEC

typedef struct dmaoptEntry_s {
    char *device;
    dmaPeripheral_e peripheral;
    pgn_t pgn;
    uint8_t stride;
    uint8_t offset;
    uint8_t maxIndex;
    uint32_t presenceMask;
} dmaoptEntry_t;

#define MASK_IGNORED (0)

// Handy macros for keeping the table tidy.
// DEFS : Single entry
// DEFA : Array of uint8_t (stride = 1)
// DEFW : Wider stride case; array of structs.

#define DEFS(device, peripheral, pgn, type, member) \
    { device, peripheral, pgn, 0,               offsetof(type, member), 0, MASK_IGNORED }

#define DEFA(device, peripheral, pgn, type, member, max, mask) \
    { device, peripheral, pgn, sizeof(uint8_t), offsetof(type, member), max, mask }

#define DEFW(device, peripheral, pgn, type, member, max, mask) \
    { device, peripheral, pgn, sizeof(type), offsetof(type, member), max, mask }

dmaoptEntry_t dmaoptEntryTable[] = {
    DEFW("SPI_TX",  DMA_PERIPH_SPI_TX,  PG_SPI_PIN_CONFIG,     spiPinConfig_t,     txDmaopt, SPIDEV_COUNT,                    MASK_IGNORED),
    DEFW("SPI_RX",  DMA_PERIPH_SPI_RX,  PG_SPI_PIN_CONFIG,     spiPinConfig_t,     rxDmaopt, SPIDEV_COUNT,                    MASK_IGNORED),
    DEFA("ADC",     DMA_PERIPH_ADC,     PG_ADC_CONFIG,         adcConfig_t,        dmaopt,   ADCDEV_COUNT,                    MASK_IGNORED),
    DEFS("SDIO",    DMA_PERIPH_SDIO,    PG_SDIO_CONFIG,        sdioConfig_t,       dmaopt),
    DEFW("UART_TX", DMA_PERIPH_UART_TX, PG_SERIAL_UART_CONFIG, serialUartConfig_t, txDmaopt, UARTDEV_CONFIG_MAX,              MASK_IGNORED),
    DEFW("UART_RX", DMA_PERIPH_UART_RX, PG_SERIAL_UART_CONFIG, serialUartConfig_t, rxDmaopt, UARTDEV_CONFIG_MAX,              MASK_IGNORED),
#if defined(STM32H7) || defined(STM32G4)
    DEFW("TIMUP",   DMA_PERIPH_TIMUP,   PG_TIMER_UP_CONFIG,    timerUpConfig_t,    dmaopt,   HARDWARE_TIMER_DEFINITION_COUNT, TIMUP_TIMERS),
#endif
};

#undef DEFS
#undef DEFA
#undef DEFW

#define DMA_OPT_UI_INDEX(i) ((i) + 1)
#define DMA_OPT_STRING_BUFSIZE 5

#if defined(STM32H7) || defined(STM32G4)
#define DMA_CHANREQ_STRING "Request"
#else
#define DMA_CHANREQ_STRING "Channel"
#endif

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
#define DMA_STCH_STRING    "Stream"
#else
#define DMA_STCH_STRING    "Channel"
#endif

#define DMASPEC_FORMAT_STRING "DMA%d " DMA_STCH_STRING " %d " DMA_CHANREQ_STRING " %d"

static void optToString(int optval, char *buf)
{
    if (optval == DMA_OPT_UNUSED) {
        memcpy(buf, "NONE", DMA_OPT_STRING_BUFSIZE);
    } else {
        tfp_sprintf(buf, "%d", optval);
    }
}

static void printPeripheralDmaoptDetails(dmaoptEntry_t *entry, int index, const dmaoptValue_t dmaopt, const bool equalsDefault, const dumpFlags_t dumpMask, printFn *printValue)
{
    // We compute number to display for different peripherals in advance.
    // This is done to deal with TIMUP which numbered non-contiguously.
    // Note that using timerGetNumberByIndex is not a generic solution,
    // but we are lucky that TIMUP is the only peripheral with non-contiguous numbering.

    int uiIndex;

    if (entry->presenceMask) {
        uiIndex = timerGetNumberByIndex(index);
    } else {
        uiIndex = DMA_OPT_UI_INDEX(index);
    }

    if (dmaopt != DMA_OPT_UNUSED) {
        printValue(dumpMask, equalsDefault,
            "dma %s %d %d",
            entry->device, uiIndex, dmaopt);

        const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByPeripheral(entry->peripheral, index, dmaopt);
        dmaCode_t dmaCode = 0;
        if (dmaChannelSpec) {
            dmaCode = dmaChannelSpec->code;
        }
        printValue(dumpMask, equalsDefault,
            "# %s %d: " DMASPEC_FORMAT_STRING,
            entry->device, uiIndex, DMA_CODE_CONTROLLER(dmaCode), DMA_CODE_STREAM(dmaCode), DMA_CODE_CHANNEL(dmaCode));
    } else if (!(dumpMask & HIDE_UNUSED)) {
        printValue(dumpMask, equalsDefault,
            "dma %s %d NONE",
            entry->device, uiIndex);
    }
}

static const char *printPeripheralDmaopt(dmaoptEntry_t *entry, int index, dumpFlags_t dumpMask, const char *headingStr)
{
    const pgRegistry_t* pg = pgFind(entry->pgn);
    const void *currentConfig;
    const void *defaultConfig;

    if (isReadingConfigFromCopy()) {
        currentConfig = pg->copy;
        defaultConfig = pg->address;
    } else {
        currentConfig = pg->address;
        defaultConfig = NULL;
    }

    dmaoptValue_t currentOpt = *(dmaoptValue_t *)((uint8_t *)currentConfig + entry->stride * index + entry->offset);
    dmaoptValue_t defaultOpt;

    if (defaultConfig) {
        defaultOpt = *(dmaoptValue_t *)((uint8_t *)defaultConfig + entry->stride * index + entry->offset);
    } else {
        defaultOpt = DMA_OPT_UNUSED;
    }

    bool equalsDefault = currentOpt == defaultOpt;
    headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);

    if (defaultConfig) {
        printPeripheralDmaoptDetails(entry, index, defaultOpt, equalsDefault, dumpMask, cliDefaultPrintLinef);
    }

    printPeripheralDmaoptDetails(entry, index, currentOpt, equalsDefault, dumpMask, cliDumpPrintLinef);
    return headingStr;
}

#if defined(USE_TIMER_MGMT)
static void printTimerDmaoptDetails(const ioTag_t ioTag, const timerHardware_t *timer, const dmaoptValue_t dmaopt, const bool equalsDefault, const dumpFlags_t dumpMask, printFn *printValue)
{
    const char *format = "dma pin %c%02d %d";

    if (dmaopt != DMA_OPT_UNUSED) {
        const bool printDetails = printValue(dumpMask, equalsDefault, format,
            IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag),
            dmaopt
        );

        if (printDetails) {
            const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, dmaopt);
            dmaCode_t dmaCode = 0;
            if (dmaChannelSpec) {
                dmaCode = dmaChannelSpec->code;
                printValue(dumpMask, false,
                    "# pin %c%02d: " DMASPEC_FORMAT_STRING,
                    IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag),
                    DMA_CODE_CONTROLLER(dmaCode), DMA_CODE_STREAM(dmaCode), DMA_CODE_CHANNEL(dmaCode)
                );
            }
        }
    } else if (!(dumpMask & HIDE_UNUSED)) {
        printValue(dumpMask, equalsDefault,
            "dma pin %c%02d NONE",
            IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag)
        );
    }
}

static const char *printTimerDmaopt(const timerIOConfig_t *currentConfig, const timerIOConfig_t *defaultConfig, unsigned index, dumpFlags_t dumpMask, bool tagsInUse[], const char *headingStr)
{
    const ioTag_t ioTag = currentConfig[index].ioTag;

    if (!ioTag) {
        return headingStr;
    }

    const timerHardware_t *timer = timerGetByTagAndIndex(ioTag, currentConfig[index].index);
    const dmaoptValue_t dmaopt = currentConfig[index].dmaopt;

    dmaoptValue_t defaultDmaopt = DMA_OPT_UNUSED;
    bool equalsDefault = defaultDmaopt == dmaopt;
    if (defaultConfig) {
        for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
            if (defaultConfig[i].ioTag == ioTag) {
                defaultDmaopt = defaultConfig[i].dmaopt;

                // We need to check timer as well here to get 'default' DMA options for non-default timers printed, because setting the timer resets the DMA option.
                equalsDefault = (defaultDmaopt == dmaopt) && (defaultConfig[i].index == currentConfig[index].index || dmaopt == DMA_OPT_UNUSED);

                tagsInUse[index] = true;

                break;
            }
        }
    }

    headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);

    if (defaultConfig) {
        printTimerDmaoptDetails(ioTag, timer, defaultDmaopt, equalsDefault, dumpMask, cliDefaultPrintLinef);
    }

    printTimerDmaoptDetails(ioTag, timer, dmaopt, equalsDefault, dumpMask, cliDumpPrintLinef);
    return headingStr;
}
#endif

static void printDmaopt(dumpFlags_t dumpMask, const char *headingStr)
{
    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    for (size_t i = 0; i < ARRAYLEN(dmaoptEntryTable); i++) {
        dmaoptEntry_t *entry = &dmaoptEntryTable[i];
        for (int index = 0; index < entry->maxIndex; index++) {
            headingStr = printPeripheralDmaopt(entry, index, dumpMask, headingStr);
        }
    }

#if defined(USE_TIMER_MGMT)
    const pgRegistry_t* pg = pgFind(PG_TIMER_IO_CONFIG);
    const timerIOConfig_t *currentConfig;
    const timerIOConfig_t *defaultConfig;

    if (isReadingConfigFromCopy()) {
        currentConfig = (timerIOConfig_t *)pg->copy;
        defaultConfig = (timerIOConfig_t *)pg->address;
    } else {
        currentConfig = (timerIOConfig_t *)pg->address;
        defaultConfig = NULL;
    }

    bool tagsInUse[MAX_TIMER_PINMAP_COUNT] = { false };
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        headingStr = printTimerDmaopt(currentConfig, defaultConfig, i, dumpMask, tagsInUse, headingStr);
    }

    if (defaultConfig) {
        for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
            if (!tagsInUse[i] && defaultConfig[i].ioTag && defaultConfig[i].dmaopt != DMA_OPT_UNUSED) {
                const timerHardware_t *timer = timerGetByTagAndIndex(defaultConfig[i].ioTag, defaultConfig[i].index);
                headingStr = cliPrintSectionHeading(dumpMask, true, headingStr);
                printTimerDmaoptDetails(defaultConfig[i].ioTag, timer, defaultConfig[i].dmaopt, false, dumpMask, cliDefaultPrintLinef);

                printTimerDmaoptDetails(defaultConfig[i].ioTag, timer, DMA_OPT_UNUSED, false, dumpMask, cliDumpPrintLinef);
            }
        }
    }
#endif
}

static void cliDmaopt(const char *cmdName, char *cmdline)
{
    char *pch = NULL;
    char *saveptr;

    // Peripheral name or command option
    pch = strtok_r(cmdline, " ", &saveptr);
    if (!pch) {
        printDmaopt(DUMP_MASTER | HIDE_UNUSED, NULL);

        return;
    } else if (strcasecmp(pch, "list") == 0) {
        cliPrintErrorLinef(cmdName, "NOT IMPLEMENTED YET");

        return;
    }

    dmaoptEntry_t *entry = NULL;
    for (unsigned i = 0; i < ARRAYLEN(dmaoptEntryTable); i++) {
        if (strcasecmp(pch, dmaoptEntryTable[i].device) == 0) {
            entry = &dmaoptEntryTable[i];
        }
    }

    if (!entry && strcasecmp(pch, "pin") != 0) {
        cliPrintErrorLinef(cmdName, "BAD DEVICE: %s", pch);
        return;
    }

    // Index
    dmaoptValue_t orgval = DMA_OPT_UNUSED;

    int index = 0;
    dmaoptValue_t *optaddr = NULL;

    ioTag_t ioTag = IO_TAG_NONE;
#if defined(USE_TIMER_MGMT)
    timerIOConfig_t *timerIoConfig = NULL;
#endif
    const timerHardware_t *timer = NULL;
    pch = strtok_r(NULL, " ", &saveptr);
    if (entry) {
        index = atoi(pch) - 1;
        if (index < 0 || index >= entry->maxIndex || (entry->presenceMask != MASK_IGNORED && !(entry->presenceMask & BIT(index + 1)))) {
            cliPrintErrorLinef(cmdName, "BAD INDEX: '%s'", pch ? pch : "");
            return;
        }

        const pgRegistry_t* pg = pgFind(entry->pgn);
        const void *currentConfig;
        if (isWritingConfigToCopy()) {
            currentConfig = pg->copy;
        } else {
            currentConfig = pg->address;
        }
        optaddr = (dmaoptValue_t *)((uint8_t *)currentConfig + entry->stride * index + entry->offset);
        orgval = *optaddr;
    } else {
        // It's a pin
        if (!pch || !(strToPin(pch, &ioTag) && IOGetByTag(ioTag))) {
            cliPrintErrorLinef(cmdName, "INVALID PIN: '%s'", pch ? pch : "");

            return;
        }

        orgval = dmaoptByTag(ioTag);
#if defined(USE_TIMER_MGMT)
        timerIoConfig = timerIoConfigByTag(ioTag);
#endif
        timer = timerGetByTag(ioTag);
    }

    // opt or list
    pch = strtok_r(NULL, " ", &saveptr);
    if (!pch) {
        if (entry) {
            printPeripheralDmaoptDetails(entry, index, *optaddr, true, DUMP_MASTER, cliDumpPrintLinef);
        }
#if defined(USE_TIMER_MGMT)
        else {
            printTimerDmaoptDetails(ioTag, timer, orgval, true, DUMP_MASTER, cliDumpPrintLinef);
        }
#endif

        return;
    } else if (strcasecmp(pch, "list") == 0) {
        // Show possible opts
        const dmaChannelSpec_t *dmaChannelSpec;
        if (entry) {
            for (int opt = 0; (dmaChannelSpec = dmaGetChannelSpecByPeripheral(entry->peripheral, index, opt)); opt++) {
                cliPrintLinef("# %d: " DMASPEC_FORMAT_STRING, opt, DMA_CODE_CONTROLLER(dmaChannelSpec->code), DMA_CODE_STREAM(dmaChannelSpec->code), DMA_CODE_CHANNEL(dmaChannelSpec->code));
            }
        } else {
            for (int opt = 0; (dmaChannelSpec = dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, opt)); opt++) {
                cliPrintLinef("# %d: " DMASPEC_FORMAT_STRING, opt, DMA_CODE_CONTROLLER(dmaChannelSpec->code), DMA_CODE_STREAM(dmaChannelSpec->code), DMA_CODE_CHANNEL(dmaChannelSpec->code));
            }
        }

        return;
    } else if (pch) {
        int optval;
        if (strcasecmp(pch, "none") == 0) {
            optval = DMA_OPT_UNUSED;
        } else {
            optval = atoi(pch);

            if (entry) {
                if (!dmaGetChannelSpecByPeripheral(entry->peripheral, index, optval)) {
                    cliPrintErrorLinef(cmdName, "INVALID DMA OPTION FOR %s %d: '%s'", entry->device, DMA_OPT_UI_INDEX(index), pch);

                    return;
                }
            } else {
                if (!dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, optval)) {
                    cliPrintErrorLinef(cmdName, "INVALID DMA OPTION FOR PIN %c%02d: '%s'", IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag), pch);

                    return;
                }
            }
        }

        char optvalString[DMA_OPT_STRING_BUFSIZE];
        optToString(optval, optvalString);

        char orgvalString[DMA_OPT_STRING_BUFSIZE];
        optToString(orgval, orgvalString);

        if (optval != orgval) {
            if (entry) {
                *optaddr = optval;

                cliPrintLinef("# dma %s %d: changed from %s to %s", entry->device, DMA_OPT_UI_INDEX(index), orgvalString, optvalString);
            } else {
#if defined(USE_TIMER_MGMT)
                timerIoConfig->dmaopt = optval;
#endif

                cliPrintLinef("# dma pin %c%02d: changed from %s to %s", IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag), orgvalString, optvalString);
            }
        } else {
            if (entry) {
                cliPrintLinef("# dma %s %d: no change: %s", entry->device, DMA_OPT_UI_INDEX(index), orgvalString);
            } else {
                cliPrintLinef("# dma %c%02d: no change: %s", IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag),orgvalString);
            }
        }
    }
}
#endif // USE_DMA_SPEC

#ifdef USE_DMA
static void cliDma(const char *cmdName, char* cmdline)
{
    int len = strlen(cmdline);
    if (len && strncasecmp(cmdline, "show", len) == 0) {
        showDma();

        return;
    }

#if defined(USE_DMA_SPEC)
    cliDmaopt(cmdName, cmdline);
#else
    cliShowParseError(cmdName);
#endif
}
#endif
#endif // USE_RESOURCE_MGMT

#ifdef USE_TIMER_MGMT
static void printTimerDetails(const ioTag_t ioTag, const unsigned timerIndex, const bool equalsDefault, const dumpFlags_t dumpMask, printFn *printValue)
{
    const char *format = "timer %c%02d AF%d";
    const char *emptyFormat = "timer %c%02d NONE";

    if (timerIndex > 0) {
        const timerHardware_t *timer = timerGetByTagAndIndex(ioTag, timerIndex);
        const bool printDetails = printValue(dumpMask, equalsDefault, format,
            IO_GPIOPortIdxByTag(ioTag) + 'A',
            IO_GPIOPinIdxByTag(ioTag),
            timer->alternateFunction
        );
        if (printDetails) {
            printValue(dumpMask, false,
                "# pin %c%02d: TIM%d CH%d%s (AF%d)",
                IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag),
                timerGetTIMNumber(timer->tim),
                CC_INDEX_FROM_CHANNEL(timer->channel) + 1,
                timer->output & TIMER_OUTPUT_N_CHANNEL ? "N" : "",
                timer->alternateFunction
            );
        }
    } else {
        printValue(dumpMask, equalsDefault, emptyFormat,
            IO_GPIOPortIdxByTag(ioTag) + 'A',
            IO_GPIOPinIdxByTag(ioTag)
        );
    }
}

static void printTimer(dumpFlags_t dumpMask, const char *headingStr)
{
    const pgRegistry_t* pg = pgFind(PG_TIMER_IO_CONFIG);
    const timerIOConfig_t *currentConfig;
    const timerIOConfig_t *defaultConfig;

    headingStr = cliPrintSectionHeading(dumpMask, false, headingStr);
    if (isReadingConfigFromCopy()) {
        currentConfig = (timerIOConfig_t *)pg->copy;
        defaultConfig = (timerIOConfig_t *)pg->address;
    } else {
        currentConfig = (timerIOConfig_t *)pg->address;
        defaultConfig = NULL;
    }

    bool tagsInUse[MAX_TIMER_PINMAP_COUNT] = { false };
    for (unsigned int i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        const ioTag_t ioTag = currentConfig[i].ioTag;

        if (!ioTag) {
            continue;
        }

        const uint8_t timerIndex = currentConfig[i].index;

        uint8_t defaultTimerIndex = 0;
        if (defaultConfig) {
            for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
                if (defaultConfig[i].ioTag == ioTag) {
                    defaultTimerIndex = defaultConfig[i].index;
                    tagsInUse[i] = true;

                    break;
                }
            }
        }

        const bool equalsDefault = defaultTimerIndex == timerIndex;
        headingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, headingStr);
        if (defaultConfig && defaultTimerIndex) {
            printTimerDetails(ioTag, defaultTimerIndex, equalsDefault, dumpMask, cliDefaultPrintLinef);
        }

        printTimerDetails(ioTag, timerIndex, equalsDefault, dumpMask, cliDumpPrintLinef);
    }

    if (defaultConfig) {
        for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
            if (!tagsInUse[i] && defaultConfig[i].ioTag) {
                headingStr = cliPrintSectionHeading(DO_DIFF, true, headingStr);
                printTimerDetails(defaultConfig[i].ioTag, defaultConfig[i].index, false, dumpMask, cliDefaultPrintLinef);

                printTimerDetails(defaultConfig[i].ioTag, 0, false, dumpMask, cliDumpPrintLinef);
            }
        }
    }
}

#define TIMER_INDEX_UNDEFINED -1
#define TIMER_AF_STRING_BUFSIZE 5

static void alternateFunctionToString(const ioTag_t ioTag, const int index, char *buf)
{
    const timerHardware_t *timer = timerGetByTagAndIndex(ioTag, index + 1);
    if (!timer) {
        memcpy(buf, "NONE", TIMER_AF_STRING_BUFSIZE);
    } else {
        tfp_sprintf(buf, "AF%d", timer->alternateFunction);
    }
}

static void showTimers(void)
{
    cliPrintLinefeed();

#ifdef MINIMAL_CLI
    cliPrintLine("Timers:");
#else
    cliPrintLine("Currently active Timers:");
    cliRepeat('-', 23);
#endif

    int8_t timerNumber;
    for (int i = 0; (timerNumber = timerGetNumberByIndex(i)); i++) {
        cliPrintf("TIM%d:", timerNumber);
        bool timerUsed = false;
        for (unsigned timerIndex = 0; timerIndex < CC_CHANNELS_PER_TIMER; timerIndex++) {
            const resourceOwner_t *timerOwner = timerGetOwner(timerNumber, CC_CHANNEL_FROM_INDEX(timerIndex));
            if (timerOwner->owner) {
                if (!timerUsed) {
                    timerUsed = true;

                    cliPrintLinefeed();
                }

                if (timerOwner->resourceIndex > 0) {
                    cliPrintLinef("    CH%d: %s %d", timerIndex + 1, ownerNames[timerOwner->owner], timerOwner->resourceIndex);
                } else {
                    cliPrintLinef("    CH%d: %s", timerIndex + 1, ownerNames[timerOwner->owner]);
                }
            }
        }

        if (!timerUsed) {
            cliPrintLine(" FREE");
        }
    }
}

static void cliTimer(const char *cmdName, char *cmdline)
{
    int len = strlen(cmdline);

    if (len == 0) {
        printTimer(DUMP_MASTER, NULL);

        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrintErrorLinef(cmdName, "NOT IMPLEMENTED YET");

        return;
    } else if (strncasecmp(cmdline, "show", len) == 0) {
        showTimers();

        return;
    }

    char *pch = NULL;
    char *saveptr;

    ioTag_t ioTag = IO_TAG_NONE;
    pch = strtok_r(cmdline, " ", &saveptr);
    if (!pch || !strToPin(pch, &ioTag)) {
        cliShowParseError(cmdName);

        return;
    } else if (!IOGetByTag(ioTag)) {
        cliPrintErrorLinef(cmdName, "PIN NOT USED ON BOARD.");

        return;
    }

    int timerIOIndex = TIMER_INDEX_UNDEFINED;
    bool isExistingTimerOpt = false;
    /* find existing entry, or go for next available */
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            timerIOIndex = i;
            isExistingTimerOpt = true;

            break;
        }

        /* first available empty slot */
        if (timerIOIndex < 0 && timerIOConfig(i)->ioTag == IO_TAG_NONE) {
            timerIOIndex = i;
        }
    }

    if (timerIOIndex < 0) {
        cliPrintErrorLinef(cmdName, "PIN TIMER MAP FULL.");

        return;
    }

    pch = strtok_r(NULL, " ", &saveptr);
    if (pch) {
        int timerIndex = TIMER_INDEX_UNDEFINED;
        if (strcasecmp(pch, "list") == 0) {
            /* output the list of available options */
            const timerHardware_t *timer;
            for (unsigned index = 0; (timer = timerGetByTagAndIndex(ioTag, index + 1)); index++) {
                cliPrintLinef("# AF%d: TIM%d CH%d%s",
                    timer->alternateFunction,
                    timerGetTIMNumber(timer->tim),
                    CC_INDEX_FROM_CHANNEL(timer->channel) + 1,
                    timer->output & TIMER_OUTPUT_N_CHANNEL ? "N" : ""
                );
            }

            return;
        } else if (strncasecmp(pch, "af", 2) == 0) {
            unsigned alternateFunction = atoi(&pch[2]);

            const timerHardware_t *timer;
            for (unsigned index = 0; (timer = timerGetByTagAndIndex(ioTag, index + 1)); index++) {
                if (timer->alternateFunction == alternateFunction) {
                    timerIndex = index;

                    break;
                }
            }

            if (!timer) {
                cliPrintErrorLinef(cmdName, "INVALID ALTERNATE FUNCTION FOR %c%02d: '%s'", IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag), pch);

                return;
            }
        } else if (strcasecmp(pch, "none") != 0) {
            cliPrintErrorLinef(cmdName, "INVALID TIMER OPTION FOR %c%02d: '%s'", IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag), pch);

            return;
        }

        int oldTimerIndex = isExistingTimerOpt ? timerIOConfig(timerIOIndex)->index - 1 : -1;
        timerIOConfigMutable(timerIOIndex)->ioTag = timerIndex == TIMER_INDEX_UNDEFINED ? IO_TAG_NONE : ioTag;
        timerIOConfigMutable(timerIOIndex)->index = timerIndex + 1;
        timerIOConfigMutable(timerIOIndex)->dmaopt = DMA_OPT_UNUSED;

        char optvalString[DMA_OPT_STRING_BUFSIZE];
        alternateFunctionToString(ioTag, timerIndex, optvalString);

        char orgvalString[DMA_OPT_STRING_BUFSIZE];
        alternateFunctionToString(ioTag, oldTimerIndex, orgvalString);

        if (timerIndex == oldTimerIndex) {
            cliPrintLinef("# timer %c%02d: no change: %s", IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag), orgvalString);
        } else {
            cliPrintLinef("# timer %c%02d: changed from %s to %s", IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag), orgvalString, optvalString);
        }

        return;
    } else {
        printTimerDetails(ioTag, timerIOConfig(timerIOIndex)->index, false, DUMP_MASTER, cliDumpPrintLinef);

        return;
    }
}
#endif

#if defined(USE_RESOURCE_MGMT)
static void cliResource(const char *cmdName, char *cmdline)
{
    char *pch = NULL;
    char *saveptr;

    pch = strtok_r(cmdline, " ", &saveptr);
    if (!pch) {
        printResource(DUMP_MASTER | HIDE_UNUSED, NULL);

        return;
    } else if (strcasecmp(pch, "show") == 0) {
#ifdef MINIMAL_CLI
        cliPrintLine("IO");
#else
        cliPrintLine("Currently active IO resource assignments:\r\n(reboot to update)");
        cliRepeat('-', 20);
#endif
        for (int i = 0; i < DEFIO_IO_USED_COUNT; i++) {
            const char* owner;
            owner = ownerNames[ioRecs[i].owner];

            cliPrintf("%c%02d: %s", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner);
            if (ioRecs[i].index > 0) {
                cliPrintf(" %d", ioRecs[i].index);
            }
            cliPrintLinefeed();
        }

        pch = strtok_r(NULL, " ", &saveptr);
        if (strcasecmp(pch, "all") == 0) {
#if defined(USE_TIMER_MGMT)
            cliTimer(cmdName, "show");
#endif
#if defined(USE_DMA)
            cliDma(cmdName, "show");
#endif
        }

        return;
    }

    unsigned resourceIndex = 0;
    for (; ; resourceIndex++) {
        if (resourceIndex >= ARRAYLEN(resourceTable)) {
            cliPrintErrorLinef(cmdName, "INVALID RESOURCE NAME: '%s'", pch);
            return;
        }

        const char *resourceName = ownerNames[resourceTable[resourceIndex].owner];
        if (strncasecmp(pch, resourceName, strlen(resourceName)) == 0) {
            break;
        }
    }

    pch = strtok_r(NULL, " ", &saveptr);
    int index = atoi(pch);

    if (resourceTable[resourceIndex].maxIndex > 0 || index > 0) {
        if (index <= 0 || index > MAX_RESOURCE_INDEX(resourceTable[resourceIndex].maxIndex)) {
            cliShowArgumentRangeError(cmdName, "INDEX", 1, MAX_RESOURCE_INDEX(resourceTable[resourceIndex].maxIndex));
            return;
        }
        index -= 1;

        pch = strtok_r(NULL, " ", &saveptr);
    }

    ioTag_t *tag = getIoTag(resourceTable[resourceIndex], index);

    if (strlen(pch) > 0) {
        if (strToPin(pch, tag)) {
            if (*tag == IO_TAG_NONE) {
#ifdef MINIMAL_CLI
                cliPrintLine("Freed");
#else
                cliPrintLine("Resource is freed");
#endif
                return;
            } else {
                ioRec_t *rec = IO_Rec(IOGetByTag(*tag));
                if (rec) {
                    resourceCheck(resourceIndex, index, *tag);
#ifdef MINIMAL_CLI
                    cliPrintLinef(" %c%02d set", IO_GPIOPortIdx(rec) + 'A', IO_GPIOPinIdx(rec));
#else
                    cliPrintLinef("\r\nResource is set to %c%02d", IO_GPIOPortIdx(rec) + 'A', IO_GPIOPinIdx(rec));
#endif
                } else {
                    cliShowParseError(cmdName);
                }
                return;
            }
        }
    }

    cliShowParseError(cmdName);
}
#endif

#ifdef USE_DSHOT_TELEMETRY
static void cliDshotTelemetryInfo(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);

    if (useDshotTelemetry) {
        cliPrintLinef("Dshot reads: %u", dshotTelemetryState.readCount);
        cliPrintLinef("Dshot invalid pkts: %u", dshotTelemetryState.invalidPacketCount);
        uint32_t directionChangeCycles = dshotDMAHandlerCycleCounters.changeDirectionCompletedAt - dshotDMAHandlerCycleCounters.irqAt;
        uint32_t directionChangeDurationUs = clockCyclesToMicros(directionChangeCycles);
        cliPrintLinef("Dshot directionChange cycles: %u, micros: %u", directionChangeCycles, directionChangeDurationUs);
        cliPrintLinefeed();

#ifdef USE_DSHOT_TELEMETRY_STATS
        cliPrintLine("Motor      eRPM      RPM      Hz   Invalid");
        cliPrintLine("=====   =======   ======   =====   =======");
#else
        cliPrintLine("Motor      eRPM      RPM      Hz");
        cliPrintLine("=====   =======   ======   =====");
#endif
        for (uint8_t i = 0; i < getMotorCount(); i++) {
            cliPrintf("%5d   %7d   %6d   %5d   ", i,
                      (int)getDshotTelemetry(i) * 100,
                      (int)getDshotTelemetry(i) * 100 * 2 / motorConfig()->motorPoleCount,
                      (int)getDshotTelemetry(i) * 100 * 2 / motorConfig()->motorPoleCount / 60);
#ifdef USE_DSHOT_TELEMETRY_STATS
            if (isDshotMotorTelemetryActive(i)) {
                const int calcPercent = getDshotTelemetryMotorInvalidPercent(i);
                cliPrintLinef("%3d.%02d%%", calcPercent / 100, calcPercent % 100);
            } else {
                cliPrintLine("NO DATA");
            }
#else
            cliPrintLinefeed();
#endif
        }
        cliPrintLinefeed();

        const int len = MAX_GCR_EDGES;
#ifdef DEBUG_BBDECODE
        extern uint16_t bbBuffer[134];
        for (int i = 0; i < 134; i++) {
            cliPrintf("%u ", (int)bbBuffer[i]);
        }
        cliPrintLinefeed();
#endif
        for (int i = 0; i < len; i++) {
            cliPrintf("%u ", (int)dshotTelemetryState.inputBuffer[i]);
        }
        cliPrintLinefeed();
        for (int i = 1; i < len; i++) {
            cliPrintf("%u ", (int)(dshotTelemetryState.inputBuffer[i]  - dshotTelemetryState.inputBuffer[i-1]));
        }
        cliPrintLinefeed();
    } else {
        cliPrintLine("Dshot telemetry not enabled");
    }
}
#endif

static void printConfig(const char *cmdName, char *cmdline, bool doDiff)
{
    dumpFlags_t dumpMask = DUMP_MASTER;
    char *options;
    if ((options = checkCommand(cmdline, "master"))) {
        dumpMask = DUMP_MASTER; // only
    } else if ((options = checkCommand(cmdline, "profile"))) {
        dumpMask = DUMP_PROFILE; // only
    } else if ((options = checkCommand(cmdline, "rates"))) {
        dumpMask = DUMP_RATES; // only
    } else if ((options = checkCommand(cmdline, "hardware"))) {
        dumpMask = DUMP_MASTER | HARDWARE_ONLY;   // Show only hardware related settings (useful to generate unified target configs).
    } else if ((options = checkCommand(cmdline, "all"))) {
        dumpMask = DUMP_ALL;   // all profiles and rates
    } else {
        options = cmdline;
    }

    if (doDiff) {
        dumpMask = dumpMask | DO_DIFF;
    }

    if (checkCommand(options, "defaults")) {
        dumpMask = dumpMask | SHOW_DEFAULTS;   // add default values as comments for changed values
    } else if (checkCommand(options, "bare")) {
        dumpMask = dumpMask | BARE;   // show the diff / dump without extra commands and board specific data
    }

    backupAndResetConfigs((dumpMask & BARE) == 0);

#ifdef USE_CLI_BATCH
    bool batchModeEnabled = false;
#endif
    if ((dumpMask & DUMP_MASTER) || (dumpMask & DUMP_ALL)) {
        cliPrintHashLine("version");
        printVersion(cmdName, false);

        if (!(dumpMask & BARE)) {
#ifdef USE_CLI_BATCH
            cliPrintHashLine("start the command batch");
            cliPrintLine("batch start");
            batchModeEnabled = true;
#endif

            if ((dumpMask & (DUMP_ALL | DO_DIFF)) == (DUMP_ALL | DO_DIFF)) {
                cliPrintHashLine("reset configuration to default settings");
                cliPrintLine("defaults nosave");
            }
        }

#if defined(USE_BOARD_INFO)
        cliPrintLinefeed();
        printBoardName(dumpMask);
        printManufacturerId(dumpMask);
#endif

        if ((dumpMask & DUMP_ALL) && !(dumpMask & BARE)) {
            cliMcuId(cmdName, "");
#if defined(USE_SIGNATURE)
            cliSignature(cmdName, "");
#endif
        }

        if (!(dumpMask & HARDWARE_ONLY)) {
            printName(dumpMask, &pilotConfig_Copy);
        }

#ifdef USE_RESOURCE_MGMT
        printResource(dumpMask, "resources");
#if defined(USE_TIMER_MGMT)
        printTimer(dumpMask, "timer");
#endif
#ifdef USE_DMA_SPEC
        printDmaopt(dumpMask, "dma");
#endif
#endif

        printFeature(dumpMask, featureConfig_Copy.enabledFeatures, featureConfig()->enabledFeatures, "feature");

        printSerial(dumpMask, &serialConfig_Copy, serialConfig(), "serial");

        if (!(dumpMask & HARDWARE_ONLY)) {
#ifndef USE_QUAD_MIXER_ONLY
            const char *mixerHeadingStr = "mixer";
            const bool equalsDefault = mixerConfig_Copy.mixerMode == mixerConfig()->mixerMode;
            mixerHeadingStr = cliPrintSectionHeading(dumpMask, !equalsDefault, mixerHeadingStr);
            const char *formatMixer = "mixer %s";
            cliDefaultPrintLinef(dumpMask, equalsDefault, formatMixer, mixerNames[mixerConfig()->mixerMode - 1]);
            cliDumpPrintLinef(dumpMask, equalsDefault, formatMixer, mixerNames[mixerConfig_Copy.mixerMode - 1]);

            cliDumpPrintLinef(dumpMask, customMotorMixer(0)->throttle == 0.0f, "\r\nmmix reset\r\n");

            printMotorMix(dumpMask, customMotorMixer_CopyArray, customMotorMixer(0), mixerHeadingStr);

#ifdef USE_SERVOS
            printServo(dumpMask, servoParams_CopyArray, servoParams(0), "servo");

            const char *servoMixHeadingStr = "servo mixer";
            if (!(dumpMask & DO_DIFF) || customServoMixers(0)->rate != 0) {
                cliPrintHashLine(servoMixHeadingStr);
                cliPrintLine("smix reset\r\n");
                servoMixHeadingStr = NULL;
            }
            printServoMix(dumpMask, customServoMixers_CopyArray, customServoMixers(0), servoMixHeadingStr);
#endif
#endif

#if defined(USE_BEEPER)
            printBeeper(dumpMask, beeperConfig_Copy.beeper_off_flags, beeperConfig()->beeper_off_flags, "beeper", BEEPER_ALLOWED_MODES, "beeper");

#if defined(USE_DSHOT)
            printBeeper(dumpMask, beeperConfig_Copy.dshotBeaconOffFlags, beeperConfig()->dshotBeaconOffFlags, "beacon", DSHOT_BEACON_ALLOWED_MODES, "beacon");
#endif
#endif // USE_BEEPER

            printMap(dumpMask, &rxConfig_Copy, rxConfig(), "map");

#ifdef USE_LED_STRIP_STATUS_MODE
            printLed(dumpMask, ledStripStatusModeConfig_Copy.ledConfigs, ledStripStatusModeConfig()->ledConfigs, "led");

            printColor(dumpMask, ledStripStatusModeConfig_Copy.colors, ledStripStatusModeConfig()->colors, "color");

            printModeColor(dumpMask, &ledStripStatusModeConfig_Copy, ledStripStatusModeConfig(), "mode_color");
#endif

            printAux(dumpMask, modeActivationConditions_CopyArray, modeActivationConditions(0), "aux");

            printAdjustmentRange(dumpMask, adjustmentRanges_CopyArray, adjustmentRanges(0), "adjrange");

            printRxRange(dumpMask, rxChannelRangeConfigs_CopyArray, rxChannelRangeConfigs(0), "rxrange");

#ifdef USE_VTX_TABLE
            printVtxTable(dumpMask, &vtxTableConfig_Copy, vtxTableConfig(), "vtxtable");
#endif

#ifdef USE_VTX_CONTROL
            printVtx(dumpMask, &vtxConfig_Copy, vtxConfig(), "vtx");
#endif

            printRxFailsafe(dumpMask, rxFailsafeChannelConfigs_CopyArray, rxFailsafeChannelConfigs(0), "rxfail");
        }

        if (dumpMask & HARDWARE_ONLY) {
            dumpAllValues(cmdName, HARDWARE_VALUE, dumpMask, "master");
        } else {
            dumpAllValues(cmdName, MASTER_VALUE, dumpMask, "master");

            if (dumpMask & DUMP_ALL) {
                for (uint32_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
                    cliDumpPidProfile(cmdName, pidProfileIndex, dumpMask);
                }

                pidProfileIndexToUse = systemConfig_Copy.pidProfileIndex;

                if (!(dumpMask & BARE)) {
                    cliPrintHashLine("restore original profile selection");

                    cliProfile(cmdName, "");
                }

                pidProfileIndexToUse = CURRENT_PROFILE_INDEX;

                for (uint32_t rateIndex = 0; rateIndex < CONTROL_RATE_PROFILE_COUNT; rateIndex++) {
                    cliDumpRateProfile(cmdName, rateIndex, dumpMask);
                }

                rateProfileIndexToUse = systemConfig_Copy.activeRateProfile;

                if (!(dumpMask & BARE)) {
                    cliPrintHashLine("restore original rateprofile selection");

                    cliRateProfile(cmdName, "");

                    cliPrintHashLine("save configuration");
                    cliPrint("save");
#ifdef USE_CLI_BATCH
                    batchModeEnabled = false;
#endif
                }

                rateProfileIndexToUse = CURRENT_PROFILE_INDEX;
            } else {
                cliDumpPidProfile(cmdName, systemConfig_Copy.pidProfileIndex, dumpMask);

                cliDumpRateProfile(cmdName, systemConfig_Copy.activeRateProfile, dumpMask);
            }
        }
    } else if (dumpMask & DUMP_PROFILE) {
        cliDumpPidProfile(cmdName, systemConfig_Copy.pidProfileIndex, dumpMask);
    } else if (dumpMask & DUMP_RATES) {
        cliDumpRateProfile(cmdName, systemConfig_Copy.activeRateProfile, dumpMask);
    }

#ifdef USE_CLI_BATCH
    if (batchModeEnabled) {
        cliPrintHashLine("end the command batch");
        cliPrintLine("batch end");
    }
#endif

    // restore configs from copies
    restoreConfigs();
}

static void cliDump(const char *cmdName, char *cmdline)
{
    printConfig(cmdName, cmdline, false);
}

static void cliDiff(const char *cmdName, char *cmdline)
{
    printConfig(cmdName, cmdline, true);
}

#if defined(USE_USB_MSC)
static void cliMsc(const char *cmdName, char *cmdline)
{
    if (mscCheckFilesystemReady()) {
#ifdef USE_RTC_TIME
        int timezoneOffsetMinutes = timeConfig()->tz_offsetMinutes;
        if (!isEmpty(cmdline)) {
            timezoneOffsetMinutes = atoi(cmdline);
            if ((timezoneOffsetMinutes < TIMEZONE_OFFSET_MINUTES_MIN) || (timezoneOffsetMinutes > TIMEZONE_OFFSET_MINUTES_MAX)) {
                cliPrintErrorLinef(cmdName, "INVALID TIMEZONE OFFSET");
                return;
            }
        }
#else
        int timezoneOffsetMinutes = 0;
        UNUSED(cmdline);
#endif
        cliPrintHashLine("Restarting in mass storage mode");
        cliPrint("\r\nRebooting");
        cliWriterFlush();
        waitForSerialPortToFinishTransmitting(cliPort);
        motorShutdown();

        systemResetToMsc(timezoneOffsetMinutes);
    } else {
        cliPrintHashLine("Storage not present or failed to initialize!");
    }
}
#endif

typedef void cliCommandFn(const char* name, char *cmdline);

typedef struct {
    const char *name;
#ifndef MINIMAL_CLI
    const char *description;
    const char *args;
#endif
    cliCommandFn *cliCommand;
} clicmd_t;

#ifndef MINIMAL_CLI
#define CLI_COMMAND_DEF(name, description, args, cliCommand) \
{ \
    name , \
    description , \
    args , \
    cliCommand \
}
#else
#define CLI_COMMAND_DEF(name, description, args, cliCommand) \
{ \
    name, \
    cliCommand \
}
#endif

static void cliHelp(const char *cmdName, char *cmdline);

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    CLI_COMMAND_DEF("adjrange", "configure adjustment ranges", "<index> <unused> <range channel> <start> <end> <function> <select channel> [<center> <scale>]", cliAdjustmentRange),
#ifdef USE_SIMPLIFIED_TUNING
    CLI_COMMAND_DEF("apply_simplified_tuning", "applies tuning based on simplified tuning settings", NULL, cliApplySimplifiedTuning),
#endif
    CLI_COMMAND_DEF("aux", "configure modes", "<index> <mode> <aux> <start> <end> <logic>", cliAux),
#ifdef USE_CLI_BATCH
    CLI_COMMAND_DEF("batch", "start or end a batch of commands", "start | end", cliBatch),
#endif
#if defined(USE_BEEPER)
#if defined(USE_DSHOT)
    CLI_COMMAND_DEF("beacon", "enable/disable Dshot beacon for a condition", "list\r\n"
        "\t<->[name]", cliBeacon),
#endif
    CLI_COMMAND_DEF("beeper", "enable/disable beeper for a condition", "list\r\n"
        "\t<->[name]", cliBeeper),
#endif // USE_BEEPER
#if defined(USE_RX_BIND)
    CLI_COMMAND_DEF("bind_rx", "initiate binding for RX SPI or SRXL2", NULL, cliRxBind),
#endif
#if defined(USE_FLASH_BOOT_LOADER)
    CLI_COMMAND_DEF("bl", "reboot into bootloader", "[flash|rom]", cliBootloader),
#else
    CLI_COMMAND_DEF("bl", "reboot into bootloader", "[rom]", cliBootloader),
#endif
#if defined(USE_BOARD_INFO)
    CLI_COMMAND_DEF("board_name", "get / set the name of the board model", "[board name]", cliBoardName),
#endif
#ifdef USE_LED_STRIP_STATUS_MODE
        CLI_COMMAND_DEF("color", "configure colors", NULL, cliColor),
#endif
#if defined(USE_CUSTOM_DEFAULTS)
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", "[nosave|bare|show]", cliDefaults),
#else
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", "[nosave|show]", cliDefaults),
#endif
    CLI_COMMAND_DEF("diff", "list configuration changes from default", "[master|profile|rates|hardware|all] {defaults|bare}", cliDiff),
#ifdef USE_RESOURCE_MGMT

#ifdef USE_DMA
#ifdef USE_DMA_SPEC
    CLI_COMMAND_DEF("dma", "show/set DMA assignments", "<> | <device> <index> list | <device> <index> [<option>|none] | list | show", cliDma),
#else
    CLI_COMMAND_DEF("dma", "show DMA assignments", "show", cliDma),
#endif
#endif

#endif
#ifdef USE_DSHOT_TELEMETRY
    CLI_COMMAND_DEF("dshot_telemetry_info", "display dshot telemetry info and stats", NULL, cliDshotTelemetryInfo),
#endif
#ifdef USE_DSHOT
    CLI_COMMAND_DEF("dshotprog", "program DShot ESC(s)", "<index> <command>+", cliDshotProg),
#endif
    CLI_COMMAND_DEF("dump", "dump configuration",
        "[master|profile|rates|hardware|all] {defaults|bare}", cliDump),
#ifdef USE_ESCSERIAL
    CLI_COMMAND_DEF("escprog", "passthrough esc to serial", "<mode [sk/bl/ki/cc]> <index>", cliEscPassthrough),
#endif
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("feature", "configure features",
        "list\r\n"
        "\t<->[name]", cliFeature),
#ifdef USE_FLASHFS
    CLI_COMMAND_DEF("flash_erase", "erase flash chip", NULL, cliFlashErase),
    CLI_COMMAND_DEF("flash_info", "show flash chip info", NULL, cliFlashInfo),
#ifdef USE_FLASH_TOOLS
    CLI_COMMAND_DEF("flash_read", NULL, "<length> <address>", cliFlashRead),
    CLI_COMMAND_DEF("flash_scan", "scan flash device for errors", NULL, cliFlashVerify),
    CLI_COMMAND_DEF("flash_write", NULL, "<address> <message>", cliFlashWrite),
#endif
#endif
    CLI_COMMAND_DEF("get", "get variable value", "[name]", cliGet),
#ifdef USE_GPS
    CLI_COMMAND_DEF("gpspassthrough", "passthrough gps to serial", NULL, cliGpsPassthrough),
#endif
#if defined(USE_GYRO_REGISTER_DUMP) && !defined(SIMULATOR_BUILD)
    CLI_COMMAND_DEF("gyroregisters", "dump gyro config registers contents", NULL, cliDumpGyroRegisters),
#endif
    CLI_COMMAND_DEF("help", "display command help", "[search string]", cliHelp),
#ifdef USE_LED_STRIP_STATUS_MODE
        CLI_COMMAND_DEF("led", "configure leds", NULL, cliLed),
#endif
#if defined(USE_BOARD_INFO)
    CLI_COMMAND_DEF("manufacturer_id", "get / set the id of the board manufacturer", "[manufacturer id]", cliManufacturerId),
#endif
    CLI_COMMAND_DEF("map", "configure rc channel order", "[<map>]", cliMap),
    CLI_COMMAND_DEF("mcu_id", "id of the microcontroller", NULL, cliMcuId),
#ifndef USE_QUAD_MIXER_ONLY
    CLI_COMMAND_DEF("mixer", "configure mixer", "list\r\n\t<name>", cliMixer),
#endif
    CLI_COMMAND_DEF("mmix", "custom motor mixer", NULL, cliMotorMix),
#ifdef USE_LED_STRIP_STATUS_MODE
    CLI_COMMAND_DEF("mode_color", "configure mode and special colors", NULL, cliModeColor),
#endif
    CLI_COMMAND_DEF("motor",  "get/set motor", "<index> [<value>]", cliMotor),
#ifdef USE_USB_MSC
#ifdef USE_RTC_TIME
    CLI_COMMAND_DEF("msc", "switch into msc mode", "[<timezone offset minutes>]", cliMsc),
#else
    CLI_COMMAND_DEF("msc", "switch into msc mode", NULL, cliMsc),
#endif
#endif
#ifndef MINIMAL_CLI
    CLI_COMMAND_DEF("play_sound", NULL, "[<index>]", cliPlaySound),
#endif
    CLI_COMMAND_DEF("profile", "change profile", "[<index>]", cliProfile),
    CLI_COMMAND_DEF("rateprofile", "change rate profile", "[<index>]", cliRateProfile),
#ifdef USE_RC_SMOOTHING_FILTER
    CLI_COMMAND_DEF("rc_smoothing_info", "show rc_smoothing operational settings", NULL, cliRcSmoothing),
#endif // USE_RC_SMOOTHING_FILTER
#ifdef USE_RESOURCE_MGMT
    CLI_COMMAND_DEF("resource", "show/set resources", "<> | <resource name> <index> [<pin>|none] | show [all]", cliResource),
#endif
    CLI_COMMAND_DEF("rxfail", "show/set rx failsafe settings", NULL, cliRxFailsafe),
    CLI_COMMAND_DEF("rxrange", "configure rx channel ranges", NULL, cliRxRange),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
#ifdef USE_SDCARD
    CLI_COMMAND_DEF("sd_info", "sdcard info", NULL, cliSdInfo),
#endif
    CLI_COMMAND_DEF("serial", "configure serial ports", NULL, cliSerial),
#if defined(USE_SERIAL_PASSTHROUGH)
#if defined(USE_PINIO)
    CLI_COMMAND_DEF("serialpassthrough", "passthrough serial data data from port 1 to VCP / port 2", "<id1> [<baud1>] [<mode1>] [none|<dtr pinio>|reset] [<id2>] [<baud2>] [<mode2>]", cliSerialPassthrough),
#else
    CLI_COMMAND_DEF("serialpassthrough", "passthrough serial data from port 1 to VCP / port 2", "<id1> [<baud1>] [<mode1>] [none|reset] [<id2>] [<baud2>] [<mode2>]", cliSerialPassthrough),
#endif
#endif
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("servo", "configure servos", NULL, cliServo),
#endif
    CLI_COMMAND_DEF("set", "change setting", "[<name>=<value>]", cliSet),
#if defined(USE_SIGNATURE)
    CLI_COMMAND_DEF("signature", "get / set the board type signature", "[signature]", cliSignature),
#endif
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("smix", "servo mixer", "<rule> <servo> <source> <rate> <speed> <min> <max> <box>\r\n"
        "\treset\r\n"
        "\tload <mixer>\r\n"
        "\treverse <servo> <source> r|n", cliServoMix),
#endif
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
#if defined(USE_TASK_STATISTICS)
    CLI_COMMAND_DEF("tasks", "show task stats", NULL, cliTasks),
#endif
#ifdef USE_TIMER_MGMT
    CLI_COMMAND_DEF("timer", "show/set timers", "<> | <pin> list | <pin> [af<alternate function>|none|<option(deprecated)>] | list | show", cliTimer),
#endif
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
#ifdef USE_VTX_CONTROL
#ifdef MINIMAL_CLI
    CLI_COMMAND_DEF("vtx", "vtx channels on switch", NULL, cliVtx),
#else
    CLI_COMMAND_DEF("vtx", "vtx channels on switch", "<index> <aux_channel> <vtx_band> <vtx_channel> <vtx_power> <start_range> <end_range>", cliVtx),
#endif
#endif
#ifdef USE_VTX_TABLE
    CLI_COMMAND_DEF("vtx_info", "vtx power config dump", NULL, cliVtxInfo),
    CLI_COMMAND_DEF("vtxtable", "vtx frequency table", "<band> <bandname> <bandletter> [FACTORY|CUSTOM] <freq> ... <freq>\r\n", cliVtxTable),
#endif
};

static void cliHelp(const char *cmdName, char *cmdline)
{
    bool anyMatches = false;

    for (uint32_t i = 0; i < ARRAYLEN(cmdTable); i++) {
        bool printEntry = false;
        if (isEmpty(cmdline)) {
            printEntry = true;
        } else {
            if (strcasestr(cmdTable[i].name, cmdline)
#ifndef MINIMAL_CLI
                || strcasestr(cmdTable[i].description, cmdline)
#endif
               ) {
                printEntry = true;
            }
        }

        if (printEntry) {
            anyMatches = true;
            cliPrint(cmdTable[i].name);
#ifndef MINIMAL_CLI
            if (cmdTable[i].description) {
                cliPrintf(" - %s", cmdTable[i].description);
            }
            if (cmdTable[i].args) {
                cliPrintf("\r\n\t%s", cmdTable[i].args);
            }
#endif
            cliPrintLinefeed();
        }
    }
    if (!isEmpty(cmdline) && !anyMatches) {
        cliPrintErrorLinef(cmdName, "NO MATCHES FOR '%s'", cmdline);
    }
}

static void processCharacter(const char c)
{
    if (bufferIndex && (c == '\n' || c == '\r')) {
        // enter pressed
        cliPrintLinefeed();

#if defined(USE_CUSTOM_DEFAULTS) && defined(DEBUG_CUSTOM_DEFAULTS)
        if (processingCustomDefaults) {
            cliPrint("d: ");
        }
#endif

        // Strip comment starting with # from line
        char *p = cliBuffer;
        p = strchr(p, '#');
        if (NULL != p) {
            bufferIndex = (uint32_t)(p - cliBuffer);
        }

        // Strip trailing whitespace
        while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
            bufferIndex--;
        }

        // Process non-empty lines
        if (bufferIndex > 0) {
            cliBuffer[bufferIndex] = 0; // null terminate

            const clicmd_t *cmd;
            char *options;
            for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                if ((options = checkCommand(cliBuffer, cmd->name))) {
                    break;
                }
            }
            if (cmd < cmdTable + ARRAYLEN(cmdTable)) {
                cmd->cliCommand(cmd->name, options);
            } else {
                cliPrintError("input", "UNKNOWN COMMAND, TRY 'HELP'");
            }
            bufferIndex = 0;
        }

        memset(cliBuffer, 0, sizeof(cliBuffer));

        // 'exit' will reset this flag, so we don't need to print prompt again
        if (!cliMode) {
            return;
        }

        cliPrompt();
    } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
        if (!bufferIndex && c == ' ')
            return; // Ignore leading spaces
        cliBuffer[bufferIndex++] = c;
        cliWrite(c);
    }
}

static void processCharacterInteractive(const char c)
{
    if (c == '\t' || c == '?') {
        // do tab completion
        const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
        uint32_t i = bufferIndex;
        for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
            if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0)) {
                continue;
            }
            if (!pstart) {
                pstart = cmd;
            }
            pend = cmd;
        }
        if (pstart) {    /* Buffer matches one or more commands */
            for (; ; bufferIndex++) {
                if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                    break;
                if (!pstart->name[bufferIndex] && bufferIndex < sizeof(cliBuffer) - 2) {
                    /* Unambiguous -- append a space */
                    cliBuffer[bufferIndex++] = ' ';
                    cliBuffer[bufferIndex] = '\0';
                    break;
                }
                cliBuffer[bufferIndex] = pstart->name[bufferIndex];
            }
        }
        if (!bufferIndex || pstart != pend) {
            /* Print list of ambiguous matches */
            cliPrint("\r\n\033[K");
            for (cmd = pstart; cmd <= pend; cmd++) {
                cliPrint(cmd->name);
                cliWrite('\t');
            }
            cliPrompt();
            i = 0;    /* Redraw prompt */
        }
        for (; i < bufferIndex; i++)
            cliWrite(cliBuffer[i]);
    } else if (!bufferIndex && c == 4) {   // CTRL-D
        cliExit("", cliBuffer);
        return;
    } else if (c == 12) {                  // NewPage / CTRL-L
        // clear screen
        cliPrint("\033[2J\033[1;1H");
        cliPrompt();
    } else if (c == 127) {
        // backspace
        if (bufferIndex) {
            cliBuffer[--bufferIndex] = 0;
            cliPrint("\010 \010");
        }
    } else {
        processCharacter(c);
    }
}

void cliProcess(void)
{
    if (!cliWriter) {
        return;
    }

    // Flush the buffer to get rid of any MSP data polls sent by configurator after CLI was invoked
    cliWriterFlush();

    while (serialRxBytesWaiting(cliPort)) {
        uint8_t c = serialRead(cliPort);

        processCharacterInteractive(c);
    }
}

#if defined(USE_CUSTOM_DEFAULTS)
static bool cliProcessCustomDefaults(bool quiet)
{
    if (processingCustomDefaults || !hasCustomDefaults()) {
        return false;
    }

    bufWriter_t *cliWriterTemp = NULL;
    if (quiet
#if !defined(DEBUG_CUSTOM_DEFAULTS)
        || true
#endif
       ) {
        cliWriterTemp = cliWriter;
        cliWriter = NULL;
    }
    if (quiet) {
        cliErrorWriter = NULL;
    }

    memcpy(cliBufferTemp, cliBuffer, sizeof(cliBuffer));
    uint32_t bufferIndexTemp = bufferIndex;
    bufferIndex = 0;
    processingCustomDefaults = true;

    char *customDefaultsPtr = customDefaultsStart;
    while (customDefaultsHasNext(customDefaultsPtr)) {
        processCharacter(*customDefaultsPtr++);
    }

    // Process a newline at the very end so that the last command gets executed,
    // even when the file did not contain a trailing newline
    processCharacter('\r');

    processingCustomDefaults = false;

    if (cliWriterTemp) {
        cliWriter = cliWriterTemp;
        cliErrorWriter = cliWriter;
    }

    memcpy(cliBuffer, cliBufferTemp, sizeof(cliBuffer));
    bufferIndex = bufferIndexTemp;

    systemConfigMutable()->configurationState = CONFIGURATION_STATE_DEFAULTS_CUSTOM;

    return true;
}
#endif

void cliEnter(serialPort_t *serialPort)
{
    cliMode = true;
    cliPort = serialPort;
    setPrintfSerialPort(cliPort);
    cliWriter = bufWriterInit(cliWriteBuffer, sizeof(cliWriteBuffer), (bufWrite_t)serialWriteBufShim, serialPort);
    cliErrorWriter = cliWriter;

    schedulerSetCalulateTaskStatistics(systemConfig()->task_statistics);

#ifndef MINIMAL_CLI
    cliPrintLine("\r\nEntering CLI Mode, type 'exit' to return, or 'help'");
#else
    cliPrintLine("\r\nCLI");
#endif
    setArmingDisabled(ARMING_DISABLED_CLI);

    cliPrompt();

#ifdef USE_CLI_BATCH
    resetCommandBatch();
#endif
}

#endif // USE_CLI
