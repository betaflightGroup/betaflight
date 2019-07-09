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

#include "platform.h"

#include "build/debug.h"

#include "blackbox/blackbox.h"

#include "cms/cms.h"

#include "common/utils.h"
#include "common/time.h"

#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/camera_control.h"
#include "drivers/light_led.h"
#include "drivers/pinio.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "osd/osd.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/bus_i2c.h"
#include "pg/dashboard.h"
#include "pg/flash.h"
#include "pg/gyrodev.h"
#include "pg/max7456.h"
#include "pg/mco.h"
#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "pg/rx.h"
#include "pg/rx_pwm.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"
#include "pg/sdcard.h"
#include "pg/vcd.h"
#include "pg/vtx_io.h"
#include "pg/usb.h"
#include "pg/sdio.h"
#include "pg/rcdevice.h"
#include "pg/stats.h"
#include "pg/board.h"

#include "rx/rx.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_sfhss.h"
#include "rx/spektrum.h"
#include "rx/cyrf6936_spektrum.h"
#include "rx/a7105_flysky.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#include "telemetry/frsky_hub.h"
#include "telemetry/ibus_shared.h"
#include "telemetry/telemetry.h"

#include "settings.h"


// Sensor names (used in lookup tables for *_hardware settings and in status command output)
// sync with accelerationSensor_e
const char * const lookupTableAccHardware[] = {
    "AUTO", "NONE", "ADXL345", "MPU6050", "MMA8452", "BMA280", "LSM303DLHC",
    "MPU6000", "MPU6500", "MPU9250", "ICM20601", "ICM20602", "ICM20608G", "ICM20649", "ICM20689",
    "BMI160", "FAKE"
};

// sync with gyroSensor_e
const char * const lookupTableGyroHardware[] = {
    "AUTO", "NONE", "MPU6050", "L3G4200D", "MPU3050", "L3GD20",
    "MPU6000", "MPU6500", "MPU9250", "ICM20601", "ICM20602", "ICM20608G", "ICM20649", "ICM20689",
    "BMI160", "FAKE"
};

#if defined(USE_SENSOR_NAMES) || defined(USE_BARO)
// sync with baroSensor_e
const char * const lookupTableBaroHardware[] = {
    "AUTO", "NONE", "BMP085", "MS5611", "BMP280", "LPS", "QMP6988", "BMP388"
};
#endif
#if defined(USE_SENSOR_NAMES) || defined(USE_MAG)
// sync with magSensor_e
const char * const lookupTableMagHardware[] = {
    "AUTO", "NONE", "HMC5883", "AK8975", "AK8963", "QMC5883", "LIS3MDL"
};
#endif
#if defined(USE_SENSOR_NAMES) || defined(USE_RANGEFINDER)
const char * const lookupTableRangefinderHardware[] = {
    "NONE", "HCSR04", "TFMINI", "TF02"
};
#endif

static const char * const lookupTableOffOn[] = {
    "OFF", "ON"
};

static const char * const lookupTableCrashRecovery[] = {
    "OFF", "ON" ,"BEEP", "DISARM"
};

static const char * const lookupTableUnit[] = {
    "IMPERIAL", "METRIC"
};

static const char * const lookupTableAlignment[] = {
    "DEFAULT",
    "CW0",
    "CW90",
    "CW180",
    "CW270",
    "CW0FLIP",
    "CW90FLIP",
    "CW180FLIP",
    "CW270FLIP"
};

#ifdef USE_MULTI_GYRO
static const char * const lookupTableGyro[] = {
    "FIRST", "SECOND", "BOTH"
};
#endif

#ifdef USE_GPS
static const char * const lookupTableGPSProvider[] = {
    "NMEA", "UBLOX", "MSP"
};

static const char * const lookupTableGPSSBASMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN"
};
#endif

#ifdef USE_SERVOS
static const char * const lookupTableGimbalMode[] = {
    "NORMAL", "MIXTILT"
};
#endif

#ifdef USE_BLACKBOX
static const char * const lookupTableBlackboxDevice[] = {
    "NONE", "SPIFLASH", "SDCARD", "SERIAL"
};

static const char * const lookupTableBlackboxMode[] = {
    "NORMAL", "MOTOR_TEST", "ALWAYS"
};
#endif

#ifdef USE_SERIAL_RX
static const char * const lookupTableSerialRX[] = {
    "SPEK1024",
    "SPEK2048",
    "SBUS",
    "SUMD",
    "SUMH",
    "XB-B",
    "XB-B-RJ01",
    "IBUS",
    "JETIEXBUS",
    "CRSF",
    "SRXL",
    "CUSTOM",
    "FPORT",
    "DJI_HDL",
};
#endif

#ifdef USE_RX_SPI
// sync with rx_spi_protocol_e
static const char * const lookupTableRxSpi[] = {
    "V202_250K",
    "V202_1M",
    "SYMA_X",
    "SYMA_X5C",
    "CX10",
    "CX10A",
    "H8_3D",
    "INAV",
    "FRSKY_D",
    "FRSKY_X",
    "FLYSKY",
    "FLYSKY_2A",
    "KN",
    "SFHSS",
    "SPEKTRUM",
    "FRSKY_X_LBT"
};
#endif

static const char * const lookupTableGyroHardwareLpf[] = {
    "NORMAL",
    "1KHZ_SAMPLING",
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    "EXPERIMENTAL"
#endif
};

#ifdef USE_CAMERA_CONTROL
static const char * const lookupTableCameraControlMode[] = {
    "HARDWARE_PWM",
    "SOFTWARE_PWM",
    "DAC"
};
#endif

static const char * const lookupTablePwmProtocol[] = {
    "OFF", "ONESHOT125", "ONESHOT42", "MULTISHOT", "BRUSHED",
#ifdef USE_DSHOT
    "DSHOT150", "DSHOT300", "DSHOT600", "DSHOT1200", "PROSHOT1000"
#endif
};

static const char * const lookupTableRcInterpolation[] = {
    "OFF", "PRESET", "AUTO", "MANUAL"
};

static const char * const lookupTableRcInterpolationChannels[] = {
    "RP", "RPY", "RPYT", "T", "RPT",
};

static const char * const lookupTableLowpassType[] = {
    "PT1",
    "BIQUAD",
};

static const char * const lookupTableDtermLowpassType[] = {
    "PT1",
    "BIQUAD",
};

static const char * const lookupTableAntiGravityMode[] = {
    "SMOOTH",
    "STEP",
};

static const char * const lookupTableFailsafe[] = {
    "AUTO-LAND", "DROP", "GPS-RESCUE"
};

static const char * const lookupTableFailsafeSwitchMode[] = {
    "STAGE1", "KILL", "STAGE2"
};

static const char * const lookupTableBusType[] = {
    "NONE", "I2C", "SPI", "SLAVE",
#if defined(USE_SPI_GYRO) && defined(USE_I2C_GYRO)
    "GYROAUTO"
#endif
};

#ifdef USE_MAX7456
static const char * const lookupTableMax7456Clock[] = {
    "HALF", "DEFAULT", "FULL"
};
#endif

#ifdef USE_RX_FRSKY_SPI
static const char * const lookupTableFrskySpiA1Source[] = {
    "VBAT", "EXTADC", "CONST"
};
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
static const char * const lookupTableGyroOverflowCheck[] = {
    "OFF", "YAW", "ALL"
};
#endif

static const char * const lookupTableRatesType[] = {
    "BETAFLIGHT", "RACEFLIGHT", "KISS"
};

#ifdef USE_OVERCLOCK
static const char * const lookupOverclock[] = {
    "OFF",
#if defined(STM32F40_41xxx)
    "192MHZ", "216MHZ", "240MHZ"
#elif defined(STM32F411xE)
    "108MHZ", "120MHZ"
#elif defined(STM32F7)
    "240MHZ"
#endif
};
#endif

#ifdef USE_LED_STRIP
    static const char * const lookupLedStripFormatRGB[] = {
        "GRB", "RGB"
    };
#endif

static const char * const lookupTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};


#ifdef USE_GPS_RESCUE
static const char * const lookupTableRescueSanityType[] = {
    "RESCUE_SANITY_OFF", "RESCUE_SANITY_ON", "RESCUE_SANITY_FS_ONLY"
};
const char * const lookupTableRescueAltitudeMode[] = {
    "MAX_ALT", "FIXED_ALT", "CURRENT_ALT"
};
#endif

#ifdef USE_MAX7456
static const char * const lookupTableVideoSystem[] = {
    "AUTO", "PAL", "NTSC"
};
#endif // USE_MAX7456

#ifdef USE_MAX7456_EXTENDED
static const char * const lookupTableVideoSystemExtended[] = {
    "AUTO", "ON", "OFF"
};
#endif

#if defined(USE_ITERM_RELAX)
static const char * const lookupTableItermRelax[] = {
    "OFF", "RP", "RPY", "RP_INC", "RPY_INC"
};
static const char * const lookupTableItermRelaxType[] = {
    "GYRO", "SETPOINT"
};
#endif

#ifdef USE_ACRO_TRAINER
static const char * const lookupTableAcroTrainerDebug[] = {
    "ROLL", "PITCH"
};
#endif // USE_ACRO_TRAINER

#ifdef USE_RC_SMOOTHING_FILTER
static const char * const lookupTableRcSmoothingType[] = {
    "INTERPOLATION", "FILTER"
};
static const char * const lookupTableRcSmoothingDebug[] = {
    "ROLL", "PITCH", "YAW", "THROTTLE"
};
static const char * const lookupTableRcSmoothingInputType[] = {
    "PT1", "BIQUAD"
};
static const char * const lookupTableRcSmoothingDerivativeType[] = {
    "OFF", "PT1", "BIQUAD"
};
#endif // USE_RC_SMOOTHING_FILTER

#ifdef USE_GYRO_DATA_ANALYSE
static const char * const lookupTableDynamicFilterRange[] = {
    "HIGH", "MEDIUM", "LOW", "AUTO"
};
#endif // USE_GYRO_DATA_ANALYSE

#ifdef USE_VTX_COMMON
static const char * const lookupTableVtxLowPowerDisarm[] = {
    "OFF", "ON", "UNTIL_FIRST_ARM"
};
#endif

#ifdef USE_SDCARD
static const char * const lookupTableSdcardMode[] = {
    "OFF", "SPI", "SDIO"
};
#endif

#ifdef USE_LAUNCH_CONTROL
static const char * const lookupTableLaunchControlMode[] = {
    "NORMAL", "PITCHONLY", "FULL"
};
#endif

#ifdef USE_TPA_MODE
static const char * const lookupTableTpaMode[] = {
    "PD", "D"
};
#endif

#ifdef USE_LED_STRIP
#ifdef USE_LED_STRIP_STATUS_MODE
static const char * const lookupTableLEDProfile[] = {
    "RACE", "BEACON", "STATUS"
};
#else
static const char * const lookupTableLEDProfile[] = {
    "RACE", "BEACON"
};
#endif
#endif

const char * const lookupTableLedstripColors[COLOR_COUNT] = {
    "BLACK",
    "WHITE",
    "RED",
    "ORANGE",
    "YELLOW",
    "LIME_GREEN",
    "GREEN",
    "MINT_GREEN",
    "CYAN",
    "LIGHT_BLUE",
    "BLUE",
    "DARK_VIOLET",
    "MAGENTA",
    "DEEP_PINK"
};

static const char * const lookupTableGyroFilterDebug[] = {
    "ROLL", "PITCH", "YAW"
};

static const char * const lookupTablePositionAltSource[] = {
    "DEFAULT", "BARO_ONLY", "GPS_ONLY"
};

#define LOOKUP_TABLE_ENTRY(name) { name, ARRAYLEN(name) }

const lookupTableEntry_t lookupTables[] = {
    LOOKUP_TABLE_ENTRY(lookupTableOffOn),
    LOOKUP_TABLE_ENTRY(lookupTableUnit),
    LOOKUP_TABLE_ENTRY(lookupTableAlignment),
#ifdef USE_GPS
    LOOKUP_TABLE_ENTRY(lookupTableGPSProvider),
    LOOKUP_TABLE_ENTRY(lookupTableGPSSBASMode),
#ifdef USE_GPS_RESCUE
    LOOKUP_TABLE_ENTRY(lookupTableRescueSanityType),
    LOOKUP_TABLE_ENTRY(lookupTableRescueAltitudeMode),
#endif
#endif
#ifdef USE_BLACKBOX
    LOOKUP_TABLE_ENTRY(lookupTableBlackboxDevice),
    LOOKUP_TABLE_ENTRY(lookupTableBlackboxMode),
#endif
    LOOKUP_TABLE_ENTRY(currentMeterSourceNames),
    LOOKUP_TABLE_ENTRY(voltageMeterSourceNames),
#ifdef USE_SERVOS
    LOOKUP_TABLE_ENTRY(lookupTableGimbalMode),
#endif
#ifdef USE_SERIAL_RX
    LOOKUP_TABLE_ENTRY(lookupTableSerialRX),
#endif
#ifdef USE_RX_SPI
    LOOKUP_TABLE_ENTRY(lookupTableRxSpi),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableGyroHardwareLpf),
    LOOKUP_TABLE_ENTRY(lookupTableAccHardware),
#ifdef USE_BARO
    LOOKUP_TABLE_ENTRY(lookupTableBaroHardware),
#endif
#ifdef USE_MAG
    LOOKUP_TABLE_ENTRY(lookupTableMagHardware),
#endif
    LOOKUP_TABLE_ENTRY(debugModeNames),
    LOOKUP_TABLE_ENTRY(lookupTablePwmProtocol),
    LOOKUP_TABLE_ENTRY(lookupTableRcInterpolation),
    LOOKUP_TABLE_ENTRY(lookupTableRcInterpolationChannels),
    LOOKUP_TABLE_ENTRY(lookupTableLowpassType),
    LOOKUP_TABLE_ENTRY(lookupTableDtermLowpassType),
    LOOKUP_TABLE_ENTRY(lookupTableAntiGravityMode),
    LOOKUP_TABLE_ENTRY(lookupTableFailsafe),
    LOOKUP_TABLE_ENTRY(lookupTableFailsafeSwitchMode),
    LOOKUP_TABLE_ENTRY(lookupTableCrashRecovery),
#ifdef USE_CAMERA_CONTROL
    LOOKUP_TABLE_ENTRY(lookupTableCameraControlMode),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableBusType),
#ifdef USE_MAX7456
    LOOKUP_TABLE_ENTRY(lookupTableMax7456Clock),
#endif
#ifdef USE_RX_FRSKY_SPI
    LOOKUP_TABLE_ENTRY(lookupTableFrskySpiA1Source),
#endif
#ifdef USE_RANGEFINDER
    LOOKUP_TABLE_ENTRY(lookupTableRangefinderHardware),
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    LOOKUP_TABLE_ENTRY(lookupTableGyroOverflowCheck),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableRatesType),
#ifdef USE_OVERCLOCK
    LOOKUP_TABLE_ENTRY(lookupOverclock),
#endif
#ifdef USE_LED_STRIP
    LOOKUP_TABLE_ENTRY(lookupLedStripFormatRGB),
#endif
#ifdef USE_MULTI_GYRO
    LOOKUP_TABLE_ENTRY(lookupTableGyro),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableThrottleLimitType),
#ifdef USE_MAX7456
    LOOKUP_TABLE_ENTRY(lookupTableVideoSystem),
#endif // USE_MAX7456
#ifdef USE_MAX7456_EXTENDED
    LOOKUP_TABLE_ENTRY(lookupTableVideoSystemExtended),
#endif
#if defined(USE_ITERM_RELAX)
    LOOKUP_TABLE_ENTRY(lookupTableItermRelax),
    LOOKUP_TABLE_ENTRY(lookupTableItermRelaxType),
#endif
#ifdef USE_ACRO_TRAINER
    LOOKUP_TABLE_ENTRY(lookupTableAcroTrainerDebug),
#endif // USE_ACRO_TRAINER
#ifdef USE_RC_SMOOTHING_FILTER
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingType),
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingDebug),
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingInputType),
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingDerivativeType),
#endif // USE_RC_SMOOTHING_FILTER
#ifdef USE_GYRO_DATA_ANALYSE
    LOOKUP_TABLE_ENTRY(lookupTableDynamicFilterRange),
#endif // USE_GYRO_DATA_ANALYSE
#ifdef USE_VTX_COMMON
    LOOKUP_TABLE_ENTRY(lookupTableVtxLowPowerDisarm),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableGyroHardware),
#ifdef USE_SDCARD
    LOOKUP_TABLE_ENTRY(lookupTableSdcardMode),
#endif
#ifdef USE_LAUNCH_CONTROL
    LOOKUP_TABLE_ENTRY(lookupTableLaunchControlMode),
#endif
#ifdef USE_TPA_MODE
    LOOKUP_TABLE_ENTRY(lookupTableTpaMode),
#endif
#ifdef USE_LED_STRIP
    LOOKUP_TABLE_ENTRY(lookupTableLEDProfile),
    LOOKUP_TABLE_ENTRY(lookupTableLedstripColors),
#endif

    LOOKUP_TABLE_ENTRY(lookupTableGyroFilterDebug),

    LOOKUP_TABLE_ENTRY(lookupTablePositionAltSource)
};

#undef LOOKUP_TABLE_ENTRY

const clivalue_t valueTable[] = {
// PG_GYRO_CONFIG
    { "gyro_hardware_lpf",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_HARDWARE_LPF }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_hardware_lpf) },
#if defined(USE_GYRO_SPI_ICM20649)
    { "gyro_high_range",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_high_fsr) },
#endif
    { "gyro_sync_denom",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 32 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_sync_denom) },

    { "gyro_lowpass_type",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LOWPASS_TYPE }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lowpass_type) },
    { "gyro_lowpass_hz",            VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lowpass_hz) },

    { "gyro_lowpass2_type",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LOWPASS_TYPE }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lowpass2_type) },
    { "gyro_lowpass2_hz",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0,  FILTER_FREQUENCY_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lowpass2_hz) },

    { "gyro_notch1_hz",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_1) },
    { "gyro_notch1_cutoff",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_1) },
    { "gyro_notch2_hz",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_2) },
    { "gyro_notch2_cutoff",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_2) },

    { "gyro_calib_duration",        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50,  3000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroCalibrationDuration) },
    { "gyro_calib_noise_limit",     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0,  200 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroMovementCalibrationThreshold) },
    { "gyro_offset_yaw",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -1000, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_offset_yaw) },
#ifdef USE_GYRO_OVERFLOW_CHECK
    { "gyro_overflow_detect",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_OVERFLOW_CHECK }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, checkOverflow) },
#endif
#ifdef USE_YAW_SPIN_RECOVERY
    { "yaw_spin_recovery",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, yaw_spin_recovery) },
    { "yaw_spin_threshold",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 500,  1950 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, yaw_spin_threshold) },
#endif

#ifdef USE_MULTI_GYRO
    { "gyro_to_use",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_to_use) },
#endif
#if defined(USE_GYRO_DATA_ANALYSE)
    { "dyn_notch_range",           VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DYNAMIC_FILTER_RANGE }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_notch_range) },
    { "dyn_notch_width_percent",   VAR_UINT8   | MASTER_VALUE, .config.minmaxUnsigned = { 0, 20 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_notch_width_percent) },
    { "dyn_notch_q",               VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_notch_q) },
    { "dyn_notch_min_hz",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 60, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_notch_min_hz) },
#endif
#ifdef USE_DYN_LPF
    { "dyn_lpf_gyro_min_hz",        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_lpf_gyro_min_hz) },
    { "dyn_lpf_gyro_max_hz",        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_lpf_gyro_max_hz) },
#endif
    { "gyro_filter_debug_axis",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_FILTER_DEBUG }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_filter_debug_axis) },

// PG_ACCELEROMETER_CONFIG
#if defined(USE_ACC)
    { "acc_hardware",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ACC_HARDWARE }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_hardware) },
#if defined(USE_GYRO_SPI_ICM20649)
    { "acc_high_range",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_high_fsr) },
#endif
    { "acc_lpf_hz",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 400 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_lpf_hz) },
    { "acc_trim_pitch",             VAR_INT16  | MASTER_VALUE, .config.minmax = { -300, 300 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accelerometerTrims.values.pitch) },
    { "acc_trim_roll",              VAR_INT16  | MASTER_VALUE, .config.minmax = { -300, 300 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accelerometerTrims.values.roll) },
    { "acc_calibration",            VAR_INT16  | MASTER_VALUE | MODE_ARRAY, .config.array.length = XYZ_AXIS_COUNT, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accZero.raw) },
#endif

// PG_COMPASS_CONFIG
#ifdef USE_MAG
    { "align_mag",                  VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_align) },
    { "mag_bustype",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_bustype) },
    { "mag_i2c_device",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_i2c_device) },
    { "mag_i2c_address",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_i2c_address) },
    { "mag_spi_device",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_spi_device) },
    { "mag_hardware",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAG_HARDWARE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_hardware) },
    { "mag_declination",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -18000, 18000 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_declination) },
    { "mag_calibration",            VAR_INT16  | MASTER_VALUE | MODE_ARRAY, .config.array.length = XYZ_AXIS_COUNT, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw) },
#endif

// PG_BAROMETER_CONFIG
#ifdef USE_BARO
    { "baro_bustype",               VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_bustype) },
    { "baro_spi_device",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 5 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_spi_device) },
    { "baro_i2c_device",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 5 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_i2c_device) },
    { "baro_i2c_address",           VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_i2c_address) },
    { "baro_hardware",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BARO_HARDWARE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_hardware) },
    { "baro_tab_size",              VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, BARO_SAMPLE_COUNT_MAX }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_sample_count) },
    { "baro_noise_lpf",             VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_noise_lpf) },
    { "baro_cf_vel",                VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_cf_vel) },
#endif

// PG_RX_CONFIG
    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1200, 1700 }, PG_RX_CONFIG, offsetof(rxConfig_t, midrc) },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, mincheck) },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, maxcheck) },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE, .config.minmax = { 0, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_channel) },
    { "rssi_src_frame_errors",      VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_src_frame_errors) },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { RSSI_SCALE_MIN, RSSI_SCALE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_scale) },
    { "rssi_offset",                VAR_INT8   | MASTER_VALUE, .config.minmax = { -100, 100 }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_offset) },
    { "rssi_invert",                VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_invert) },
    { "rssi_src_frame_lpf_period",  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_src_frame_lpf_period) },
    { "rc_interp",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_INTERPOLATION }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolation) },
    { "rc_interp_ch",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_INTERPOLATION_CHANNELS }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolationChannels) },
    { "rc_interp_int",              VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 50 }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolationInterval) },

#ifdef USE_RC_SMOOTHING_FILTER
    { "rc_smoothing_type",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_TYPE }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_type) },
    { "rc_smoothing_input_hz",      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_input_cutoff) },
    { "rc_smoothing_derivative_hz", VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_derivative_cutoff) },
    { "rc_smoothing_debug_axis",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_DEBUG }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_debug_axis) },
    { "rc_smoothing_input_type",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_INPUT_TYPE }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_input_type) },
    { "rc_smoothing_derivative_type",VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_DERIVATIVE_TYPE }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_derivative_type) },
    { "rc_smoothing_auto_smoothness",VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 50 }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_auto_factor) },
#endif // USE_RC_SMOOTHING_FILTER

    { "fpv_mix_degrees",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_RX_CONFIG, offsetof(rxConfig_t, fpvCamAngleDegrees) },
    { "max_aux_channels",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, MAX_AUX_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, max_aux_channel) },
#ifdef USE_SERIAL_RX
    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SERIAL_RX }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_provider) },
    { "serialrx_inverted",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_inverted) },
#endif
#ifdef USE_SPEKTRUM_BIND
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { SPEKTRUM_SAT_BIND_DISABLED, SPEKTRUM_SAT_BIND_MAX}, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind) },
    { "spektrum_sat_bind_autoreset",VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind_autoreset) },
#endif
    { "airmode_start_throttle_percent",     VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_RX_CONFIG, offsetof(rxConfig_t, airModeActivateThreshold) },
    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_min_usec) },
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_max_usec) },
    { "serialrx_halfduplex",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, halfDuplex) },
#ifdef USE_RX_SPI
    { "rx_spi_protocol",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RX_SPI }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, rx_spi_protocol) },
    { "rx_spi_bus",                 VAR_UINT8   | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, spibus) },
    { "rx_spi_led_inversion",       VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, ledInversion) },
#endif

// PG_ADC_CONFIG
#if defined(USE_ADC)
    { "adc_device",                 VAR_INT8 | HARDWARE_VALUE, .config.minmax = { 0, ADCDEV_COUNT }, PG_ADC_CONFIG, offsetof(adcConfig_t, device) },
    { "adc_vrefint_calibration",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, vrefIntCalibration) },
    { "adc_tempsensor_calibration30", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, tempSensorCalibration1) },
    { "adc_tempsensor_calibration110", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, tempSensorCalibration2) },
#endif

// PG_PWM_CONFIG
#if defined(USE_PWM)
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  .config.lookup = { TABLE_OFF_ON }, PG_PWM_CONFIG, offsetof(pwmConfig_t, inputFilteringMode) },
#endif

// PG_BLACKBOX_CONFIG
#ifdef USE_BLACKBOX
    { "blackbox_p_ratio",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, INT16_MAX }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, p_ratio) },
    { "blackbox_device",            VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_DEVICE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, device) },
    { "blackbox_record_acc",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, record_acc) },
    { "blackbox_mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_MODE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, mode) },
#endif

// PG_MOTOR_CONFIG
    { "min_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, minthrottle) },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, maxthrottle) },
    { "min_command",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, mincommand) },
#ifdef USE_DSHOT
    { "dshot_idle_value",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, digitalIdleOffsetValue) },
#ifdef USE_DSHOT_DMAR
    { "dshot_burst",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useBurstDshot) },
#endif
#ifdef USE_DSHOT_TELEMETRY
    { "dshot_bidir",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useDshotTelemetry) },
#endif
#endif
    { "use_unsynced_pwm",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useUnsyncedPwm) },
    { "motor_pwm_protocol",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MOTOR_PWM_PROTOCOL }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmProtocol) },
    { "motor_pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 200, 32000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmRate) },
    { "motor_pwm_inversion",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmInversion) },
    { "motor_poles",                VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 4, UINT8_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, motorPoleCount) },

// PG_THROTTLE_CORRECTION_CONFIG
    { "thr_corr_value",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0,  150 }, PG_THROTTLE_CORRECTION_CONFIG, offsetof(throttleCorrectionConfig_t, throttle_correction_value) },
    { "thr_corr_angle",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1,  900 }, PG_THROTTLE_CORRECTION_CONFIG, offsetof(throttleCorrectionConfig_t, throttle_correction_angle) },

// PG_FAILSAFE_CONFIG
    { "failsafe_delay",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_delay) },
    { "failsafe_off_delay",         VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_off_delay) },
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle) },
    { "failsafe_switch_mode",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FAILSAFE_SWITCH_MODE }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_switch_mode) },
    { "failsafe_throttle_low_delay",VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 300 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle_low_delay) },
    { "failsafe_procedure",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FAILSAFE }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_procedure) },
    { "failsafe_recovery_delay",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_recovery_delay) },
    { "failsafe_stick_threshold",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 50 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_stick_threshold) },

// PG_BOARDALIGNMENT_CONFIG
    { "align_board_roll",           VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, rollDegrees) },
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, pitchDegrees) },
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, yawDegrees) },

// PG_GIMBAL_CONFIG
#ifdef USE_SERVOS
    { "gimbal_mode",                VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GIMBAL_MODE }, PG_GIMBAL_CONFIG, offsetof(gimbalConfig_t, mode) },
#endif

// PG_BATTERY_CONFIG
    { "bat_capacity",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 20000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, batteryCapacity) },
    { "vbat_max_cell_voltage",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmaxcellvoltage) },
    { "vbat_full_cell_voltage",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatfullcellvoltage) },
    { "vbat_min_cell_voltage",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmincellvoltage) },
    { "vbat_warning_cell_voltage",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatwarningcellvoltage) },
    { "vbat_hysteresis",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbathysteresis) },
    { "current_meter",              VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CURRENT_METER }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, currentMeterSource) },
    { "battery_meter",              VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VOLTAGE_METER }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, voltageMeterSource) },
    { "vbat_detect_cell_voltage",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatnotpresentcellvoltage) },
    { "use_vbat_alerts",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, useVBatAlerts) },
    { "use_cbat_alerts",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, useConsumptionAlerts) },
    { "cbat_alert_percent",         VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, consumptionWarningPercentage) },
    { "vbat_cutoff_percent",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, lvcPercentage) },
    { "force_battery_cell_count",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 24 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, forceBatteryCellCount) },
    { "vbat_lpf_period",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatLpfPeriod) },
    { "ibat_lpf_period",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, ibatLpfPeriod) },
    { "vbat_duration_for_warning",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 150 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatDurationForWarning) },
    { "vbat_duration_for_critical",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 150 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatDurationForCritical) },

//  PG_VOLTAGE_SENSOR_ADC_CONFIG
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_SCALE_MIN, VBAT_SCALE_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatscale) },
    { "vbat_divider",               VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_DIVIDER_MIN, VBAT_DIVIDER_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatresdivval) },
    { "vbat_multiplier",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_MULTIPLIER_MIN, VBAT_MULTIPLIER_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatresdivmultiplier) },

// PG_CURRENT_SENSOR_ADC_CONFIG
    { "ibata_scale",                VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_ADC_CONFIG, offsetof(currentSensorADCConfig_t, scale) },
    { "ibata_offset",               VAR_INT16  | MASTER_VALUE, .config.minmax = { -32000, 32000 }, PG_CURRENT_SENSOR_ADC_CONFIG, offsetof(currentSensorADCConfig_t, offset) },
// PG_CURRENT_SENSOR_ADC_CONFIG
#ifdef USE_VIRTUAL_CURRENT_METER
    { "ibatv_scale",                VAR_INT16  | MASTER_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, offsetof(currentSensorVirtualConfig_t, scale) },
    { "ibatv_offset",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, offsetof(currentSensorVirtualConfig_t, offset) },
#endif

#ifdef USE_BEEPER
// PG_BEEPER_DEV_CONFIG
    { "beeper_inversion",           VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, isInverted) },
    { "beeper_od",                  VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, isOpenDrain) },
    { "beeper_frequency",           VAR_INT16  | HARDWARE_VALUE, .config.minmax = { 0, 16000 }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, frequency) },

// PG_BEEPER_CONFIG
#ifdef USE_DSHOT
    { "beeper_dshot_beacon_tone",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = {1, DSHOT_CMD_BEACON5 }, PG_BEEPER_CONFIG, offsetof(beeperConfig_t, dshotBeaconTone) },
#endif
#endif // USE_BEEPER

// PG_MIXER_CONFIG
    { "yaw_motors_reversed",        VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, yaw_motors_reversed) },
    { "crashflip_motor_percent",    VAR_UINT8 |  MASTER_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, crashflip_motor_percent) },

// PG_MOTOR_3D_CONFIG
    { "3d_deadband_low",            VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_RANGE_MIDDLE }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_low) },
    { "3d_deadband_high",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_RANGE_MIDDLE, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_high) },
    { "3d_neutral",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, neutral3d) },
    { "3d_deadband_throttle",       VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1, 100 }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_throttle) },
    { "3d_limit_low",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_RANGE_MIDDLE }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, limit3d_low) },
    { "3d_limit_high",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_RANGE_MIDDLE, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, limit3d_high) },
    { "3d_switched_mode",           VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, switched_mode3d) },

// PG_SERVO_CONFIG
#ifdef USE_SERVOS
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_SERVO_CONFIG, offsetof(servoConfig_t, dev.servoCenterPulse) },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 498 }, PG_SERVO_CONFIG, offsetof(servoConfig_t, dev.servoPwmRate) },
    { "servo_lowpass_hz",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 400}, PG_SERVO_CONFIG, offsetof(servoConfig_t, servo_lowpass_freq) },
    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SERVO_CONFIG, offsetof(servoConfig_t, tri_unarmed_servo) },
    { "channel_forwarding_start",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { AUX1, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_SERVO_CONFIG, offsetof(servoConfig_t, channelForwardingStartChannel) },
#endif

// PG_CONTROLRATE_PROFILES
#ifdef USE_PROFILE_NAMES
    { "rateprofile_name",           VAR_UINT8  | PROFILE_RATE_VALUE | MODE_STRING, .config.string = { 1, MAX_RATE_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, profileName) },
#endif
    { "thr_mid",                    VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrMid8) },
    { "thr_expo",                   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrExpo8) },
    { "rates_type",                 VAR_UINT8  | PROFILE_RATE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RATES_TYPE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates_type) },
    { "roll_rc_rate",               VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_ROLL]) },
    { "pitch_rc_rate",              VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_PITCH]) },
    { "yaw_rc_rate",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_YAW]) },
    { "roll_expo",                  VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_ROLL]) },
    { "pitch_expo",                 VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_PITCH]) },
    { "yaw_expo",                   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_YAW]) },
    { "roll_srate",                 VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_ROLL]) },
    { "pitch_srate",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_PITCH]) },
    { "yaw_srate",                  VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_YAW]) },
    { "tpa_rate",                   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_TPA_MAX}, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, dynThrPID) },
    { "tpa_breakpoint",             VAR_UINT16 | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, tpa_breakpoint) },
#ifdef USE_TPA_MODE
    { "tpa_mode",                   VAR_UINT8  | PROFILE_RATE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_TPA_MODE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, tpaMode) },
#endif
    { "throttle_limit_type",        VAR_UINT8  | PROFILE_RATE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_THROTTLE_LIMIT_TYPE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, throttle_limit_type) },
    { "throttle_limit_percent",     VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 25, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, throttle_limit_percent) },
    { "roll_rate_limit",            VAR_UINT16 | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { CONTROL_RATE_CONFIG_RATE_LIMIT_MIN, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rate_limit[FD_ROLL]) },
    { "pitch_rate_limit",           VAR_UINT16 | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { CONTROL_RATE_CONFIG_RATE_LIMIT_MIN, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rate_limit[FD_PITCH]) },
    { "yaw_rate_limit",             VAR_UINT16 | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { CONTROL_RATE_CONFIG_RATE_LIMIT_MIN, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rate_limit[FD_YAW]) },

// PG_SERIAL_CONFIG
    { "reboot_character",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 48, 126 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, reboot_character) },
    { "serial_update_rate_hz",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 2000 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, serial_update_rate_hz) },

// PG_IMU_CONFIG
    { "imu_dcm_kp",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_kp) },
    { "imu_dcm_ki",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_ki) },
    { "small_angle",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 180 }, PG_IMU_CONFIG, offsetof(imuConfig_t, small_angle) },

// PG_ARMING_CONFIG
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 60 }, PG_ARMING_CONFIG, offsetof(armingConfig_t, auto_disarm_delay) },
    { "gyro_cal_on_first_arm",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ARMING_CONFIG, offsetof(armingConfig_t, gyro_cal_on_first_arm) },


// PG_GPS_CONFIG
#ifdef USE_GPS
    { "gps_provider",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_PROVIDER }, PG_GPS_CONFIG, offsetof(gpsConfig_t, provider) },
    { "gps_sbas_mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_SBAS_MODE }, PG_GPS_CONFIG, offsetof(gpsConfig_t, sbasMode) },
    { "gps_auto_config",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, autoConfig) },
    { "gps_auto_baud",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, autoBaud) },
    { "gps_ublox_use_galileo",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_ublox_use_galileo) },
    { "gps_set_home_point_once",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_set_home_point_once) },

#ifdef USE_GPS_RESCUE
    // PG_GPS_RESCUE
    { "gps_rescue_angle",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, angle) },
    { "gps_rescue_initial_alt",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 20, 100 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, initialAltitudeM) },
    { "gps_rescue_descent_dist",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 30, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, descentDistanceM) },
    { "gps_rescue_landing_alt",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 3, 10 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, targetLandingAltitudeM) },
    { "gps_rescue_landing_dist",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 5, 15 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, targetLandingDistanceM) },
    { "gps_rescue_ground_speed",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 30, 3000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, rescueGroundspeed) },
    { "gps_rescue_throttle_p",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleP) },
    { "gps_rescue_throttle_i",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleI) },
    { "gps_rescue_throttle_d",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleD) },
    { "gps_rescue_velocity_p",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velP) },
    { "gps_rescue_velocity_i",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velI) },
    { "gps_rescue_velocity_d",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velD) },
    { "gps_rescue_yaw_p",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, yawP) },

    { "gps_rescue_throttle_min",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleMin) },
    { "gps_rescue_throttle_max",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleMax) },
    { "gps_rescue_ascend_rate",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 2500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, ascendRate) },
    { "gps_rescue_descend_rate",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, descendRate) },
    { "gps_rescue_throttle_hover",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleHover) },
    { "gps_rescue_sanity_checks",   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_RESCUE_SANITY_CHECK }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, sanityChecks) },
    { "gps_rescue_min_sats",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 50 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, minSats) },
    { "gps_rescue_min_dth",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 1000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, minRescueDth) },
    { "gps_rescue_allow_arming_without_fix", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, allowArmingWithoutFix) },
    { "gps_rescue_alt_mode",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_RESCUE_ALT_MODE }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, altitudeMode) },
#ifdef USE_MAG
    { "gps_rescue_use_mag",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, useMag) },
#endif
#endif
#endif

    { "deadband",                   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, deadband) },
    { "yaw_deadband",               VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_deadband) },
    { "yaw_control_reversed",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_control_reversed) },

// PG_PID_CONFIG
    { "pid_process_denom",          VAR_UINT8  | MASTER_VALUE,  .config.minmaxUnsigned = { 1, MAX_PID_PROCESS_DENOM }, PG_PID_CONFIG, offsetof(pidConfig_t, pid_process_denom) },
#ifdef USE_RUNAWAY_TAKEOFF
    { "runaway_takeoff_prevention", VAR_UINT8  | MODE_LOOKUP,  .config.lookup = { TABLE_OFF_ON }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_prevention) },    // enables/disables runaway takeoff prevention
    { "runaway_takeoff_deactivate_delay",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 100, 1000 }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_deactivate_delay) },           // deactivate time in ms
    { "runaway_takeoff_deactivate_throttle_percent",  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_deactivate_throttle) }, // minimum throttle percentage during deactivation phase
#endif

// PG_PID_PROFILE
#ifdef USE_PROFILE_NAMES
    { "profile_name",               VAR_UINT8  | PROFILE_VALUE | MODE_STRING, .config.string = { 1, MAX_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PID_PROFILE, offsetof(pidProfile_t, profileName) },
#endif
#ifdef USE_DYN_LPF
    { "dyn_lpf_dterm_min_hz",       VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_lpf_dterm_min_hz) },
    { "dyn_lpf_dterm_max_hz",       VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_lpf_dterm_max_hz) },
#endif
    { "dterm_lowpass_type",         VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DTERM_LOWPASS_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_filter_type) },
    { "dterm_lowpass_hz",           VAR_INT16  | PROFILE_VALUE, .config.minmax = { 0, FILTER_FREQUENCY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lowpass_hz) },
    { "dterm_lowpass2_type",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DTERM_LOWPASS_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_filter2_type) },
    { "dterm_lowpass2_hz",          VAR_INT16  | PROFILE_VALUE, .config.minmax = { 0, FILTER_FREQUENCY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lowpass2_hz) },
    { "dterm_notch_hz",             VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_notch_hz) },
    { "dterm_notch_cutoff",         VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_notch_cutoff) },
    { "vbat_pid_gain",              VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, vbatPidCompensation) },
    { "pid_at_min_throttle",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, pidAtMinThrottle) },
    { "anti_gravity_mode",          VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ANTI_GRAVITY_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, antiGravityMode) },
    { "anti_gravity_threshold",     VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 20, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermThrottleThreshold) },
    { "anti_gravity_gain",          VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 1000, 30000 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermAcceleratorGain) },
    { "feedforward_transition",     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, feedForwardTransition) },
    { "acc_limit_yaw",              VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, yawRateAccelLimit) },
    { "acc_limit",                  VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, rateAccelLimit) },
    { "crash_dthreshold",           VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_dthreshold) },
    { "crash_gthreshold",           VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_gthreshold) },
    { "crash_setpoint_threshold",   VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_setpoint_threshold) },
    { "crash_time",                 VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 5000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_time) },
    { "crash_delay",                VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_delay) },
    { "crash_recovery_angle",       VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 30 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery_angle) },
    { "crash_recovery_rate",        VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery_rate) },
    { "crash_limit_yaw",            VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_limit_yaw) },
    { "crash_recovery",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CRASH_RECOVERY }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery) },

    { "iterm_rotation",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_rotation) },
#if defined(USE_ITERM_RELAX)
    { "iterm_relax",                VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ITERM_RELAX }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax) },
    { "iterm_relax_type",           VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ITERM_RELAX_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax_type) },
    { "iterm_relax_cutoff",         VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax_cutoff) },
#endif
    { "iterm_windup",               VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 30, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermWindupPointPercent) },
    { "iterm_limit",                VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermLimit) },
    { "pidsum_limit",               VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PIDSUM_LIMIT_MIN, PIDSUM_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimit) },
    { "pidsum_limit_yaw",           VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PIDSUM_LIMIT_MIN, PIDSUM_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimitYaw) },
    { "yaw_lowpass_hz",             VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, yaw_lowpass_hz) },

#if defined(USE_THROTTLE_BOOST)
    { "throttle_boost",             VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, throttle_boost) },
    { "throttle_boost_cutoff",      VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 5, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, throttle_boost_cutoff) },
#endif

#ifdef USE_ACRO_TRAINER
    { "acro_trainer_angle_limit",   VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 80 }, PG_PID_PROFILE, offsetof(pidProfile_t, acro_trainer_angle_limit) },
    { "acro_trainer_lookahead_ms",  VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, acro_trainer_lookahead_ms) },
    { "acro_trainer_debug_axis",    VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ACRO_TRAINER_DEBUG }, PG_PID_PROFILE, offsetof(pidProfile_t, acro_trainer_debug_axis) },
    { "acro_trainer_gain",          VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 25, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, acro_trainer_gain) },
#endif // USE_ACRO_TRAINER

    { "p_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].P) },
    { "i_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].I) },
    { "d_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].D) },
    { "f_pitch",                    VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 },PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].F) },
    { "p_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].P) },
    { "i_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].I) },
    { "d_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].D) },
    { "f_roll",                     VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 },PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].F) },
    { "p_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].P) },
    { "i_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].I) },
    { "d_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].D) },
    { "f_yaw",                      VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 },PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].F) },

    { "angle_level_strength",       VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].P) },
    { "horizon_level_strength",     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].I) },
    { "horizon_transition",         VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].D) },

    { "level_limit",                VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 90 }, PG_PID_PROFILE, offsetof(pidProfile_t, levelAngleLimit) },

    { "horizon_tilt_effect",        VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0,  250 }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_tilt_effect) },
    { "horizon_tilt_expert_mode",   VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_tilt_expert_mode) },

#if defined(USE_ABSOLUTE_CONTROL)
    { "abs_control_gain",           VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 20 }, PG_PID_PROFILE, offsetof(pidProfile_t, abs_control_gain) },
    { "abs_control_limit",          VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 10, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, abs_control_limit) },
    { "abs_control_error_limit",    VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 1, 45 }, PG_PID_PROFILE, offsetof(pidProfile_t, abs_control_error_limit) },
    { "abs_control_cutoff",         VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 1, 45 }, PG_PID_PROFILE, offsetof(pidProfile_t, abs_control_cutoff) },
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    { "use_integrated_yaw",         VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = {TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, use_integrated_yaw) },
    { "integrated_yaw_relax",       VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, integrated_yaw_relax) },
#endif

#ifdef USE_D_MIN
    { "d_min_roll",                 VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min[FD_ROLL]) },
    { "d_min_pitch",                VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min[FD_PITCH]) },
    { "d_min_yaw",                  VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min[FD_YAW]) },
    { "d_min_boost_gain",           VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min_gain) },
    { "d_min_advance",              VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min_advance) },
#endif

    { "motor_output_limit",         VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { MOTOR_OUTPUT_LIMIT_PERCENT_MIN, MOTOR_OUTPUT_LIMIT_PERCENT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, motor_output_limit) },
    { "auto_profile_cell_count",    VAR_INT8 | PROFILE_VALUE,  .config.minmax = { AUTO_PROFILE_CELL_COUNT_CHANGE, MAX_AUTO_DETECT_CELL_COUNT }, PG_PID_PROFILE, offsetof(pidProfile_t, auto_profile_cell_count) },

#ifdef USE_LAUNCH_CONTROL
    { "launch_control_mode",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LAUNCH_CONTROL_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlMode) },
    { "launch_trigger_allow_reset", VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlAllowTriggerReset) },
    { "launch_trigger_throttle_percent", VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlThrottlePercent) },
    { "launch_angle_limit",         VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 80 }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlAngleLimit) },
    { "launch_control_gain",        VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlGain) },
#endif

#ifdef USE_THRUST_LINEARIZATION
    { "thrust_linear",              VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, thrustLinearization) },
#endif
#ifdef USE_AIRMODE_LPF
    { "transient_throttle_limit",   VAR_UINT8 | MASTER_VALUE, .config.minmax = { 0, 30 }, PG_PID_PROFILE, offsetof(pidProfile_t, transient_throttle_limit) },
#endif

// PG_TELEMETRY_CONFIG
#ifdef USE_TELEMETRY
    { "tlm_inverted",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, telemetry_inverted) },
    { "tlm_halfduplex",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, halfDuplex) },
#if defined(USE_TELEMETRY_FRSKY_HUB)
#if defined(USE_GPS)
    { "frsky_default_lat",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -9000, 9000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLatitude) },
    { "frsky_default_long",         VAR_INT16  | MASTER_VALUE, .config.minmax = { -18000, 18000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLongitude) },
    { "frsky_gps_format",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, FRSKY_FORMAT_NMEA }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_coordinate_format) },
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_unit) },
#endif
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { FRSKY_VFAS_PRECISION_LOW,  FRSKY_VFAS_PRECISION_HIGH }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_vfas_precision) },
#endif // USE_TELEMETRY_FRSKY_HUB
    { "hott_alarm_int",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 120 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, hottAlarmSoundInterval) },
    { "pid_in_tlm",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = {TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, pidValuesAsTelemetry) },
    { "report_cell_voltage",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, report_cell_voltage) },
#if defined(USE_TELEMETRY_IBUS)
    { "ibus_sensor",                VAR_UINT8  | MASTER_VALUE | MODE_ARRAY, .config.array.length = IBUS_SENSOR_COUNT, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, flysky_sensors)},
#endif
#ifdef USE_TELEMETRY_MAVLINK
    // Support for misusing the heading field in MAVlink to indicate mAh drawn for Connex Prosight OSD
    // Set to 10 to show a tenth of your capacity drawn.
    // Set to $size_of_battery to get a percentage of battery used.
    { "mavlink_mah_as_heading_divisor", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 30000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, mavlink_mah_as_heading_divisor) },
#endif
#ifdef USE_TELEMETRY_SENSORS_DISABLED_DETAILS
    { "telemetry_disabled_voltage",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_VOLTAGE),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_current",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_CURRENT),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_fuel",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_FUEL),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_mode",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_MODE),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_acc_x",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_X),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_acc_y",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_Y),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_acc_z",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_Z),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_pitch",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_PITCH),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_roll",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ROLL),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_heading",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_HEADING),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_altitude",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ALTITUDE),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_vario",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_VARIO),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_lat_long",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_LAT_LONG),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_ground_speed",    VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_GROUND_SPEED),    PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_distance",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_DISTANCE),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_esc_current",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_CURRENT),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_esc_voltage",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_VOLTAGE),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_esc_rpm",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_RPM),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_esc_temperature", VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_TEMPERATURE), PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_temperature",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_TEMPERATURE),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
#else
    { "telemetry_disabled_sensors", VAR_UINT32 | MASTER_VALUE, .config.u32Max = SENSOR_ALL, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
#endif
#endif // USE_TELEMETRY

// PG_LED_STRIP_CONFIG
#ifdef USE_LED_STRIP
    { "ledstrip_visual_beeper",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_visual_beeper) },
    { "ledstrip_visual_beeper_color",VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_visual_beeper_color) },
    { "ledstrip_grb_rgb",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RGB_GRB }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_grb_rgb) },
    { "ledstrip_profile",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LED_PROFILE }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_profile) },
    { "ledstrip_race_color",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_race_color) },
    { "ledstrip_beacon_color",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_color) },
    { "ledstrip_beacon_period_ms",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 10000 }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_period_ms) },
    { "ledstrip_beacon_percent",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_percent) },
    { "ledstrip_beacon_armed_only", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_armed_only) },
#endif

// PG_SDCARD_CONFIG
#ifdef USE_SDCARD
    { "sdcard_detect_inverted",     VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, cardDetectInverted) },
    { "sdcard_mode",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SDCARD_MODE }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, mode) },
#endif
#ifdef USE_SDCARD_SPI
    { "sdcard_dma",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, useDma) },
    { "sdcard_spi_bus",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, device) },
#endif
#ifdef USE_SDCARD_SDIO
    { "sdio_clk_bypass",            VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, clockBypass) },
    { "sdio_use_cache",             VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, useCache) },
    { "sdio_use_4bit_width",        VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, use4BitWidth) },
#endif

// PG_OSD_CONFIG
#ifdef USE_OSD
    { "osd_units",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_OSD_CONFIG, offsetof(osdConfig_t, units) },

// Please try to keep the OSD warnings in the same order as presented in the Configurator.
// This makes it easier for the user to relate the CLI output as warnings are in the same relative
// position as displayed in the GUI.
    { "osd_warn_arming_disable",    VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_ARMING_DISABLE,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
    { "osd_warn_batt_not_full",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_BATTERY_NOT_FULL, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
    { "osd_warn_batt_warning",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_BATTERY_WARNING,  PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
    { "osd_warn_batt_critical",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_BATTERY_CRITICAL, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
    { "osd_warn_visual_beeper",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_VISUAL_BEEPER,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
    { "osd_warn_crash_flip",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_CRASH_FLIP,       PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
    { "osd_warn_esc_fail",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_ESC_FAIL,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
#ifdef USE_ADC_INTERNAL
    { "osd_warn_core_temp",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_CORE_TEMPERATURE, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
#endif
#ifdef USE_RC_SMOOTHING_FILTER
    { "osd_warn_rc_smoothing",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_RC_SMOOTHING,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
#endif
    { "osd_warn_fail_safe",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_FAIL_SAFE,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
#ifdef USE_LAUNCH_CONTROL
    { "osd_warn_launch_control",    VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_LAUNCH_CONTROL,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
#endif
    { "osd_warn_no_gps_rescue",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_GPS_RESCUE_UNAVAILABLE, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
    { "osd_warn_gps_rescue_disabled", VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_GPS_RESCUE_DISABLED, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
    { "osd_warn_rssi",              VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_RSSI,             PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
#ifdef USE_RX_LINK_QUALITY_INFO
    { "osd_warn_link_quality",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_LINK_QUALITY,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},
#endif

    { "osd_rssi_alarm",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_OSD_CONFIG, offsetof(osdConfig_t, rssi_alarm) },
#ifdef USE_RX_LINK_QUALITY_INFO
    { "osd_link_quality_alarm",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 300 }, PG_OSD_CONFIG, offsetof(osdConfig_t, link_quality_alarm) },
#endif
#ifdef USE_RX_RSSI_DBM
    { "osd_rssi_dbm_alarm",         VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 130 }, PG_OSD_CONFIG, offsetof(osdConfig_t, rssi_dbm_alarm) },
#endif
    { "osd_cap_alarm",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 20000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, cap_alarm) },
    { "osd_alt_alarm",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, alt_alarm) },
    { "osd_esc_temp_alarm",         VAR_INT8   | MASTER_VALUE, .config.minmax = { INT8_MIN, INT8_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_temp_alarm) },
    { "osd_esc_rpm_alarm",          VAR_INT16  | MASTER_VALUE, .config.minmax = { ESC_RPM_ALARM_OFF, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_rpm_alarm) },
    { "osd_esc_current_alarm",      VAR_INT16  | MASTER_VALUE, .config.minmax = { ESC_CURRENT_ALARM_OFF, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_current_alarm) },
#ifdef USE_ADC_INTERNAL
    { "osd_core_temp_alarm",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, core_temp_alarm) },
#endif

    { "osd_ah_max_pit",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahMaxPitch) },
    { "osd_ah_max_rol",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahMaxRoll) },
    { "osd_ah_invert",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahInvert) },

    { "osd_tim1",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, timers[OSD_TIMER_1]) },
    { "osd_tim2",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, timers[OSD_TIMER_2]) },

    { "osd_vbat_pos",               VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MAIN_BATT_VOLTAGE]) },
    { "osd_rssi_pos",               VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_RSSI_VALUE]) },
#ifdef USE_RX_LINK_QUALITY_INFO
    { "osd_link_quality_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_LINK_QUALITY]) },
#endif
#ifdef USE_RX_RSSI_DBM
    { "osd_rssi_dbm_pos",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_RSSI_DBM_VALUE]) },
#endif
    { "osd_tim_1_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ITEM_TIMER_1]) },
    { "osd_tim_2_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ITEM_TIMER_2]) },
    { "osd_remaining_time_estimate_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_REMAINING_TIME_ESTIMATE]) },
    { "osd_flymode_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_FLYMODE]) },
    { "osd_anti_gravity_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ANTI_GRAVITY]) },
    { "osd_g_force_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_G_FORCE]) },
    { "osd_throttle_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_THROTTLE_POS]) },
    { "osd_vtx_channel_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_VTX_CHANNEL]) },
    { "osd_crosshairs_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CROSSHAIRS]) },
    { "osd_ah_sbar_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_HORIZON_SIDEBARS]) },
    { "osd_ah_pos",                 VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ARTIFICIAL_HORIZON]) },
    { "osd_current_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CURRENT_DRAW]) },
    { "osd_mah_drawn_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MAH_DRAWN]) },
    { "osd_motor_diag_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MOTOR_DIAG]) },
    { "osd_craft_name_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CRAFT_NAME]) },
    { "osd_display_name_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_DISPLAY_NAME]) },
    { "osd_gps_speed_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_SPEED]) },
    { "osd_gps_lon_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_LON]) },
    { "osd_gps_lat_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_LAT]) },
    { "osd_gps_sats_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_SATS]) },
    { "osd_home_dir_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_HOME_DIR]) },
    { "osd_home_dist_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_HOME_DIST]) },
    { "osd_flight_dist_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_FLIGHT_DIST]) },
    { "osd_compass_bar_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_COMPASS_BAR]) },
    { "osd_altitude_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ALTITUDE]) },
    { "osd_pid_roll_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ROLL_PIDS]) },
    { "osd_pid_pitch_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PITCH_PIDS]) },
    { "osd_pid_yaw_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_YAW_PIDS]) },
    { "osd_debug_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_DEBUG]) },
    { "osd_power_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_POWER]) },
    { "osd_pidrate_profile_pos",    VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PIDRATE_PROFILE]) },
    { "osd_warnings_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_WARNINGS]) },
    { "osd_avg_cell_voltage_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_AVG_CELL_VOLTAGE]) },
    { "osd_pit_ang_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PITCH_ANGLE]) },
    { "osd_rol_ang_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ROLL_ANGLE]) },
    { "osd_battery_usage_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MAIN_BATT_USAGE]) },
    { "osd_disarmed_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_DISARMED]) },
    { "osd_nheading_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_NUMERICAL_HEADING]) },
#ifdef USE_VARIO
    { "osd_nvario_pos",             VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_NUMERICAL_VARIO]) },
#endif
    { "osd_esc_tmp_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ESC_TMP]) },
    { "osd_esc_rpm_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ESC_RPM]) },
    { "osd_esc_rpm_freq_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ESC_RPM_FREQ]) },
    { "osd_rtc_date_time_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_RTC_DATETIME]) },
    { "osd_adjustment_range_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ADJUSTMENT_RANGE]) },
    { "osd_flip_arrow_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_FLIP_ARROW]) },
#ifdef USE_ADC_INTERNAL
    { "osd_core_temp_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CORE_TEMPERATURE]) },
#endif
#ifdef USE_BLACKBOX
    { "osd_log_status_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_LOG_STATUS]) },
#endif

#ifdef USE_OSD_STICK_OVERLAY
    { "osd_stick_overlay_left_pos",    VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_STICK_OVERLAY_LEFT]) },
    { "osd_stick_overlay_right_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_STICK_OVERLAY_RIGHT]) },

    { "osd_stick_overlay_radio_mode",  VAR_UINT8   | MASTER_VALUE, .config.minmaxUnsigned = { 1, 4 }, PG_OSD_CONFIG, offsetof(osdConfig_t, overlay_radio_mode) },
#endif

#ifdef USE_PROFILE_NAMES
    { "osd_rate_profile_name_pos",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_RATE_PROFILE_NAME]) },
    { "osd_pid_profile_name_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PID_PROFILE_NAME]) },
#endif

#ifdef USE_OSD_PROFILES
    { "osd_profile_name_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PROFILE_NAME]) },
#endif


#ifdef USE_MAX7456_EXTENDED
    { "osd_pilot_logo_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PILOT_LOGO]) },
#endif
    // OSD stats enabled flags are stored as bitmapped values inside a 32bit parameter
    // It is recommended to keep the settings order the same as the enumeration. This way the settings are displayed in the cli in the same order making it easier on the users
    { "osd_stat_rtc_date_time",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_RTC_DATE_TIME,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_tim_1",             VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TIMER_1,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_tim_2",             VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TIMER_2,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_max_spd",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_SPEED,       PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_max_dist",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_DISTANCE,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_min_batt",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MIN_BATTERY,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_endbatt",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_END_BATTERY,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_battery",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_BATTERY,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_min_rssi",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MIN_RSSI,        PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_max_curr",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_CURRENT,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_used_mah",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_USED_MAH,        PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_max_alt",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_ALTITUDE,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_bbox",              VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_BLACKBOX,        PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_bb_no",             VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_BLACKBOX_NUMBER, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_max_g_force",       VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_G_FORCE,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_max_esc_temp",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_ESC_TEMP,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_max_esc_rpm",       VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_ESC_RPM,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_min_link_quality",  VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MIN_LINK_QUALITY,PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_flight_dist",       VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_FLIGHT_DISTANCE, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
#ifdef USE_GYRO_DATA_ANALYSE
    { "osd_stat_max_fft",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_FFT, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
#endif
#ifdef USE_PERSISTENT_STATS
    { "osd_stat_total_flights",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TOTAL_FLIGHTS, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_total_time",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TOTAL_TIME,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
    { "osd_stat_total_dist",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TOTAL_DIST,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
#endif
#ifdef USE_RX_RSSI_DBM
    { "osd_stat_min_rssi_dbm",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MIN_RSSI_DBM,  PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},
#endif

#ifdef USE_OSD_PROFILES
    { "osd_profile",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, OSD_PROFILE_COUNT }, PG_OSD_CONFIG, offsetof(osdConfig_t, osdProfileIndex) },
    { "osd_profile_1_name",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[0]) },
    { "osd_profile_2_name",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[1]) },
    { "osd_profile_3_name",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[2]) },
#endif
#endif

// PG_SYSTEM_CONFIG
#if defined(STM32F4)
    { "system_hse_mhz",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 30 }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, hseMhz) },
#endif
#if defined(USE_TASK_STATISTICS)
    { "task_statistics",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, task_statistics) },
#endif
    { "debug_mode",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DEBUG }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, debug_mode) },
    { "rate_6pos_switch",           VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, rateProfile6PosSwitch) },
#ifdef USE_OVERCLOCK
    { "cpu_overclock",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OVERCLOCK }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, cpu_overclock) },
#endif
    { "pwr_on_arm_grace",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 30 }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, powerOnArmingGraceTime) },
    { "scheduler_optimize_rate",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, schedulerOptimizeRate) },

// PG_VTX_CONFIG
#ifdef USE_VTX_COMMON
    { "vtx_band",                   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_BANDS }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, band) },
    { "vtx_channel",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_CHANNELS }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, channel) },
    { "vtx_power",                  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_POWER_LEVELS - 1 }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, power) },
    { "vtx_low_power_disarm",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VTX_LOW_POWER_DISARM }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, lowPowerDisarm) },
#ifdef VTX_SETTINGS_FREQCMD
    { "vtx_freq",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_SETTINGS_MAX_FREQUENCY_MHZ }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, freq) },
    { "vtx_pit_mode_freq",          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_SETTINGS_MAX_FREQUENCY_MHZ }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, pitModeFreq) },
#endif
#endif

// PG_VTX_CONFIG
#if defined(USE_VTX_CONTROL) && defined(USE_VTX_COMMON)
    { "vtx_halfduplex",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_VTX_CONFIG, offsetof(vtxConfig_t, halfDuplex) },
#endif

// PG_VTX_IO
#ifdef USE_VTX_RTC6705
    { "vtx_spi_bus",                VAR_UINT8  | HARDWARE_VALUE | MASTER_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_VTX_IO_CONFIG, offsetof(vtxIOConfig_t, spiDevice) },
#endif

// PG_VCD_CONFIG
#ifdef USE_MAX7456
    { "vcd_video_system",           VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VIDEO_SYSTEM }, PG_VCD_CONFIG, offsetof(vcdProfile_t, video_system) },
    { "vcd_h_offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -32, 31 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, h_offset) },
    { "vcd_v_offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -15, 16 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, v_offset) },
#endif
#ifdef USE_MAX7456_EXTENDED
    { "vcd_video_system_extended",           VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VIDEO_SYSTEM_EXTENDED }, PG_VCD_CONFIG, offsetof(vcdProfile_t, video_system_extended) },
#endif

// PG_MAX7456_CONFIG
#ifdef USE_MAX7456
    { "max7456_clock",              VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAX7456_CLOCK }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, clockConfig) },
    { "max7456_spi_bus",            VAR_UINT8   | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, spiDevice) },
    { "max7456_preinit_opu",        VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, preInitOPU) },
#endif

// PG_DISPLAY_PORT_MSP_CONFIG
#ifdef USE_MSP_DISPLAYPORT
    { "displayport_msp_col_adjust", VAR_INT8    | MASTER_VALUE, .config.minmax = { -6, 0 }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, colAdjust) },
    { "displayport_msp_row_adjust", VAR_INT8    | MASTER_VALUE, .config.minmax = { -3, 0 }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, rowAdjust) },
#endif

// PG_DISPLAY_PORT_MSP_CONFIG
#ifdef USE_MAX7456
    { "displayport_max7456_col_adjust", VAR_INT8| MASTER_VALUE, .config.minmax = { -6, 0 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, colAdjust) },
    { "displayport_max7456_row_adjust", VAR_INT8| MASTER_VALUE, .config.minmax = { -3, 0 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, rowAdjust) },
    { "displayport_max7456_inv",        VAR_UINT8| MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, invert) },
    { "displayport_max7456_blk",        VAR_UINT8| MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, blackBrightness) },
    { "displayport_max7456_wht",        VAR_UINT8| MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, whiteBrightness) },
#endif

#ifdef USE_ESC_SENSOR
    { "esc_sensor_halfduplex",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ESC_SENSOR_CONFIG, offsetof(escSensorConfig_t, halfDuplex) },
    { "esc_sensor_current_offset",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_ESC_SENSOR_CONFIG, offsetof(escSensorConfig_t, offset) },
#endif

#ifdef USE_RX_FRSKY_SPI
    { "frsky_spi_autobind",             VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, autoBind) },
    { "frsky_spi_tx_id",                VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 2, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindTxId) },
    { "frsky_spi_offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -127, 127 }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindOffset) },
    { "frsky_spi_bind_hop_data",        VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 50, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindHopData) },
    { "frsky_x_rx_num",                 VAR_UINT8   | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, rxNum) },
    { "frsky_spi_a1_source",            VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RX_FRSKY_SPI_A1_SOURCE }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, a1Source) },
    { "cc2500_spi_chip_detect",         VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, chipDetectEnabled) },
#endif
    { "led_inversion",                  VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, ((1 << STATUS_LED_NUMBER) - 1) }, PG_STATUS_LED_CONFIG, offsetof(statusLedConfig_t, inversion) },
#ifdef USE_DASHBOARD
    { "dashboard_i2c_bus",           VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_DASHBOARD_CONFIG, offsetof(dashboardConfig_t, device) },
    { "dashboard_i2c_addr",          VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { I2C_ADDR7_MIN, I2C_ADDR7_MAX }, PG_DASHBOARD_CONFIG, offsetof(dashboardConfig_t, address) },
#endif

// PG_CAMERA_CONTROL_CONFIG
#ifdef USE_CAMERA_CONTROL
    { "camera_control_mode", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CAMERA_CONTROL_MODE }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, mode) },
    { "camera_control_ref_voltage", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 200, 400 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, refVoltage) },
    { "camera_control_key_delay", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 500 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, keyDelayMs) },
    { "camera_control_internal_resistance", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 10, 1000 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, internalResistance) },
    { "camera_control_inverted", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, inverted) },
#endif

// PG_RANGEFINDER_CONFIG
#ifdef USE_RANGEFINDER
    { "rangefinder_hardware", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RANGEFINDER_HARDWARE }, PG_RANGEFINDER_CONFIG, offsetof(rangefinderConfig_t, rangefinder_hardware) },
#endif

// PG_PINIO_CONFIG
#ifdef USE_PINIO
    { "pinio_config", VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = PINIO_COUNT, PG_PINIO_CONFIG, offsetof(pinioConfig_t, config) },
#ifdef USE_PINIOBOX
    { "pinio_box", VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = PINIO_COUNT, PG_PINIOBOX_CONFIG, offsetof(pinioBoxConfig_t, permanentId) },
#endif
#endif

//PG USB
#ifdef USE_USB_CDC_HID
    { "usb_hid_cdc", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_USB_CONFIG, offsetof(usbDev_t, type) },
#endif
#ifdef USE_USB_MSC
    { "usb_msc_pin_pullup", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_USB_CONFIG, offsetof(usbDev_t, mscButtonUsePullup) },
#endif
// PG_FLASH_CONFIG
#ifdef USE_FLASH_CHIP
    { "flash_spi_bus", VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_FLASH_CONFIG, offsetof(flashConfig_t, spiDevice) },
#endif
// RCDEVICE
#ifdef USE_RCDEVICE
    { "rcdevice_init_dev_attempts", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10 }, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, initDeviceAttempts) },
    { "rcdevice_init_dev_attempt_interval", VAR_UINT32 | MASTER_VALUE, .config.u32Max = 5000, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, initDeviceAttemptInterval) },
    { "rcdevice_protocol_version", VAR_UINT8 | MASTER_VALUE, .config.minmax = { 0, 1 }, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, protocolVersion) },
    { "rcdevice_feature", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = {0, 65535}, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, feature) },
#endif

// PG_GYRO_DEVICE_CONFIG
    { "gyro_1_bustype", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, bustype) },
    { "gyro_1_spibus",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, spiBus) },
    { "gyro_1_i2cBus",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, i2cBus) },
    { "gyro_1_i2c_address", VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, i2cAddress) },
    { "gyro_1_sensor_align", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, align) },
#ifdef USE_MULTI_GYRO
    { "gyro_2_bustype", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, bustype) },
    { "gyro_2_spibus",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, spiBus) },
    { "gyro_2_i2cBus",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, i2cBus) },
    { "gyro_2_i2c_address", VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, i2cAddress) },
    { "gyro_2_sensor_align", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, align) },
#endif
#ifdef I2C_FULL_RECONFIGURABILITY
#ifdef USE_I2C_DEVICE_1
    { "i2c1_pullup",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 0, pullUp) },
    { "i2c1_overclock", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 0, overClock) },
#endif
#ifdef USE_I2C_DEVICE_2
    { "i2c2_pullup",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 1, pullUp) },
    { "i2c2_overclock", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 1, overClock) },
#endif
#ifdef USE_I2C_DEVICE_3
    { "i2c3_pullup",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 2, pullUp) },
    { "i2c3_overclock", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 2, overClock) },
#endif
#ifdef USE_I2C_DEVICE_4
    { "i2c4_pullup",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 3, pullUp) },
    { "i2c4_overclock", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 3, overClock) },
#endif
#endif
#ifdef USE_MCO
    { "mco2_on_pc9",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MCO_CONFIG, offsetof(mcoConfig_t, enabled[1]) },
#endif
#ifdef USE_RX_SPEKTRUM
    { "spektrum_spi_protocol",     VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, protocol) },
    { "spektrum_spi_mfg_id",       VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = 4, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, mfgId) },
    { "spektrum_spi_num_channels", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, DSM_MAX_CHANNEL_COUNT }, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, numChannels) },
#endif

// PG_TIMECONFIG
#ifdef USE_RTC_TIME
    { "timezone_offset_minutes",  VAR_INT16 | MASTER_VALUE, .config.minmax = { TIMEZONE_OFFSET_MINUTES_MIN, TIMEZONE_OFFSET_MINUTES_MAX }, PG_TIME_CONFIG, offsetof(timeConfig_t, tz_offsetMinutes) },
#endif

#ifdef USE_RPM_FILTER
    { "gyro_rpm_notch_harmonics",  VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, gyro_rpm_notch_harmonics) },
    { "gyro_rpm_notch_q",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1, 3000 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, gyro_rpm_notch_q) },
    { "gyro_rpm_notch_min",  VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 200 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, gyro_rpm_notch_min) },
    { "dterm_rpm_notch_harmonics",  VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, dterm_rpm_notch_harmonics) },
    { "dterm_rpm_notch_q",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1, 3000 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, dterm_rpm_notch_q) },
    { "dterm_rpm_notch_min",  VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 200 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, dterm_rpm_notch_min) },
    { "rpm_notch_lpf",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 500 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, rpm_lpf) },
#endif

#ifdef USE_RX_FLYSKY
    { "flysky_spi_tx_id",       VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_FLYSKY_CONFIG, offsetof(flySkyConfig_t, txId) },
    { "flysky_spi_rf_channels", VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = 16, PG_FLYSKY_CONFIG, offsetof(flySkyConfig_t, rfChannelMap) },
#endif

#ifdef USE_PERSISTENT_STATS
    { "stats",                  VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_enabled) },
    { "stats_total_flights",    VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_flights) },

    { "stats_total_time_s",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_time_s) },
    { "stats_total_dist_m",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_dist_m) },
#endif
    { "name",             VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, MAX_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PILOT_CONFIG, offsetof(pilotConfig_t, name) },
#ifdef USE_OSD
    { "display_name",     VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, MAX_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PILOT_CONFIG, offsetof(pilotConfig_t, displayName) },
#endif

// PG_POSITION
    { "position_alt_source",           VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_POSITION_ALT_SOURCE }, PG_POSITION, offsetof(positionConfig_t, altSource) },
};

const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);

void settingsBuildCheck() {
    STATIC_ASSERT(LOOKUP_TABLE_COUNT == ARRAYLEN(lookupTables), LOOKUP_TABLE_COUNT_incorrect);
}
