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

#define USE_PARAMETER_GROUPS
// type conversion warnings.
// -Wconversion can be turned on to enable the process of eliminating these warnings
//#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
// -Wpadded can be turned on to check padding of structs
//#pragma GCC diagnostic warning "-Wpadded"

//#define SCHEDULER_DEBUG // define this to use scheduler debug[] values. Undefined by default for performance reasons

#ifdef STM32F1
#define MINIMAL_CLI
// Using RX DMA disables the use of receive callbacks
#define USE_UART1_RX_DMA
#define USE_UART1_TX_DMA
#endif

#ifdef STM32F3
#define MINIMAL_CLI
#define USE_DSHOT
#define USE_GYRO_DATA_ANALYSE
#define USE_CCM_CODE
#endif

#ifdef STM32F4
#if defined(STM32F40_41xxx)
#define USE_FAST_DATA
#endif
#define USE_DSHOT
#define USE_DSHOT_BITBANG
#define USE_DSHOT_TELEMETRY
#define USE_DSHOT_TELEMETRY_STATS
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_GYRO_DATA_ANALYSE
#define USE_ADC
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_DMA_SPEC
#define USE_TIMER_MGMT
#define USE_PERSISTENT_OBJECTS
#define USE_CUSTOM_DEFAULTS_ADDRESS
#define USE_SPI_TRANSACTION

#if defined(STM32F40_41xxx) || defined(STM32F411xE)
#define USE_OVERCLOCK
#endif

#endif // STM32F4

#ifdef STM32F7
#define USE_ITCM_RAM
#define USE_FAST_DATA
#define USE_DSHOT
#define USE_DSHOT_BITBANG
#define USE_DSHOT_TELEMETRY
#define USE_DSHOT_TELEMETRY_STATS
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_GYRO_DATA_ANALYSE
#define USE_OVERCLOCK
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_DMA_SPEC
#define USE_TIMER_MGMT
#define USE_PERSISTENT_OBJECTS
#define USE_CUSTOM_DEFAULTS_ADDRESS
#define USE_SPI_TRANSACTION
#endif // STM32F7

#ifdef STM32H7
#define USE_ITCM_RAM
#define USE_FAST_DATA
#define USE_DSHOT
#define USE_DSHOT_BITBANG
#define USE_DSHOT_TELEMETRY
#define USE_DSHOT_TELEMETRY_STATS
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_GYRO_DATA_ANALYSE
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_DMA_SPEC
#define USE_TIMER_MGMT
#define USE_PERSISTENT_OBJECTS
#define USE_DMA_RAM
#define USE_USB_MSC
#define USE_RTC_TIME
#define USE_PERSISTENT_MSC_RTC
#define USE_DSHOT_CACHE_MGMT
#define USE_LATE_TASK_STATISTICS
#endif

#ifdef STM32G4
#define USE_FAST_RAM
#define USE_DSHOT
#define USE_DSHOT_BITBANG
#define USE_DSHOT_TELEMETRY
#define USE_DSHOT_TELEMETRY_STATS
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_OVERCLOCK
#define USE_GYRO_DATA_ANALYSE
#define USE_ADC_INTERNAL
#define USE_USB_MSC
#define USE_USB_CDC_HID
#define USE_MCO
#define USE_DMA_SPEC
#define USE_TIMER_MGMT
#endif

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT       MAX_AUX_CHANNEL_COUNT
#else
#define DEFAULT_AUX_CHANNEL_COUNT       6
#endif

// Set the default cpu_overclock to the first level (108MHz) for F411
// Helps with looptime stability as the CPU is borderline when running native gyro sampling
#if defined(USE_OVERCLOCK) && defined(STM32F411xE)
#define DEFAULT_CPU_OVERCLOCK 1
#else
#define DEFAULT_CPU_OVERCLOCK 0
#endif


#ifdef USE_ITCM_RAM
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#define FAST_CODE_NOINLINE          NOINLINE
#else
#define FAST_CODE
#define FAST_CODE_NOINLINE
#endif // USE_ITCM_RAM

#ifdef USE_CCM_CODE
#define CCM_CODE              __attribute__((section(".ccm_code")))
#else
#define CCM_CODE
#endif

#ifdef USE_FAST_DATA
#define FAST_DATA_ZERO_INIT          __attribute__ ((section(".fastram_bss"), aligned(4)))
#define FAST_DATA                    __attribute__ ((section(".fastram_data"), aligned(4)))
#else
#define FAST_DATA_ZERO_INIT
#define FAST_DATA
#endif // USE_FAST_DATA

#if defined(STM32F4)
// F4 can't DMA to/from CCM (core coupled memory) SRAM (where the stack lives)
#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define DMA_DATA_AUTO               static
#elif defined (STM32F7)
// F7 has no cache coherency issues DMAing to/from DTCM, otherwise buffers must be cache aligned
#define DMA_DATA_ZERO_INIT          FAST_DATA_ZERO_INIT
#define DMA_DATA                    FAST_DATA
#define DMA_DATA_AUTO               static DMA_DATA
#else
// DMA to/from any memory
#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define DMA_DATA_AUTO               static DMA_DATA
#endif

#if defined(STM32F4) || defined (STM32H7)
// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))
#endif

#ifdef USE_DMA_RAM
#if defined(STM32H7)
#define DMA_RAM __attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI __attribute__((section(".DMA_RW_AXI"), aligned(32)))
extern uint8_t _dmaram_start__;
extern uint8_t _dmaram_end__;
#elif defined(STM32G4)
#define DMA_RAM_R __attribute__((section(".DMA_RAM_R")))
#define DMA_RAM_W __attribute__((section(".DMA_RAM_W")))
#define DMA_RAM_RW __attribute__((section(".DMA_RAM_RW")))
#endif
#else
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW
#endif

#define USE_BRUSHED_ESC_AUTODETECT  // Detect if brushed motors are connected and set defaults appropriately to avoid motors spinning on boot

#define USE_MOTOR
#define USE_PWM_OUTPUT
#define USE_DMA
#define USE_TIMER

#define USE_CLI
#define USE_SERIAL_PASSTHROUGH
#define USE_TASK_STATISTICS
#define USE_GYRO_REGISTER_DUMP  // Adds gyroregisters command to cli to dump configured register values
#define USE_IMU_CALC
#define USE_PPM
#define USE_SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_GHST       // ImmersionRC Ghost Protocol
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SUMD       // Graupner Hott protocol

#if (TARGET_FLASH_SIZE > 128)
#define PID_PROFILE_COUNT 3
#define CONTROL_RATE_PROFILE_COUNT  6
#else
#define PID_PROFILE_COUNT 2
#define CONTROL_RATE_PROFILE_COUNT  3
#endif

#if (TARGET_FLASH_SIZE > 64)
#define USE_ACRO_TRAINER
#define USE_BLACKBOX
#define USE_CLI_BATCH
#define USE_RESOURCE_MGMT
#define USE_RUNAWAY_TAKEOFF     // Runaway Takeoff Prevention (anti-taz)
#define USE_SERVOS
#define USE_TELEMETRY
#define USE_TELEMETRY_FRSKY_HUB
#define USE_TELEMETRY_SMARTPORT
#endif

#if (TARGET_FLASH_SIZE > 128)
#define USE_GYRO_OVERFLOW_CHECK
#define USE_YAW_SPIN_RECOVERY
#define USE_DSHOT_DMAR
#define USE_SERIALRX_FPORT      // FrSky FPort
#define USE_TELEMETRY_CRSF
#define USE_TELEMETRY_GHST
#define USE_TELEMETRY_SRXL

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 12))
#define USE_CMS
#define USE_MSP_DISPLAYPORT
#define USE_MSP_OVER_TELEMETRY
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_LED_STRIP
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 11))
#define USE_VTX_COMMON
#define USE_VTX_CONTROL
#define USE_VTX_SMARTAUDIO
#define USE_VTX_TRAMP
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 10))
#define USE_VIRTUAL_CURRENT_METER
#define USE_CAMERA_CONTROL
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_RCDEVICE
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 9))
#define USE_GYRO_LPF2
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 8))
#define USE_LAUNCH_CONTROL
#define USE_DYN_LPF
#define USE_D_MIN
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 7))
#define USE_THROTTLE_BOOST
#define USE_INTEGRATED_YAW_CONTROL
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 6))
#define USE_ITERM_RELAX
#define USE_RC_SMOOTHING_FILTER
#define USE_THRUST_LINEARIZATION
#define USE_TPA_MODE
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 5))
#define USE_PWM
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 4))
#define USE_HUFFMAN
#define USE_PINIO
#define USE_PINIOBOX
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 3))
#ifdef USE_SERIALRX_SPEKTRUM
#define USE_SPEKTRUM_BIND
#define USE_SPEKTRUM_BIND_PLUG
#define USE_SPEKTRUM_REAL_RSSI
#define USE_SPEKTRUM_FAKE_RSSI
#define USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
#define USE_SPEKTRUM_VTX_CONTROL
#define USE_SPEKTRUM_VTX_TELEMETRY
#define USE_SPEKTRUM_CMS_TELEMETRY
#define USE_PIN_PULL_UP_DOWN
#endif
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 2))
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_LTM
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#endif

#if ((TARGET_FLASH_SIZE > 256) || (FEATURE_CUT_LEVEL < 1))
#define USE_BOARD_INFO
#define USE_EXTENDED_CMS_MENUS
#define USE_RTC_TIME
#define USE_RX_MSP
#define USE_ESC_SENSOR_INFO
#define USE_CRSF_CMS_TELEMETRY
#define USE_CRSF_LINK_STATISTICS
#define USE_RX_RSSI_DBM
#endif

#endif // TARGET_FLASH_SIZE > 128

#if (TARGET_FLASH_SIZE > 256)
#define USE_AIRMODE_LPF
#define USE_CANVAS
#define USE_FRSKYOSD
#define USE_GPS
#define USE_GPS_NMEA
#define USE_GPS_UBLOX
#define USE_GPS_RESCUE
#define USE_GYRO_DLPF_EXPERIMENTAL
#define USE_OSD
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_MULTI_GYRO
#define USE_OSD_ADJUSTMENTS
#define USE_SENSOR_NAMES
#define USE_SERIALRX_JETIEXBUS
#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_IBUS_EXTENDED
#define USE_TELEMETRY_JETIEXBUS
#define USE_TELEMETRY_MAVLINK
#define USE_UNCOMMON_MIXERS
#define USE_SIGNATURE
#define USE_ABSOLUTE_CONTROL
#define USE_HOTT_TEXTMODE
#define USE_LED_STRIP_STATUS_MODE
#define USE_VARIO
#define USE_RX_LINK_QUALITY_INFO
#define USE_ESC_SENSOR_TELEMETRY
#define USE_OSD_PROFILES
#define USE_OSD_STICK_OVERLAY
#define USE_CMS_FAILSAFE_MENU
#define USE_CMS_GPS_RESCUE_MENU
#define USE_TELEMETRY_SENSORS_DISABLED_DETAILS
#define USE_VTX_TABLE
#define USE_PERSISTENT_STATS
#define USE_PROFILE_NAMES
#define USE_SERIALRX_SRXL2     // Spektrum SRXL2 protocol
#define USE_FEEDFORWARD
#define USE_CUSTOM_BOX_NAMES
#define USE_BATTERY_VOLTAGE_SAG_COMPENSATION
#define USE_RX_MSP_OVERRIDE
#define USE_SIMPLIFIED_TUNING
#define USE_RX_LINK_UPLINK_POWER
#define USE_GPS_PLUS_CODES
#define USE_CRSF_V3
#endif

#if (TARGET_FLASH_SIZE > 512)
#define USE_ESCSERIAL_SIMONK
#define USE_SERIAL_4WAY_SK_BOOTLOADER
#define USE_DASHBOARD
#endif
