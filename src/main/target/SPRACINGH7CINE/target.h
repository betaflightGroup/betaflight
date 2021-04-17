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

#define TARGET_BOARD_IDENTIFIER "SP7C"
#define USBD_PRODUCT_STRING "SPRacingH7CINE"

#define SPRACINGH7CINE_REV 3 // REV C
//#define SPRACINGH7CINE_REV 2 // REV B
//#define SPRACINGH7CINE_REV 1 // REV A
#ifndef SPRACINGH7CINE_REV
#define SPRACINGH7CINE_REV 3 // REV C
#endif

#define USE_TARGET_CONFIG

#if (SPRACINGH7CINE_REV <= 2) // Rev A & B
#define LED0_PIN                PE3
#elif (SPRACINGH7CINE_REV == 3) // Rev C
#define LED0_PIN                PE4
#endif

#define USE_BEEPER
#if (SPRACINGH7CINE_REV <= 2) // Rev A & B
#define BEEPER_PIN              PD7
#elif (SPRACINGH7CINE_REV == 3) // Rev C
#define BEEPER_PIN              PB11
#endif
#define BEEPER_INVERTED

// Force two buttons to look at the single button so config reset on button works
#define USE_BUTTONS
#define BUTTON_A_PIN            PD10
#define BUTTON_A_PIN_INVERTED // Active high
#define BUTTON_B_PIN            PD10
#define BUTTON_B_PIN_INVERTED // Active high

#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1

#define QUADSPI1_SCK_PIN PB2

#define QUADSPI1_BK1_IO0_PIN PD11
#define QUADSPI1_BK1_IO1_PIN PD12
#define QUADSPI1_BK1_IO2_PIN PE2
#define QUADSPI1_BK1_IO3_PIN PD13
#define QUADSPI1_BK1_CS_PIN PB10

// BK2 is NC.
#define QUADSPI1_BK2_IO0_PIN PE7
#define QUADSPI1_BK2_IO1_PIN PE8
#define QUADSPI1_BK2_IO2_PIN PE9
#define QUADSPI1_BK2_IO3_PIN PE10
#define QUADSPI1_BK2_CS_PIN NONE

#define QUADSPI1_MODE QUADSPI_MODE_BK1_ONLY
#define QUADSPI1_CS_FLAGS (QUADSPI_BK1_CS_HARDWARE | QUADSPI_BK2_CS_NONE | QUADSPI_CS_MODE_LINKED)

#define USE_FLASH_CHIP
#define CONFIG_IN_EXTERNAL_FLASH
//#define CONFIG_IN_SDCARD
//#define CONFIG_IN_RAM
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH)
#error "CONFIG storage location not defined"
#endif

#define USE_UART

#define USE_UART1
#define UART1_RX_PIN            PB15
#define UART1_TX_PIN            PB14

#define USE_UART2
#define UART2_RX_PIN            NONE
#define UART2_TX_PIN            PD5   // TX pin is bidirectional.

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PD0
#define UART4_TX_PIN            PD1

#define USE_UART5
#define UART5_RX_PIN            PB5
#define UART5_TX_PIN            PB13

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_UART8
#define UART8_RX_PIN            PE0
#define UART8_TX_PIN            PE1

#define USE_VCP
#define USE_USB_ID

#define SERIAL_PORT_COUNT       8

#define USE_SPI

#if (SPRACINGH7CINE_REV == 1)
// SPI2 is NC.
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3
#define SPI2_NSS_PIN            PB12
#elif (SPRACINGH7CINE_REV == 2)
// SPI2 unavailable
#define SPI2_SCK_PIN            NONE
#define SPI2_MISO_PIN           NONE
#define SPI2_MOSI_PIN           NONE
#define SPI2_NSS_PIN            NONE
#elif (SPRACINGH7CINE_REV == 3)
// SPI2 is NC
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PC1
#define SPI2_MOSI_PIN           PC2
#define SPI2_NSS_PIN            PB12
#endif

// SPI3 for Gyro
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PD6
#define SPI3_NSS_PIN            PA15

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7
#define I2C_DEVICE              (I2CDEV_1)

#define USE_MAG
#define USE_MAG_HMC5883

#define USE_BARO
#define USE_BARO_BMP388
#define BARO_EXTI_PIN           PC13 // NOTE Unused.

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_ICM42605
#undef USE_GYRO_REGISTER_DUMP

#define USE_EXTI
#define USE_GYRO_EXTI
#if (SPRACINGH7CINE_REV <= 2) // Rev A & B
#define GYRO_1_EXTI_PIN         PB11 // ICM42605-INT1/INT
#elif (SPRACINGH7CINE_REV == 3) // Rev C
#define GYRO_1_EXTI_PIN         PD7  // ICM42605-INT1/INT
#endif
#define GYRO_1_INT2_PIN         PD4  // ICM42605-INT2/FSYNC // TODO - Ensure this pin is configured correctly on MCU and via Gyro registers.

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_CS_PIN           SPI3_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI3

#define USE_ACC
#define USE_ACC_SPI_ICM42605

#define GYRO_1_ALIGN        CW0_DEG

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

#define USE_FLASHFS
#define USE_FLASH_TOOLS
#define USE_FLASH_W25N01G
#define FLASH_QUADSPI_INSTANCE    QUADSPI

#define USE_PID_AUDIO

#define USE_TRANSPONDER

#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI
#define VTX_RTC6705_OPTIONAL    // VTX/OSD board is OPTIONAL

#define RTC6705_POWER_PIN                   PB1  // M5 / J4:3
#define RTC6705_CS_PIN                      PB0  // M6 / J4:4
#define RTC6705_SPICLK_PIN                  PA7  // M7 / J4:5
#define RTC6705_SPI_MOSI_PIN                PA6  // M8 / J4:6

#ifdef USE_DMA_SPEC
//#define UART1_TX_DMA_OPT        0
//#define UART2_TX_DMA_OPT        1
//#define UART3_TX_DMA_OPT        2
//#define UART4_TX_DMA_OPT        3
//#define UART5_TX_DMA_OPT        4
//#define UART6_TX_DMA_OPT        5
//#define UART7_TX_DMA_OPT        6
//#define UART8_TX_DMA_OPT        7
#define ADC1_DMA_OPT 8    // VIDEO                  - Specific DMA requirements, which cannot be shared with other ADC inputs.
#define ADC2_DMA_OPT 12   // RSSI                   - same DMA requirements as inputs on ADC3, optional.
#define ADC3_DMA_OPT 9    // TEMP/REF/BATTERY/CURRENT - same DMA requirements
#else
#define ADC1_DMA_STREAM DMA2_Stream0
#define ADC3_DMA_STREAM DMA2_Stream1
#define ADC2_DMA_STREAM DMA2_Stream4
#endif

#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_DETECT_PIN PD15
#define SDCARD_DETECT_INVERTED
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           true
#define SDIO_CK_PIN             PC12
#define SDIO_CMD_PIN            PD2
#define SDIO_D0_PIN             PC8
#define SDIO_D1_PIN             PC9
#define SDIO_D2_PIN             PC10
#define SDIO_D3_PIN             PC11

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define USE_ADC
#define USE_ADC_INTERNAL // ADC3

#if (SPRACINGH7CINE_REV <= 2)
#define ADC1_INSTANCE ADC1 // ADC1 for VIDEO ADC
#define ADC2_INSTANCE ADC2 // ADC2 for RSSI/CURRENT
#define ADC3_INSTANCE ADC3 // ADC3 for TEMP/REF/BATTERY/EXTERNAL

#define ADC_INSTANCE ADC3 // Use ADC3 by default, for as many pins as possible.

#define EXTERNAL1_ADC_PIN           PC0  // ADC123 aka CURRENT_B for 2nd 4in1 ESC
#define EXTERNAL1_ADC_INSTANCE      ADC3
#define CURRENT_METER_ADC_PIN       PC1  // ADC12
#define CURRENT_METER_ADC_INSTANCE  ADC2
#define VBAT_ADC_PIN                PC4  // ADC123
#define VBAT_ADC_INSTANCE           ADC3
#define RSSI_ADC_PIN                PC5  // ADC12
#define RSSI_ADC_INSTANCE           ADC2
#define VIDEO_ADC_PIN               PC3  // ADC12
#define VIDEO_ADC_INSTANCE          ADC1

#elif (SPRACINGH7CINE_REV == 3)
#define ADC1_INSTANCE ADC1 // ADC1 for Battery/Current/RSSI/Monitoring
#define ADC2_INSTANCE ADC2 // ADC2 for VIDEO ADC
#define ADC3_INSTANCE ADC3 // ADC3 only for core temp and vrefint

#define ADC_INSTANCE ADC3 // Use ADC3 by default, for as many pins as possible.

// FIXME the current betaflight ADC implementation will pick the first ADC instance that can be connected to each ADC pin
// however, ADC2 and ADC3 should be used for monitoring, and ADC1 should be dedicated to video so that a timer-triggered
// DMA stream can be used which should be independent from CURRENT/POWER/RSSI.
#define CURRENT_METER_ADC_PIN       PC0  // ADC3_IN10 CURRENT
#define CURRENT_METER_ADC_INSTANCE  ADC3
#define VBAT_ADC_PIN                PC3  // ADC3_INP1 POWER
#define VBAT_ADC_INSTANCE           ADC3
#define RSSI_ADC_PIN                PC4  // ADC2_INP4 RSSI
#define RSSI_ADC_INSTANCE           ADC2
#define VIDEO_ADC_PIN               PC5  // ADC1_INP8 VIDEO
#define VIDEO_ADC_INSTANCE          ADC1
#endif

#if (SPRACINGH7CINE_REV <= 2) // Rev A & B
#define CAM_SELECT_PIN          PE4
#elif (SPRACINGH7CINE_REV == 3) // Rev C
#define CAM_SELECT_PIN          PE3
#endif

#define USE_PINIO
#define PINIO1_PIN              CAM_SELECT_PIN
#define USE_PINIOBOX

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define DEBUG_BLANKING
#define DEBUG_GATING

// #define USE_SPRACING_PIXELOSD_DEBUG_OVERLAY
#if (SPRACINGH7CINE_REV == 1)
// Rev A
#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE13
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE14
#define SPRACING_PIXEL_OSD_RESERVED_PIN                 PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_A_PIN               PE12 // TIM1_CH3N
#define SPRACING_PIXEL_OSD_SYNC_OUT_B_PIN               PA8  // TIM1_CH1 / MCO / Currently Unused
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 SPRACING_PIXEL_OSD_SYNC_OUT_A_PIN

#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5  // DAC1_OUT2
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0 // TIM1_CH2N // actual gating is on CH4
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1 // TIM1_CH3N // actual blanking is on CH5
#elif (SPRACINGH7CINE_REV == 2)
// Rev B
#define SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN      PE12
#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE13
#define SPRACING_PIXEL_OSD_MASK_ENABLE_PIN              PE14
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 PA8  // TIM1_CH1

#define SPRACING_PIXEL_OSD_WHITE_SOURCE_PIN             PA4  // DAC1_OUT1
#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5  // DAC1_OUT2
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0 // TIM1_CH2N // actual gating is on CH4
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1 // TIM1_CH3N // actual blanking is on CH5
#elif (SPRACINGH7CINE_REV == 3)
// Rev C
#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE12
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE13
#define SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN      PE14
#define SPRACING_PIXEL_OSD_MASK_ENABLE_PIN              PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 PA8  // TIM1_CH1

#define SPRACING_PIXEL_OSD_WHITE_SOURCE_PIN             PA4  // DAC1_OUT1
#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5  // DAC1_OUT2
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0 // TIM1_CH2N // actual gating is on CH4
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1 // TIM1_CH3N // actual blanking is on CH5
#endif

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP)

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 17

// TIM1, TIM2, TIM15 reserved for OSD
#define USED_TIMERS  ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12) | TIM_N(15) | TIM_N(16) | TIM_N(17) )
