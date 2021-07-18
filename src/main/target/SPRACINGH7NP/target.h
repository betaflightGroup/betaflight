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

#define TARGET_BOARD_IDENTIFIER "SPNP"
#define USBD_PRODUCT_STRING "SPRacingH7NP"

#define USE_TARGET_CONFIG

#define LED0_PIN                PE4

#define USE_BEEPER
#define BEEPER_PIN              PD10
#define BEEPER_INVERTED

// Force two buttons to look at the single button so config reset on button works
#define USE_BUTTONS
#define BUTTON_A_PIN            PE3
#define BUTTON_A_PIN_INVERTED // Active high
#define BUTTON_B_PIN            PE3
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

// SPI2 is NC (Reserved)
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC1
#define SPI2_NSS_PIN            PB12
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

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_MULTI_GYRO          // CPU pinout supports it, but current hardware doesn't use it.
#undef USE_GYRO_REGISTER_DUMP

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PD7 // ICM20602/INT
#define GYRO_2_EXTI_PIN         PD4 // NC

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_CS_PIN           SPI3_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI3

#define GYRO_2_CS_PIN           SPI2_NSS_PIN
#define GYRO_2_SPI_INSTANCE     SPI2

#define USE_ACC
#define USE_ACC_SPI_MPU6500

#define GYRO_1_ALIGN        CW0_DEG

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

#define USE_FLASHFS
#define USE_FLASH_TOOLS
#define USE_FLASH_W25N01G
#define FLASH_QUADSPI_INSTANCE    QUADSPI

#define USE_PID_AUDIO

#define USE_TRANSPONDER

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
#define ADC2_DMA_STREAM DMA2_Stream4
#define ADC3_DMA_STREAM DMA2_Stream1
#endif

#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_DETECT_PIN       PD15
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
#define USE_ADC_INTERNAL        // ADC3

//#define ADC1_INSTANCE ADC1      // ADC1 for VIDEO
#define ADC2_INSTANCE ADC2      // ADC2 for RSSI
#define ADC3_INSTANCE ADC3      // ADC3 for TEMP/REF/BATTERY/CURRENT

#define CURRENT_METER_ADC_PIN   PC0  // ADC3_INP10
#define VBAT_ADC_PIN            PC3  // ADC3_INP1
#define RSSI_ADC_PIN            PC4  // ADC2_INP4
#define VIDEO_ADC_PIN           PC5  // ADC1_INP8

#define VTX_ENABLE_PIN          PB11

#define USE_PINIO
#define PINIO1_PIN              VTX_ENABLE_PIN
#define USE_PINIOBOX

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE12
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE13
#define SPRACING_PIXEL_OSD_MASK_ENABLE_PIN              PE14
#define SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN      PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 PA8  // TIM1_CH1

#define SPRACING_PIXEL_OSD_WHITE_SOURCE_PIN             PA4  // DAC1_OUT1
#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5  // DAC1_OUT2
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0 // TIM1_CH2N // actual gating is on CH4
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1 // TIM1_CH3N // actual blanking is on CH5

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP)
#define SERIALRX_PROVIDER       SERIALRX_SBUS

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
