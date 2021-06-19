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

// pg/max7456

#ifndef DEBUG_MODE
#define DEBUG_MODE DEBUG_NONE
#endif

#ifdef USE_MAX7456
#ifndef MAX7456_CLOCK_CONFIG_DEFAULT
#define MAX7456_CLOCK_CONFIG_DEFAULT    MAX7456_CLOCK_CONFIG_OC
#endif

#ifndef MAX7456_SPI_CS_PIN
#define MAX7456_SPI_CS_PIN              NONE
#endif

#ifndef MAX7456_SPI_INSTANCE
#define MAX7456_SPI_INSTANCE            NULL
#endif
#endif

// pg/flash

#ifdef USE_FLASH_M25P16
#ifndef FLASH_CS_PIN
#define FLASH_CS_PIN                    NONE
#endif

#ifndef FLASH_SPI_INSTANCE
#define FLASH_SPI_INSTANCE              NULL
#endif
#endif

// pg/flash

#ifdef USE_FLASH_M25P16
#ifndef FLASH_CS_PIN
#define FLASH_CS_PIN                    NONE
#endif

#ifndef FLASH_SPI_INSTANCE
#define FLASH_SPI_INSTANCE              NULL
#endif
#endif

// pg/bus_i2c

#ifdef I2C_FULL_RECONFIGURABILITY
#ifdef USE_I2C_DEVICE_1
#define I2C1_SCL NONE
#define I2C1_SDA NONE
#endif

#ifdef USE_I2C_DEVICE_2
#define I2C2_SCL NONE
#define I2C2_SDA NONE
#endif

#ifdef USE_I2C_DEVICE_3
#define I2C3_SCL NONE
#define I2C3_SDA NONE
#endif

#ifdef USE_I2C_DEVICE_4
#define I2C4_SCL NONE
#define I2C4_SDA NONE
#endif

#else // I2C_FULL_RECONFIGURABILITY

// Backward compatibility for exisiting targets

#ifdef STM32F1
#ifndef I2C1_SCL
#define I2C1_SCL PB8
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB9
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#endif // STM32F1

#ifdef STM32F3
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PA9
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PA10
#endif
#endif // STM32F3

#ifdef STM32F4
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PC9
#endif
#endif // STM32F4

#ifdef STM32F7
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PB4
#endif
#ifndef I2C4_SCL
#define I2C4_SCL PD12
#endif
#ifndef I2C4_SDA
#define I2C4_SDA PD13
#endif
#endif // STM32F7

#endif // I2C_FULL_RECONFIGURABILITY

#ifndef I2C1_CLOCKSPEED
#define I2C1_CLOCKSPEED 800
#endif
#ifndef I2C2_CLOCKSPEED
#define I2C2_CLOCKSPEED 800
#endif
#ifndef I2C3_CLOCKSPEED
#define I2C3_CLOCKSPEED 800
#endif
#ifndef I2C4_CLOCKSPEED
#define I2C4_CLOCKSPEED 800
#endif

// Default values for internal pullup

#if defined(USE_I2C_PULLUP)
#define I2C1_PULLUP true
#define I2C2_PULLUP true
#define I2C3_PULLUP true
#define I2C4_PULLUP true
#else
#define I2C1_PULLUP false
#define I2C2_PULLUP false
#define I2C3_PULLUP false
#define I2C4_PULLUP false
#endif

// pg/bus_spi

#ifdef SPI_FULL_RECONFIGURABILITY

#ifdef USE_SPI_DEVICE_1
#define SPI1_SCK_PIN    NONE
#define SPI1_MISO_PIN   NONE
#define SPI1_MOSI_PIN   NONE
#endif

#ifdef USE_SPI_DEVICE_2
#define SPI2_SCK_PIN    NONE
#define SPI2_MISO_PIN   NONE
#define SPI2_MOSI_PIN   NONE
#endif

#ifdef USE_SPI_DEVICE_3
#define SPI3_SCK_PIN    NONE
#define SPI3_MISO_PIN   NONE
#define SPI3_MOSI_PIN   NONE
#endif

#ifdef USE_SPI_DEVICE_4
#define SPI4_SCK_PIN    NONE
#define SPI4_MISO_PIN   NONE
#define SPI4_MOSI_PIN   NONE
#endif

#else

// Pin defaults for backward compatibility

#ifndef SPI1_SCK_PIN
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif

#ifndef SPI2_SCK_PIN
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

#ifndef SPI4_SCK_PIN
#define SPI4_SCK_PIN    NONE
#define SPI4_MISO_PIN   NONE
#define SPI4_MOSI_PIN   NONE
#endif

#ifndef SPI5_SCK_PIN
#define SPI5_SCK_PIN    NONE
#define SPI5_MISO_PIN   NONE
#define SPI5_MOSI_PIN   NONE
#endif

#ifndef SPI6_SCK_PIN
#define SPI6_SCK_PIN    NONE
#define SPI6_MISO_PIN   NONE
#define SPI6_MOSI_PIN   NONE
#endif

#endif

// Extracted from rx/rx.c and rx/rx.h

#define RX_MAPPABLE_CHANNEL_COUNT 8

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
#endif

#define RX_MIN_USEC 885
#define RX_MAX_USEC 2115
#define RX_MID_USEC 1500

#ifndef SPEKTRUM_BIND_PIN
#define SPEKTRUM_BIND_PIN NONE
#endif

#ifndef BINDPLUG_PIN
#define BINDPLUG_PIN NONE
#endif

#ifdef USE_RX_SPI
#if !defined(RX_SPI_INSTANCE)
#define RX_SPI_INSTANCE NULL
#endif

#if !defined(RX_NSS_PIN)
#define RX_NSS_PIN NONE
#endif

#ifndef RX_SPI_LED_PIN
#define RX_SPI_LED_PIN NONE
#endif

#if !defined(RX_SPI_EXTI_PIN)
#define RX_SPI_EXTI_PIN NONE
#endif

#if !defined(RX_SPI_BIND_PIN)
#define RX_SPI_BIND_PIN NONE
#endif

#if defined(USE_RX_CC2500)
#if !defined(RX_CC2500_SPI_TX_EN_PIN)
#define RX_CC2500_SPI_TX_EN_PIN NONE
#endif

#if !defined(RX_CC2500_SPI_LNA_EN_PIN)
#define RX_CC2500_SPI_LNA_EN_PIN NONE
#endif

#if !defined(RX_CC2500_SPI_ANT_SEL_PIN)
#define RX_CC2500_SPI_ANT_SEL_PIN NONE
#endif
#endif
#endif

// gyro hardware

#if !defined(GYRO_1_SPI_INSTANCE)
#define GYRO_1_SPI_INSTANCE     NULL
#endif

#if !defined(GYRO_1_CS_PIN)
#define GYRO_1_CS_PIN           NONE
#endif

#if !defined(GYRO_1_EXTI_PIN)
#define GYRO_1_EXTI_PIN         NONE
#endif

// F4 and F7 single gyro boards
#if defined(USE_MULTI_GYRO) && !defined(GYRO_2_SPI_INSTANCE)
#define GYRO_2_SPI_INSTANCE     NULL
#define GYRO_2_CS_PIN           NONE
#define GYRO_2_EXTI_PIN         NONE
#endif

#if defined(MPU_ADDRESS)
#define GYRO_I2C_ADDRESS MPU_ADDRESS
#else
#define GYRO_I2C_ADDRESS 0 // AUTO
#endif

#ifdef USE_MULTI_GYRO
#define MAX_GYRODEV_COUNT 2
#define MAX_ACCDEV_COUNT 2
#else
#define MAX_GYRODEV_COUNT 1
#define MAX_ACCDEV_COUNT 1
#endif

// gyro alignments

#if !defined(GYRO_1_ALIGN)
#define GYRO_1_ALIGN            CW0_DEG
#endif

#if !defined(GYRO_2_ALIGN)
#define GYRO_2_ALIGN            CW0_DEG
#endif

// Previously there was logic here to default GYRO_1_CUSTOM_ALIGN and GYRO_2_CUSTOM_ALIGN
// to CUSTOM_ALIGN_CW0_DEG if they weren't defined in the target. The defaulting logic
// has been moved to pg/gyrodev.c to set the custom alignment based on the sensor alignment
// if a custom alignment is not applied in the target.

#ifdef USE_VCP
#ifndef USB_DETECT_PIN
#define USB_DETECT_PIN NONE
#endif
#ifndef USB_MSC_BUTTON_PIN
#define USB_MSC_BUTTON_PIN NONE
#endif
#if !defined(MSC_BUTTON_IPU)
#define MSC_BUTTON_IPU true
#endif
#endif

#ifdef USE_TIMER_MGMT
#ifndef MAX_TIMER_PINMAP_COUNT
#define MAX_TIMER_PINMAP_COUNT 21 // Largest known for F405RG (OMNINXT)
#endif
#endif

#ifdef USE_SDCARD
#ifndef SDCARD_DETECT_PIN
#define SDCARD_DETECT_PIN NONE
#endif
#ifdef SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_IS_INVERTED 1
#else
#define SDCARD_DETECT_IS_INVERTED 0
#endif
#ifdef USE_SDCARD_SPI
#ifndef SDCARD_SPI_INSTANCE
#define SDCARD_SPI_INSTANCE NULL
#endif
#ifndef SDCARD_SPI_CS_PIN
#define SDCARD_SPI_CS_PIN NONE
#endif
#endif // USE_SDCARD_SPI
#ifdef USE_SDCARD_SDIO
#ifndef SDCARD_SDIO_DMA_OPT
#define SDCARD_SDIO_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SDIO_DEVICE
#define SDIO_DEVICE SDIOINVALID
#endif
#ifndef SDIO_USE_4BIT
#define SDIO_USE_4BIT false
#endif
#ifndef SDIO_CK_PIN
#define SDIO_CK_PIN NONE
#endif
#ifndef SDIO_CMD_PIN
#define SDIO_CMD_PIN NONE
#endif
#ifndef SDIO_D0_PIN
#define SDIO_D0_PIN NONE
#endif
#ifndef SDIO_D1_PIN
#define SDIO_D1_PIN NONE
#endif
#ifndef SDIO_D2_PIN
#define SDIO_D2_PIN NONE
#endif
#ifndef SDIO_D3_PIN
#define SDIO_D3_PIN NONE
#endif
#endif // USE_SDCARD_SDIO
#endif // USE_SDCARD

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8)
#define USE_UART
#endif

#ifdef USE_UART
#if defined(INVERTER_PIN_UART1) || defined(INVERTER_PIN_UART2) || defined(INVERTER_PIN_UART3) || defined(INVERTER_PIN_UART4) || defined(INVERTER_PIN_UART5) || defined(INVERTER_PIN_UART6)
#define USE_INVERTER
#endif
#endif

#ifndef DEFAULT_MIXER
#define DEFAULT_MIXER    MIXER_QUADX
#endif

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_HCSR04)
#ifndef RANGEFINDER_HCSR04_TRIGGER_PIN
#define RANGEFINDER_HCSR04_TRIGGER_PIN     NONE
#endif
#ifndef RANGEFINDER_HCSR04_ECHO_PIN
#define RANGEFINDER_HCSR04_ECHO_PIN        NONE
#endif
#endif

// Mag
#if defined(USE_MAG)
#ifndef MAG_SPI_INSTANCE
#define MAG_SPI_INSTANCE        NULL
#endif
#ifndef MAG_CS_PIN
#define MAG_CS_PIN              NONE
#endif
#ifndef MAG_I2C_INSTANCE
#define MAG_I2C_INSTANCE        I2C_DEVICE
#endif
#endif

#ifndef MAG_INT_EXTI
#define MAG_INT_EXTI            NONE
#endif

// Baro
#if defined(USE_BARO)
#ifndef BARO_SPI_INSTANCE
#define BARO_SPI_INSTANCE       NULL
#endif
#ifndef BARO_CS_PIN
#define BARO_CS_PIN             NONE
#endif
#ifndef BARO_I2C_INSTANCE
#define BARO_I2C_INSTANCE       I2C_DEVICE
#endif
#ifndef BARO_XCLR_PIN
#define BARO_XCLR_PIN           NONE
#endif
#endif

#ifdef USE_ADC
#if !defined(USE_UNIFIED_TARGET) && !defined(ADC_INSTANCE)
#define ADC_INSTANCE ADC1
#ifndef ADC1_DMA_OPT
#define ADC1_DMA_OPT 1
#endif
#endif

#if !defined(ADC1_DMA_OPT)
#define ADC1_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC2_DMA_OPT)
#define ADC2_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC3_DMA_OPT)
#define ADC3_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC4_DMA_OPT)
#define ADC4_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC5_DMA_OPT)
#define ADC5_DMA_OPT (DMA_OPT_UNUSED)
#endif

#endif // USE_ADC

#ifdef USE_SPI
#ifdef USE_SPI_DEVICE_1
#ifndef SPI1_TX_DMA_OPT
#define SPI1_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI1_RX_DMA_OPT
#define SPI1_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_2
#ifndef SPI2_TX_DMA_OPT
#define SPI2_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI2_RX_DMA_OPT
#define SPI2_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_3
#ifndef SPI3_TX_DMA_OPT
#define SPI3_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI3_RX_DMA_OPT
#define SPI3_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_4
#ifndef SPI4_TX_DMA_OPT
#define SPI4_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI4_RX_DMA_OPT
#define SPI4_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#endif

#ifdef USE_UART1
#ifndef UART1_TX_DMA_OPT
#define UART1_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART1_RX_DMA_OPT
#define UART1_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART2
#ifndef UART2_TX_DMA_OPT
#define UART2_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART2_RX_DMA_OPT
#define UART2_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART3
#ifndef UART3_TX_DMA_OPT
#define UART3_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART3_RX_DMA_OPT
#define UART3_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART4
#ifndef UART4_TX_DMA_OPT
#define UART4_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART4_RX_DMA_OPT
#define UART4_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART5
#ifndef UART5_TX_DMA_OPT
#define UART5_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART5_RX_DMA_OPT
#define UART5_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART6
#ifndef UART6_TX_DMA_OPT
#define UART6_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART6_RX_DMA_OPT
#define UART6_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART7
#ifndef UART7_TX_DMA_OPT
#define UART7_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART7_RX_DMA_OPT
#define UART7_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART8
#ifndef UART8_TX_DMA_OPT
#define UART8_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART8_RX_DMA_OPT
#define UART8_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART9
#ifndef UART9_TX_DMA_OPT
#define UART9_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART9_RX_DMA_OPT
#define UART9_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifdef USE_UART10
#ifndef UART10_TX_DMA_OPT
#define UART10_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART10_RX_DMA_OPT
#define UART10_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

#ifndef RTC6705_CS_PIN
#define RTC6705_CS_PIN NONE
#endif

#ifndef RTC6705_POWER_PIN
#define RTC6705_POWER_PIN NONE
#endif

#ifndef RTC6705_SPICLK_PIN
#define RTC6705_SPICLK_PIN NONE
#endif

#ifndef RTC6705_SPI_MOSI_PIN
#define RTC6705_SPI_MOSI_PIN NONE
#endif

#ifndef RTC6705_SPI_INSTANCE
#define RTC6705_SPI_INSTANCE NULL
#endif

#if defined(USE_QUAD_MIXER_ONLY)
#define MAX_SUPPORTED_MOTORS 4
#define MAX_SUPPORTED_SERVOS 1
#else
#ifndef MAX_SUPPORTED_MOTORS
#define MAX_SUPPORTED_MOTORS 8
#endif
#define MAX_SUPPORTED_SERVOS 8
#endif

#if defined(USE_DSHOT_BITBANG)
#if !defined(DSHOT_BITBANG_DEFAULT)
#define DSHOT_BITBANG_DEFAULT DSHOT_BITBANG_AUTO
#endif

#if !defined(DSHOT_BITBANGED_TIMER_DEFAULT)
#define DSHOT_BITBANGED_TIMER_DEFAULT DSHOT_BITBANGED_TIMER_AUTO
#endif
#endif // USE_DSHOT_BITBANG
