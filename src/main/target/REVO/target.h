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

#define USE_LATE_TASK_STATISTICS

#if defined(AIRBOTF4)
#define TARGET_BOARD_IDENTIFIER "AIR4"
#define USBD_PRODUCT_STRING     "AirbotF4"

#elif defined(AIRBOTF4SD)
#define TARGET_BOARD_IDENTIFIER "A4SD"
#define USBD_PRODUCT_STRING     "AirbotF4SD"

#elif defined(SOULF4)
#define TARGET_BOARD_IDENTIFIER "SOUL"
#define USBD_PRODUCT_STRING     "DemonSoulF4"

#elif defined(PODIUMF4)
#define TARGET_BOARD_IDENTIFIER "PODI"
#define USBD_PRODUCT_STRING     "PodiumF4"

#elif defined(ELINF405)
#define TARGET_BOARD_IDENTIFIER "ELIN"
#define USBD_PRODUCT_STRING     "ElinF405"

#else
#define TARGET_BOARD_IDENTIFIER "REVO"
#define USBD_PRODUCT_STRING     "Revolution"

#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000"
#endif

#endif

#define LED0_PIN                PB5
#if defined(PODIUMF4)
#define LED1_PIN                PB4
#define LED2_PIN                PB6
#endif

// Disable LED1, conflicts with AirbotF4/Flip32F4 beeper
#if defined(AIRBOTF4) || defined(AIRBOTF4SD)
#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED
#elif defined(SOULF4)
#define USE_BEEPER
#define BEEPER_PIN              PB6
#define BEEPER_INVERTED
#elif defined(ELINF405)
#define USE_BEEPER
#define BEEPER_PIN              PB4
#else
#define LED1_PIN                PB4
// Leave beeper here but with none as io - so disabled unless mapped.
#define USE_BEEPER
#define BEEPER_PIN              NONE
#endif


// PC0 used as inverter select GPIO
#ifdef AIRBOTF4SD
#define INVERTER_PIN_UART6      PD2
#else
#define INVERTER_PIN_UART1      PC0
#endif

#define USE_GYRO
#define USE_ACC

#ifdef AIRBOTF4SD
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500

#define GYRO_1_CS_PIN           PB13
#define GYRO_2_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_2_SPI_INSTANCE     SPI1

#define GYRO_1_ALIGN            CW270_DEG
#define GYRO_2_ALIGN            CW270_DEG

#elif defined(SOULF4)

#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN            CW180_DEG

#define USE_ACC_SPI_MPU6000

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#elif defined(PODIUMF4)

#define USE_GYRO_SPI_MPU6500
#define GYRO_1_ALIGN            CW0_DEG

#define USE_ACC_SPI_MPU6500

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#elif defined(ELINF405)

#define USE_GYRO_SPI_MPU6500
#define GYRO_1_ALIGN            CW0_DEG
#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_ACC_SPI_MPU6500


#else

#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_ALIGN            CW270_DEG

#endif

// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define USE_MPU_DATA_READY_SIGNAL

#if defined(ELINF405)

#define USE_OSD
#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_SOFTSERIAL)
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PC8

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_AK8963
#define USE_MAG_AK8975

#else

#define GYRO_2_EXTI_PIN         NONE

// Configure MAG and BARO unconditionally.
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW90_DEG

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

#endif

#if defined(AIRBOTF4) || defined(AIRBOTF4SD)
#define USE_BARO_SPI_BMP280
#define BARO_SPI_INSTANCE       SPI1
#define BARO_CS_PIN             PC13
#endif

#if defined(AIRBOTF4SD)
// SDCARD support for AIRBOTF4SD
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PC0
#define SDCARD_SPI_INSTANCE     SPI3
#define SDCARD_SPI_CS_PIN       SPI3_NSS_PIN
#define SPI3_TX_DMA_OPT                     0     // DMA 1 Stream 5 Channel 0

#else

#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16

#endif // AIRBOTF4SD


#define USE_VCP
#if defined(PODIUMF4)
#define USE_USB_DETECT
#define USB_DETECT_PIN          PA8
#else
#define USE_USB_DETECT
#define USB_DETECT_PIN          PC5
#endif

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#if defined(REVO) || defined(ELINF405)
#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0
#endif // REVO

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#if defined(ELINF405)
#define PINIO1_PIN              PC13
#define PINIO2_PIN              PC14

#define DEFAULT_MIXER           MIXER_QUADX
#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON
#define USE_TARGET_CONFIG
#define SOFTSERIAL1_TX_PIN      PC9
#define SOFTSERIAL2_RX_PIN      PA8

#else

#define PINIO1_PIN              PC8 // DTR pin

#endif

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#if defined(REVO) || defined(ELINF405)
#define SERIAL_PORT_COUNT       7 //VCP, USART1, USART3, UART4,  USART6, SOFTSERIAL x 2
#else
#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART3, USART6, SOFTSERIAL x 2
#endif

#define USE_ESCSERIAL
#if defined(ELINF405)
#define ESCSERIAL_TIMER_TX_PIN  PB6
#else
#define ESCSERIAL_TIMER_TX_PIN  PB14  // (HARDARE=0,PPM)
#endif

#define USE_SPI

#if defined(ELINF405)
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#else

#define USE_SPI_DEVICE_1

#endif

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#if defined(AIRBOTF4) || defined(AIRBOTF4SD)
// On AIRBOTF4 and AIRBOTF4SD, I2C2 and I2C3 are configurable
#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE // PB10, shared with UART3TX
#define I2C2_SDA                NONE // PB11, shared with UART3RX
#define USE_I2C_DEVICE_3
#define I2C3_SCL                NONE // PA8, PWM6
#define I2C3_SDA                NONE // PC9, CH6
#define I2C_DEVICE              (I2CDEV_2)
#else
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#endif

#define USE_ADC
#if !defined(PODIUMF4)
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2
#else
#define VBAT_ADC_PIN            PC3
#endif

#if defined(ELINF405)
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#endif

#if defined(AIRBOTF4SD)
#define RSSI_ADC_PIN            PA0
#endif

#define USE_TRANSPONDER

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#if defined(PODIUMF4)
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
#define DEFAULT_FEATURES        FEATURE_TELEMETRY
#elif defined(ELINF405)
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1
#endif

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#if defined(AIRBOTF4) || defined(AIRBOTF4SD)
#define USABLE_TIMER_CHANNEL_COUNT 13
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(12) )
#elif defined(ELINF405)
#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(11) )
#else
#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(12) )
#endif
