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

#ifdef FURYF4OSD
#define TARGET_BOARD_IDENTIFIER "FY4O"
#define USBD_PRODUCT_STRING     "FuryF4OSD"
#else
#define TARGET_BOARD_IDENTIFIER "FYF4"
#define USBD_PRODUCT_STRING     "FuryF4"
#endif

#define LED0_PIN                PB5
#define LED1_PIN                PB4

#define USE_BEEPER
#define BEEPER_PIN              PA8
#define BEEPER_INVERTED

#define INVERTER_PIN_UART1      PC0 // PC0 used as inverter select GPIO

// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_ICM20689
#define GYRO_1_ALIGN            CW180_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_ICM20689

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define MS5611_I2C_INSTANCE     I2CDEV_1

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL

#ifdef FURYF4OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_FEATURES        FEATURE_OSD

#else

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PD2
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   PB12
#define SPI2_TX_DMA_OPT                     0     // DMA 1 Stream 4 Channel 0
#define SPI2_TX_DMA_OPT                     0     // DMA 1 Stream 4 Channel 0
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#endif

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3

#define USE_VCP
#define USB_DETECT_PIN          PC5
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART3, USART6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PC9  // (HARDARE=0,PPM)

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define USE_I2C_PULLUP
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define VBAT_ADC_PIN            PC1
#define RSSI_ADC_PIN            PC2
#define CURRENT_METER_ADC_PIN   PC3

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART1
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(11) )
