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

#define TARGET_BOARD_IDENTIFIER "HK41"
#define USBD_PRODUCT_STRING  "HAKRCF411"


#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#define USE_I2C_PULLUP

#define USE_BARO
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_QMP6988
#define BARO_I2C_INSTANCE       (I2CDEV_1)
#define DEFAULT_BARO_QMP6988

// XXX CAMERA_CONTROL_PIN is deprecated.
// XXX Target maintainer must create a valid timerHardware[] array entry for PB5 with TIM_USE_CAMERA_CONTROL
//#define CAMERA_CONTROL_PIN    PB5

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PB2
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_ACC

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_GYRO_SPI_ICM20689
#define GYRO_1_ALIGN            CW270_DEG

#define USE_ACC_SPI_ICM20689

#define USE_GYRO_SPI_MPU6000

#define USE_ACC_SPI_MPU6000


#define LED0_PIN                PC13

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS


#define USE_GPS
#define USE_GPS_UBLOX
#define USE_GPS_NMEA

#define USE_SPI_DEVICE_2
#define SPI3_SCK_PIN            PB13
#define SPI3_MISO_PIN           PB14 
#define SPI3_MOSI_PIN           PB15

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12 

#define USE_ADC
#define VBAT_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PA1

 
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

#define USE_BEEPER
#define BEEPER_PIN              PC14
#define BEEPER_INVERTED

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN PA3

#define INVERTER_PIN_UART1      PB4 

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff


#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4))
