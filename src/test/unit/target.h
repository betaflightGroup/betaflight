/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define SCHEDULER_DELAY_LIMIT 1
#define TASK_GYROPID_DESIRED_PERIOD 100

#define DMA_DATA
#define DMA_DATA_ZERO_INIT

#define USE_ACC
#define USE_CMS
#define CMS_MAX_DEVICE 4
#define USE_FAKE_GYRO
#define USE_BEEPER
#define USE_BLACKBOX
#define USE_MAG
#define USE_BARO
#define USE_GPS
#define USE_DASHBOARD
#define USE_SERIAL_RX
#define USE_RX_MSP
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_JETIEXBUS
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_TELEMETRY
#define USE_TELEMETRY_CRSF
#define USE_TELEMETRY_FRSKY_HUB
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_JETIEXBUS
#define USE_TELEMETRY_LTM
#define USE_TELEMETRY_MAVLINK
#define USE_TELEMETRY_SMARTPORT
#define USE_LED_STRIP
#define USE_LED_STRIP_STATUS_MODE
#define USE_SERVOS
#define USE_TRANSPONDER
#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 8

#define DEFAULT_AUX_CHANNEL_COUNT       MAX_AUX_CHANNEL_COUNT
#define MAX_SIMULTANEOUS_ADJUSTMENT_COUNT 6  // needed for unittest

#define TARGET_BOARD_IDENTIFIER "TEST"

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SERIAL

#define LED_STRIP_TIMER 1
#define SOFTSERIAL_1_TIMER 2
#define SOFTSERIAL_2_TIMER 3

#define USABLE_TIMER_CHANNEL_COUNT 0

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff

