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

#include "platform.h"

#include "resource.h"

const char * const ownerNames[OWNER_TOTAL_COUNT] = {
    "FREE",
    "PWM",
    "PPM",
    "MOTOR",
    "SERVO",
    "LED",
    "ADC",
    "ADC_BATT",
    "ADC_CURR",
    "ADC_EXT",
    "ADC_RSSI",
    "SERIAL_TX",
    "SERIAL_RX",
    "DEBUG",
    "TIMER",
    "SONAR_TRIGGER",
    "SONAR_ECHO",
    "SYSTEM",
    "SPI_SCK",
    "SPI_MISO",
    "SPI_MOSI",
    "I2C_SCL",
    "I2C_SDA",
    "SDCARD",
    "SDIO_CK",
    "SDIO_CMD",
    "SDIO_D0",
    "SDIO_D1",
    "SDIO_D2",
    "SDIO_D3",
    "SDCARD_CS",
    "SDCARD_DETECT",
    "FLASH_CS",
    "BARO_CS",
    "GYRO_CS",
    "OSD_CS",
    "RX_SPI_CS",
    "SPI_CS",
    "GYRO_EXTI",
    "BARO_EOC",
    "COMPASS_EXTI",
    "USB",
    "USB_DETECT",
    "BEEPER",
    "OSD",
    "RX_BIND",
    "INVERTER",
    "LED_STRIP",
    "TRANSPONDER",
    "VTX_POWER",
    "VTX_CS",
    "VTX_DATA",
    "VTX_CLK",
    "COMPASS_CS",
    "RX_BIND_PLUG",
    "ESCSERIAL",
    "CAMERA_CONTROL",
    "TIMUP",
    "RANGEFINDER",
    "RX_SPI",
    "PINIO",
    "USB_MSC_PIN",
    "MCO",
    "RX_SPI_BIND",
    "RX_SPI_LED",
    "PREINIT",
    "RX_SPI_EXTI",
    "RX_SPI_CC2500_TX_EN",
    "RX_SPI_CC2500_LNA_EN",
    "RX_SPI_CC2500_ANT_SEL",
    "QSPI_CLK",
    "QSPI_BK1IO0",
    "QSPI_BK1IO1",
    "QSPI_BK1IO2",
    "QSPI_BK1IO3",
    "QSPI_BK1CS",
    "QSPI_BK2IO0",
    "QSPI_BK2IO1",
    "QSPI_BK2IO2",
    "QSPI_BK2IO3",
    "QSPI_BK2CS",
    "BARO_XCLR",
    "PULLUP",
    "PULLDOWN",
    "DSHOT_BITBANG",
    "SWD",
    "RX_SPI_EXPRESSLRS_RESET",
    "RX_SPI_EXPRESSLRS_BUSY",
};
