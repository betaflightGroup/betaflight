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

#if defined(USE_BARO) && (defined(USE_BARO_MS5611) || defined(USE_BARO_SPI_MS5611))

#include "build/build_config.h"

#include "barometer.h"
#include "barometer_ms5611.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

// 10 MHz max SPI frequency
#define MS5611_MAX_SPI_CLK_HZ 10000000

// MS5611, Standard address 0x77
#define MS5611_I2C_ADDR                 0x77

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

STATIC_UNIT_TESTED uint32_t ms5611_ut;  // static result of temperature measurement
STATIC_UNIT_TESTED uint32_t ms5611_up;  // static result of pressure measurement
STATIC_UNIT_TESTED uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t ms5611_osr = CMD_ADC_4096;
#define MS5611_DATA_FRAME_SIZE 3
static DMA_DATA_ZERO_INIT uint8_t sensor_data[MS5611_DATA_FRAME_SIZE];

void ms5611BusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_MS5611
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(MS5611_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

void ms5611BusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_MS5611
    if (dev->bus->busType == BUS_TYPE_SPI) {
        spiPreinitByIO(dev->busType_u.spi.csnPin);
    }
#else
    UNUSED(dev);
#endif
}

static void ms5611Reset(const extDevice_t *dev)
{
    busRawWriteRegister(dev, CMD_RESET, 1);

    delayMicroseconds(2800);
}

static uint16_t ms5611Prom(const extDevice_t *dev, int8_t coef_num)
{
    uint8_t rxbuf[2] = { 0, 0 };

    busRawReadRegisterBuffer(dev, CMD_PROM_RD + coef_num * 2, rxbuf, 2); // send PROM READ command

    return rxbuf[0] << 8 | rxbuf[1];
}

STATIC_UNIT_TESTED int8_t ms5611CRC(uint16_t *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    bool blankEeprom = true;

    for (i = 0; i < 16; i++) {
        if (prom[i >> 1]) {
            blankEeprom = false;
        }
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (!blankEeprom && crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}

static void ms5611ReadAdc(const extDevice_t *dev)
{
    busRawReadRegisterBufferStart(dev, CMD_ADC_READ, sensor_data, MS5611_DATA_FRAME_SIZE); // read ADC
}

static void ms5611StartUT(baroDev_t *baro)
{
    busRawWriteRegisterStart(&baro->dev, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static bool ms5611ReadUT(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    ms5611ReadAdc(&baro->dev);

    return true;
}

static bool ms5611GetUT(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    ms5611_ut = sensor_data[0] << 16 | sensor_data[1] << 8 | sensor_data[2];

    return true;
}

static void ms5611StartUP(baroDev_t *baro)
{
    busRawWriteRegisterStart(&baro->dev, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static bool ms5611ReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    ms5611ReadAdc(&baro->dev);

    return true;
}

static bool ms5611GetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    ms5611_up = sensor_data[0] << 16 | sensor_data[1] << 8 | sensor_data[2];

    return true;
}

STATIC_UNIT_TESTED void ms5611Calculate(int32_t *pressure, int32_t *temperature)
{
    uint32_t press;
    int64_t temp;
    int64_t delt;
    int64_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
    int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    temp -= ((dT * dT) >> 31);
    }
    press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;


    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}

bool ms5611Detect(baroDev_t *baro)
{
    uint8_t sig;
    int i;
    bool defaultAddressApplied = false;

    delay(10); // No idea how long the chip takes to power-up, but let's make it 10ms

    extDevice_t *dev = &baro->dev;

    ms5611BusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for MS5611
        dev->busType_u.i2c.address = MS5611_I2C_ADDR;
        defaultAddressApplied = true;
    }

    if (!busRawReadRegisterBuffer(dev, CMD_PROM_RD, &sig, 1) || sig == 0xFF) {
        goto fail;
    }

    ms5611Reset(dev);

    // read all coefficients
    for (i = 0; i < PROM_NB; i++)
        ms5611_c[i] = ms5611Prom(dev, i);

    // check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
    if (ms5611CRC(ms5611_c) != 0) {
        goto fail;
    }

    busDeviceRegister(dev);

    // TODO prom + CRC
    baro->ut_delay = 10000;
    baro->up_delay = 10000;
    baro->start_ut = ms5611StartUT;
    baro->read_ut = ms5611ReadUT;
    baro->get_ut = ms5611GetUT;
    baro->start_up = ms5611StartUP;
    baro->read_up = ms5611ReadUP;
    baro->get_up = ms5611GetUP;
    baro->calculate = ms5611Calculate;

    return true;

fail:;
    ms5611BusDeinit(dev);

    if (defaultAddressApplied) {
        dev->busType_u.i2c.address = 0;
    }

    return false;
}
#endif
