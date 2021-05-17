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

/*
 * Author: Giles Burgess (giles@multiflite.co.uk)
 *
 * This source code is provided as is and can be used/modified so long
 * as this header is maintained with the file at all times.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_VTX_RTC6705)

#include "common/maths.h"

#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/vtx_rtc6705_soft_spi.h"

#include "vtx_rtc6705.h"

// 1 MHz max SPI frequency - datasheet says 10MHz, but this is lower for some reason
#define RTC6705_MAX_SPI_CLK_HZ 1000000

#define RTC6705_SET_HEAD 0x3210 //fosc=8mhz r=400
#define RTC6705_SET_R  400     //Reference clock
#define RTC6705_SET_FDIV 1024  //128*(fosc/1000000)
#define RTC6705_SET_NDIV 16    //Remainder divider to get 'A' part of equation
#define RTC6705_SET_WRITE 0x11 //10001b to write to register
#define RTC6705_SET_DIVMULT 1000000 //Division value (to fit into a uint32_t) (Hz to MHz)

#ifdef RTC6705_POWER_PIN
static IO_t vtxPowerPin     = IO_NONE;
#endif

static extDevice_t *dev = NULL;

#define DISABLE_RTC6705()   IOHi(dev->busType_u.spi.csnPin)
#define ENABLE_RTC6705()    IOLo(dev->busType_u.spi.csnPin)

#define DP_5G_MASK          0x7000 // b111000000000000
#define PA5G_BS_MASK        0x0E00 // b000111000000000
#define PA5G_PW_MASK        0x0180 // b000000110000000
#define PD_Q5G_MASK         0x0040 // b000000001000000
#define QI_5G_MASK          0x0038 // b000000000111000
#define PA_BS_MASK          0x0007 // b000000000000111

#define PA_CONTROL_DEFAULT  0x4FBD

#define RTC6705_RW_CONTROL_BIT      (1 << 4)
#define RTC6705_ADDRESS             (0x07)

/**
 * Reverse a uint32_t (LSB to MSB)
 * This is easier for when generating the frequency to then
 * reverse the bits afterwards
 */
static uint32_t reverse32(uint32_t in)
{
    uint32_t out = 0;

    for (uint8_t i = 0 ; i < 32 ; i++) {
        out |= ((in>>i) & 1)<<(31-i);
    }

    return out;
}

/**
 * Start chip if available
 */
bool rtc6705IOInit(const vtxIOConfig_t *vtxIOConfig)
{
    static extDevice_t devInstance;

    IO_t csnPin = IOGetByTag(vtxIOConfig->csTag);
    if (!csnPin) {
        return false;
    }

    vtxPowerPin = IOGetByTag(vtxIOConfig->powerTag);
    if (vtxPowerPin) {
        IOInit(vtxPowerPin, OWNER_VTX_POWER, 0);

        IOHi(vtxPowerPin);

        IOConfigGPIO(vtxPowerPin, IOCFG_OUT_PP);
    }

    if (vtxIOConfig->csTag && spiSetBusInstance(dev, vtxIOConfig->spiDevice, OWNER_VTX_CS)) {
        devInstance.busType_u.spi.csnPin = csnPin;
        IOInit(devInstance.busType_u.spi.csnPin, OWNER_VTX_CS, 0);

        DISABLE_RTC6705();
        // GPIO bit is enabled so here so the output is not pulled low when the GPIO is set in output mode.
        // Note: It's critical to ensure that incorrect signals are not sent to the VTX.
        IOConfigGPIO(devInstance.busType_u.spi.csnPin, IOCFG_OUT_PP);

        return true;
#if defined(USE_VTX_RTC6705_SOFTSPI)
    } else {
        return rtc6705SoftSpiIOInit(vtxIOConfig, csnPin);
#endif
    }

    return false;;
}

/**
 * Transfer a 25bit packet to RTC6705
 * This will just send it as a 32bit packet LSB meaning
 * extra 0's get truncated on RTC6705 end
 */
static void rtc6705Transfer(uint32_t command)
{
    // Perform bitwise reverse of the command.
    command = reverse32(command);

    spiReadWriteBuf(dev, (uint8_t *)&command, NULL, sizeof (command));

    delayMicroseconds(2);
}

 /**
 * Set a frequency in Mhz
 * Formula derived from datasheet
 */
void rtc6705SetFrequency(uint16_t frequency)
{
#if defined(USE_VTX_RTC6705_SOFTSPI)
    if (!dev) {
        rtc6705SoftSpiSetFrequency(frequency);

        return;
    }
#endif

    frequency = constrain(frequency, VTX_RTC6705_FREQ_MIN, VTX_RTC6705_FREQ_MAX);

    const uint32_t val_a = ((((uint64_t)frequency*(uint64_t)RTC6705_SET_DIVMULT*(uint64_t)RTC6705_SET_R)/(uint64_t)RTC6705_SET_DIVMULT) % RTC6705_SET_FDIV) / RTC6705_SET_NDIV; //Casts required to make sure correct math (large numbers)
    const uint32_t val_n = (((uint64_t)frequency*(uint64_t)RTC6705_SET_DIVMULT*(uint64_t)RTC6705_SET_R)/(uint64_t)RTC6705_SET_DIVMULT) / RTC6705_SET_FDIV; //Casts required to make sure correct math (large numbers)

    uint32_t val_hex = RTC6705_SET_WRITE;
    val_hex |= (val_a << 5);
    val_hex |= (val_n << 12);

    spiSetClkDivisor(dev, spiCalculateDivider(RTC6705_MAX_SPI_CLK_HZ));

    rtc6705Transfer(RTC6705_SET_HEAD);
    delayMicroseconds(10);
    rtc6705Transfer(val_hex);
}

void rtc6705SetRFPower(uint8_t rf_power)
{
    rf_power = constrain(rf_power, 1, 2);
#if defined(USE_VTX_RTC6705_SOFTSPI)
    if (!dev) {
        rtc6705SoftSpiSetRFPower(rf_power);

        return;
    }
#endif

    uint32_t val_hex = RTC6705_RW_CONTROL_BIT; // write
    val_hex |= RTC6705_ADDRESS; // address
    const uint32_t data = rf_power > 1 ? PA_CONTROL_DEFAULT : (PA_CONTROL_DEFAULT | PD_Q5G_MASK) & (~(PA5G_PW_MASK | PA5G_BS_MASK));
    val_hex |= data << 5; // 4 address bits and 1 rw bit.

    spiSetClkDivisor(dev, spiCalculateDivider(RTC6705_MAX_SPI_CLK_HZ));

    rtc6705Transfer(val_hex);
}

void rtc6705Disable(void)
{
    if (vtxPowerPin) {
        IOHi(vtxPowerPin);
    }
}

void rtc6705Enable(void)
{
    if (vtxPowerPin) {
        IOLo(vtxPowerPin);
    }
}
#endif
