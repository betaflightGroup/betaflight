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

#include <math.h>

#include "platform.h"

#if defined(USE_MAG_HMC5883) || defined(USE_MAG_SPI_HMC5883)

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"

#include "compass_hmc5883l.h"

//#define DEBUG_MAG_DATA_READY_INTERRUPT

// 10 MHz max SPI frequency
#define HMC5883_MAX_SPI_CLK_HZ 10000000

// HMC5883L, default address 0x1E
// NAZE Target connections
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

/* CTRL_REGA: Control Register A
 * Read Write
 * Default value: 0x10
 * 7:5  0   These bits must be cleared for correct operation.
 * 4:2 DO2-DO0: Data Output Rate Bits
 *             DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *            ------------------------------------------------------
 *              0  |  0   |  0   |            0.75
 *              0  |  0   |  1   |            1.5
 *              0  |  1   |  0   |            3
 *              0  |  1   |  1   |            7.5
 *              1  |  0   |  0   |           15 (default)
 *              1  |  0   |  1   |           30
 *              1  |  1   |  0   |           75
 *              1  |  1   |  1   |           Not Used
 * 1:0 MS1-MS0: Measurement Configuration Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0  |  Normal
 *              0  |  1  |  Positive Bias
 *              1  |  0  |  Negative Bias
 *              1  |  1  |  Not Used
 *
 * CTRL_REGB: Control RegisterB
 * Read Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits.
 *             GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
 *                 |      |      |  Range[Ga]    | [LSB/mGa]  |
 *            ------------------------------------------------------
 *              0  |  0   |  0   |  �0.88Ga      |   1370     | 0xF800?0x07FF (-2048:2047)
 *              0  |  0   |  1   |  �1.3Ga (def) |   1090     | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  0   |  �1.9Ga       |   820      | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  1   |  �2.5Ga       |   660      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  0   |  �4.0Ga       |   440      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  1   |  �4.7Ga       |   390      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  0   |  �5.6Ga       |   330      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  1   |  �8.1Ga       |   230      | 0xF800?0x07FF (-2048:2047)
 *                               |Not recommended|
 *
 * 4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
 *
 * _MODE_REG: Mode Register
 * Read Write
 * Default value: 0x02
 * 7:2  0   These bits must be cleared for correct operation.
 * 1:0 MD1-MD0: Mode Select Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0  |  Continuous-Conversion Mode.
 *              0  |  1  |  Single-Conversion Mode
 *              1  |  0  |  Negative Bias
 *              1  |  1  |  Sleep Mode
 */

#define HMC5883_MAG_I2C_ADDRESS     0x1E
#define HMC5883_DEVICE_ID           0x48

#define HMC58X3_REG_CONFA           0x00
#define HMC58X3_REG_CONFB           0x01
#define HMC58X3_REG_MODE            0x02
#define HMC58X3_REG_DATA            0x03
#define HMC58X3_REG_IDA             0x0A

#define HMC_CONFA_NORMAL            0x00
#define HMC_CONFA_POS_BIAS          0x01
#define HMC_CONFA_NEG_BIAS          0x02
#define HMC_CONFA_DOR_15HZ          0X10
#define HMC_CONFA_8_SAMLES          0X60
#define HMC_CONFB_GAIN_2_5GA        0X60
#define HMC_CONFB_GAIN_1_3GA        0X20
#define HMC_MODE_CONTINOUS          0X00
#define HMC_MODE_SINGLE             0X01

#define HMC58X3_X_SELF_TEST_GAUSS   (+1.16f)            // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS   (+1.16f)            // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS   (+1.08f)            // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT         (243.0f / 390.0f)   // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT        (575.0f / 390.0f)   // High limit when gain is 5.

#ifdef USE_MAG_DATA_READY_SIGNAL

static void hmc5883_extiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);
#ifdef DEBUG_MAG_DATA_READY_INTERRUPT
    // Measure the delta between calls to the interrupt handler
    // currently should be around 65/66 milli seconds / 15hz output rate
    static uint32_t lastCalledAt = 0;
    static int32_t callDelta = 0;

    uint32_t now = millis();
    callDelta = now - lastCalledAt;

    //UNUSED(callDelta);
    debug[0] = callDelta;

    lastCalledAt = now;
#endif
}
#endif

static void hmc5883lConfigureDataReadyInterruptHandling(magDev_t* mag)
{
#ifdef USE_MAG_DATA_READY_SIGNAL
    if (mag->magIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t magIntIO = IOGetByTag(mag->magIntExtiTag);

#ifdef ENSURE_MAG_DATA_READY_IS_HIGH
    uint8_t status = IORead(magIntIO);
    if (!status) {
        return;
    }
#endif

    IOInit(magIntIO, OWNER_COMPASS_EXTI, 0);
    EXTIHandlerInit(&mag->exti, hmc5883_extiHandler);
    EXTIConfig(magIntIO, &mag->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(magIntIO, true);
    EXTIEnable(magIntIO, true);
#else
    UNUSED(mag);
#endif
}

#ifdef USE_MAG_SPI_HMC5883
static void hmc5883SpiInit(const extDevice_t *dev)
{
    busDeviceRegister(dev);

    IOHi(dev->busType_u.spi.csnPin); // Disable

    IOInit(dev->busType_u.spi.csnPin, OWNER_COMPASS_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
    spiSetClkDivisor(dev, spiCalculateDivider(HMC5883_MAX_SPI_CLK_HZ));
}
#endif

static bool hmc5883lRead(magDev_t *mag, int16_t *magData)
{
    uint8_t buf[6];

    extDevice_t *dev = &mag->dev;

    bool ack = busReadRegisterBuffer(dev, HMC58X3_REG_DATA, buf, 6);

    if (!ack) {
        return false;
    }

    magData[X] = (int16_t)(buf[0] << 8 | buf[1]);
    magData[Z] = (int16_t)(buf[2] << 8 | buf[3]);
    magData[Y] = (int16_t)(buf[4] << 8 | buf[5]);

    return true;
}

static bool hmc5883lInit(magDev_t *mag)
{

    extDevice_t *dev = &mag->dev;

    // leave test mode
    busWriteRegister(dev, HMC58X3_REG_CONFA, HMC_CONFA_8_SAMLES | HMC_CONFA_DOR_15HZ | HMC_CONFA_NORMAL);    // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    busWriteRegister(dev, HMC58X3_REG_CONFB, HMC_CONFB_GAIN_1_3GA);                                          // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    busWriteRegister(dev, HMC58X3_REG_MODE, HMC_MODE_CONTINOUS);                                             // Mode register             -- 000000 00    continuous Conversion Mode

    delay(100);

    hmc5883lConfigureDataReadyInterruptHandling(mag);
    return true;
}

bool hmc5883lDetect(magDev_t* mag)
{
    extDevice_t *dev = &mag->dev;

    uint8_t sig = 0;

#ifdef USE_MAG_SPI_HMC5883
    if (dev->bus->busType == BUS_TYPE_SPI) {
        hmc5883SpiInit(dev);
    }
#endif

#ifdef USE_MAG_HMC5883
    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = HMC5883_MAG_I2C_ADDRESS;
    }
#endif

    bool ack = busReadRegisterBuffer(&mag->dev, HMC58X3_REG_IDA, &sig, 1);

    if (!ack || sig != HMC5883_DEVICE_ID) {
        return false;
    }

    mag->init = hmc5883lInit;
    mag->read = hmc5883lRead;

    return true;
}
#endif
