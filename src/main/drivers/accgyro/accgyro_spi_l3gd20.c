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

#ifdef USE_GYRO_L3GD20

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "drivers/accgyro/accgyro.h"

#include "accgyro_spi_l3gd20.h"

// 10 MHz max SPI frequency
#define L3GD20_MAX_SPI_CLK_HZ 10000000

#define READ_CMD               ((uint8_t)0x80)
#define MULTIPLEBYTE_CMD       ((uint8_t)0x40)
#define DUMMY_BYTE             ((uint8_t)0x00)

#define CTRL_REG1_ADDR         0x20
#define CTRL_REG4_ADDR         0x23
#define CTRL_REG5_ADDR         0x24
#define OUT_TEMP_ADDR          0x26
#define OUT_X_L_ADDR           0x28

#define MODE_ACTIVE                   ((uint8_t)0x08)

#define OUTPUT_DATARATE_1             ((uint8_t)0x00)
#define OUTPUT_DATARATE_2             ((uint8_t)0x40)
#define OUTPUT_DATARATE_3             ((uint8_t)0x80)
#define OUTPUT_DATARATE_4             ((uint8_t)0xC0)

#define AXES_ENABLE                   ((uint8_t)0x07)

#define BANDWIDTH_1                   ((uint8_t)0x00)
#define BANDWIDTH_2                   ((uint8_t)0x10)
#define BANDWIDTH_3                   ((uint8_t)0x20)
#define BANDWIDTH_4                   ((uint8_t)0x30)

#define FULLSCALE_250                 ((uint8_t)0x00)
#define FULLSCALE_500                 ((uint8_t)0x10)
#define FULLSCALE_2000                ((uint8_t)0x20)

#define BLOCK_DATA_UPDATE_CONTINUOUS  ((uint8_t)0x00)

#define BLE_MSB                       ((uint8_t)0x40)

#define BOOT                          ((uint8_t)0x80)

#if defined(USE_MPU_DATA_READY_SIGNAL)
static void l3gd20ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}

static void l3gd20IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, l3gd20ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO, true);
}
#endif

void l3gd20GyroInit(gyroDev_t *gyro)
{
    spiSetClkDivisor(dev, spiCalculateDivider(L3GD20_MAX_SPI_CLK_HZ));

    spiWriteReg(&gyro->bus, CTRL_REG5_ADDR, BOOT);

    delayMicroseconds(100);

    spiWriteReg(&gyro->bus, CTRL_REG1_ADDR, MODE_ACTIVE | OUTPUT_DATARATE_3 | AXES_ENABLE | BANDWIDTH_3);
    //spiWriteReg(&gyro->bus, CTRL_REG1_ADDR. MODE_ACTIVE | OUTPUT_DATARATE_3 | AXES_ENABLE | BANDWIDTH_4);

    delayMicroseconds(1);

    spiWriteReg(&gyro->bus, CTRL_REG4_ADDR, BLOCK_DATA_UPDATE_CONTINUOUS | BLE_MSB | FULLSCALE_2000);

    delay(100);

#if defined(USE_MPU_DATA_READY_SIGNAL)
    l3gd20IntExtiInit(gyro);
#endif

}

static bool l3gd20GyroRead(gyroDev_t *gyro)
{
    uint8_t buf[6];

    const bool ack = spiReadRegMskBufRB(&gyro->bus, OUT_X_L_ADDR | READ_CMD | MULTIPLEBYTE_CMD,buf, sizeof(buf));
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyro->gyroADCRaw[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyro->gyroADCRaw[2] = (int16_t)((buf[4] << 8) | buf[5]);

    return true;
}

// Page 9 in datasheet, So - Sensitivity, Full Scale = 2000, 70 mdps/digit
#define L3GD20_GYRO_SCALE_FACTOR  0.07f

uint8_t l3gd20Detect(const busDevice_t *bus)
{
    UNUSED(bus);

    return L3GD20_SPI; // blindly assume it's present, for now.
}
    
bool l3gd20GyroDetect(gyroDev_t *gyro)
{
    gyro->initFn = l3gd20GyroInit;
    gyro->readFn = l3gd20GyroRead;

    gyro->scale = L3GD20_GYRO_SCALE_FACTOR;

    return true;  // blindly assume it's present, for now.
}

#endif // USE_GYRO_L3GD20
