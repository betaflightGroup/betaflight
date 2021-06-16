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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/accgyro/accgyro_mpu.h"

#include "drivers/compass/compass.h"
#include "drivers/compass/compass_ak8963.h"
#include "drivers/compass/compass_mpu925x_ak8963.h"

#if defined(USE_MAG_MPU925X_AK8963)

#define MPU925X_I2C_ADDRESS             0x68
#define AK8963_MAG_I2C_ADDRESS          0x0C
#define MPU9250_BIT_RESET               0x80

static bool mpu925xDeviceDetect(const extDevice_t * dev)
{
    busWriteRegister(dev, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delay(150);
    switch (busReadRegister(dev, MPU_RA_WHO_AM_I)) {
        case MPU9250_WHO_AM_I_CONST:
        case MPU9255_WHO_AM_I_CONST:
            return true;
        default:
            return false;
    }
}

bool mpu925Xak8963CompassDetect(magDev_t * mag)
{
    extDevice_t *dev = &mag->dev;
    busDeviceRegister(dev);
    if (!mpu925xDeviceDetect(dev->bus->busType_u.mpuSlave.master)) {
        return false;
    }
    // set bypass mode on mpu9250
    busWriteRegister(dev->bus->busType_u.mpuSlave.master, MPU_RA_INT_PIN_CFG, 0x02);
    delay(150);
    // now we have ak8963 alike on the bus
    dev->busType_u.i2c.address = AK8963_MAG_I2C_ADDRESS;
    busDeviceRegister(dev);
    if(!ak8963Detect(mag)) {
        // if ak8963 is not detected, reset the MPU to disable bypass mode
        dev->busType_u.i2c.address = MPU925X_I2C_ADDRESS;
        busWriteRegister(dev, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
        return false;
    } else {
        return true;
    }
}

#endif
