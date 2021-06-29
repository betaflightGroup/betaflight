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

#if defined(USE_MAG_LIS3MDL)

#include "compass.h"
#include "drivers/time.h"
#include "common/axis.h"

#define LIS3MDL_MAG_I2C_ADDRESS     0x1E
#define LIS3MDL_DEVICE_ID           0x3D

#define LIS3MDL_REG_WHO_AM_I        0x0F

#define LIS3MDL_REG_CTRL_REG1       0x20
#define LIS3MDL_REG_CTRL_REG2       0x21
#define LIS3MDL_REG_CTRL_REG3       0x22
#define LIS3MDL_REG_CTRL_REG4       0x23
#define LIS3MDL_REG_CTRL_REG5       0x24

#define LIS3MDL_REG_STATUS_REG      0x27

#define LIS3MDL_REG_OUT_X_L         0x28
#define LIS3MDL_REG_OUT_X_H         0x29
#define LIS3MDL_REG_OUT_Y_L         0x2A
#define LIS3MDL_REG_OUT_Y_H         0x2B
#define LIS3MDL_REG_OUT_Z_L         0x2C
#define LIS3MDL_REG_OUT_Z_H         0x2D

#define LIS3MDL_TEMP_OUT_L          0x2E
#define LIS3MDL_TEMP_OUT_H          0x2F

#define LIS3MDL_INT_CFG             0x30
#define LIS3MDL_INT_SRC             0x31
#define LIS3MDL_THS_L               0x32
#define LIS3MDL_THS_H               0x33

// CTRL_REG1
#define LIS3MDL_TEMP_EN             0x80  // Default 0
#define LIS3MDL_OM_LOW_POWER        0x00  // Default
#define LIS3MDL_OM_MED_PROF         0x20
#define LIS3MDL_OM_HI_PROF          0x40
#define LIS3MDL_OM_ULTRA_HI_PROF    0x60
#define LIS3MDL_DO_0_625            0x00
#define LIS3MDL_DO_1_25             0x04
#define LIS3MDL_DO_2_5              0x08
#define LIS3MDL_DO_5                0x0C
#define LIS3MDL_DO_10               0x10  // Default
#define LIS3MDL_DO_20               0x14
#define LIS3MDL_DO_40               0x18
#define LIS3MDL_DO_80               0x1C
#define LIS3MDL_FAST_ODR            0x02

// CTRL_REG2
#define LIS3MDL_FS_4GAUSS           0x00  // Default
#define LIS3MDL_FS_8GAUSS           0x20
#define LIS3MDL_FS_12GAUSS          0x40
#define LIS3MDL_FS_16GAUSS          0x60
#define LIS3MDL_REBOOT              0x08
#define LIS3MDL_SOFT_RST            0x04

// CTRL_REG3
#define LIS3MDL_LP                  0x20  // Default 0
#define LIS3MDL_SIM                 0x04  // Default 0
#define LIS3MDL_MD_CONTINUOUS       0x00  // Default
#define LIS3MDL_MD_SINGLE           0x01
#define LIS3MDL_MD_POWERDOWN        0x03

// CTRL_REG4
#define LIS3MDL_ZOM_LP              0x00  // Default
#define LIS3MDL_ZOM_MP              0x04
#define LIS3MDL_ZOM_HP              0x08
#define LIS3MDL_ZOM_UHP             0x0C
#define LIS3MDL_BLE                 0x02  // Default 0

// CTRL_REG5
#define LIS3MDL_FAST_READ           0x80  // Default 0
#define LIS3MDL_BDU                 0x40  // Default 0

static bool lis3mdlRead(magDev_t * mag, int16_t *magData)
{
    uint8_t buf[6];

    extDevice_t *dev = &mag->dev;

    bool ack = busReadRegisterBuffer(dev, LIS3MDL_REG_OUT_X_L, buf, 6);

    if (!ack) {
        return false;
    }

    magData[X] = (int16_t)(buf[1] << 8 | buf[0]) / 4;
    magData[Y] = (int16_t)(buf[3] << 8 | buf[2]) / 4;
    magData[Z] = (int16_t)(buf[5] << 8 | buf[4]) / 4;

    return true;
}

static bool lis3mdlInit(magDev_t *mag)
{
    extDevice_t *dev = &mag->dev;

    busDeviceRegister(dev);

    busWriteRegister(dev, LIS3MDL_REG_CTRL_REG2, LIS3MDL_FS_4GAUSS);
    busWriteRegister(dev, LIS3MDL_REG_CTRL_REG1, LIS3MDL_TEMP_EN | LIS3MDL_OM_ULTRA_HI_PROF | LIS3MDL_DO_80);
    busWriteRegister(dev, LIS3MDL_REG_CTRL_REG5, LIS3MDL_BDU);
    busWriteRegister(dev, LIS3MDL_REG_CTRL_REG4, LIS3MDL_ZOM_UHP);
    busWriteRegister(dev, LIS3MDL_REG_CTRL_REG3, 0x00);

    delay(100);

    return true;
}

bool lis3mdlDetect(magDev_t * mag)
{
    extDevice_t *dev = &mag->dev;

    uint8_t sig = 0;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = LIS3MDL_MAG_I2C_ADDRESS;
    }

    bool ack = busReadRegisterBuffer(&mag->dev, LIS3MDL_REG_WHO_AM_I, &sig, 1);

    if (!ack || sig != LIS3MDL_DEVICE_ID) {
        return false;
    }

    mag->init = lis3mdlInit;
    mag->read = lis3mdlRead;

    return true;
}
#endif
