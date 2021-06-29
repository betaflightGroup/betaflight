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

#ifdef USE_MAG_AK8975

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_ak8975.h"

// This sensor is available in MPU-9150.
// This driver only support I2C mode, direct and in bypass configuration.

// AK8975, mag sensor address
#define AK8975_MAG_I2C_ADDRESS          0x0C
#define AK8975_DEVICE_ID                0x48

// Registers
#define AK8975_MAG_REG_WIA              0x00
#define AK8975_MAG_REG_INFO             0x01
#define AK8975_MAG_REG_ST1              0x02
#define AK8975_MAG_REG_HXL              0x03
#define AK8975_MAG_REG_HXH              0x04
#define AK8975_MAG_REG_HYL              0x05
#define AK8975_MAG_REG_HYH              0x06
#define AK8975_MAG_REG_HZL              0x07
#define AK8975_MAG_REG_HZH              0x08
#define AK8975_MAG_REG_ST2              0x09
#define AK8975_MAG_REG_CNTL             0x0A
#define AK8975_MAG_REG_ASTC             0x0C // self test
#define AK8975_MAG_REG_I2CDIS           0x0F
#define AK8975_MAG_REG_ASAX             0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8975_MAG_REG_ASAY             0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8975_MAG_REG_ASAZ             0x12 // Fuse ROM z-axis sensitivity adjustment value

#define ST1_REG_DATA_READY              0x01

#define ST2_REG_DATA_ERROR              0x04
#define ST2_REG_MAG_SENSOR_OVERFLOW     0x08

#define CNTL_MODE_POWER_DOWN            0x00
#define CNTL_MODE_ONCE                  0x01
#define CNTL_MODE_CONT1                 0x02
#define CNTL_MODE_FUSE_ROM              0x0F
#define CNTL_BIT_14_BIT                 0x00
#define CNTL_BIT_16_BIT                 0x10

static bool ak8975Init(magDev_t *mag)
{
    uint8_t asa[3];
    uint8_t status;

    extDevice_t *dev = &mag->dev;

    busDeviceRegister(dev);

    busWriteRegister(dev, AK8975_MAG_REG_CNTL, CNTL_MODE_POWER_DOWN); // power down before entering fuse mode
    delay(20);

    busWriteRegister(dev, AK8975_MAG_REG_CNTL, CNTL_MODE_FUSE_ROM); // Enter Fuse ROM access mode
    delay(10);

    busReadRegisterBuffer(dev, AK8975_MAG_REG_ASAX, asa, sizeof(asa)); // Read the x-, y-, and z-axis asa values
    delay(10);

    mag->magGain[X] = asa[X] + 128;
    mag->magGain[Y] = asa[Y] + 128;
    mag->magGain[Z] = asa[Z] + 128;

    busWriteRegister(dev, AK8975_MAG_REG_CNTL, CNTL_MODE_POWER_DOWN); // power down after reading.
    delay(10);

    // Clear status registers
    busReadRegisterBuffer(dev, AK8975_MAG_REG_ST1, &status, 1);
    busReadRegisterBuffer(dev, AK8975_MAG_REG_ST2, &status, 1);

    // Trigger first measurement
    busWriteRegister(dev, AK8975_MAG_REG_CNTL, CNTL_BIT_16_BIT | CNTL_MODE_ONCE);
    return true;
}

static int16_t parseMag(uint8_t *raw, int16_t gain) {
  int ret = (int16_t)(raw[1] << 8 | raw[0]) * gain / 256;
  return constrain(ret, INT16_MIN, INT16_MAX);
}

static bool ak8975Read(magDev_t *mag, int16_t *magData)
{
    bool ack;
    uint8_t status;
    uint8_t buf[6];

    extDevice_t *dev = &mag->dev;

    ack = busReadRegisterBuffer(dev, AK8975_MAG_REG_ST1, &status, 1);
    if (!ack || (status & ST1_REG_DATA_READY) == 0) {
        return false;
    }

    busReadRegisterBuffer(dev, AK8975_MAG_REG_HXL, buf, 6); // read from AK8975_MAG_REG_HXL to AK8975_MAG_REG_HZH

    ack = busReadRegisterBuffer(dev, AK8975_MAG_REG_ST2, &status, 1);
    if (!ack) {
        return false;
    }

    busWriteRegister(dev, AK8975_MAG_REG_CNTL, CNTL_BIT_16_BIT | CNTL_MODE_ONCE); // start reading again    uint8_t status2 = buf[6];

    if (status & ST2_REG_DATA_ERROR) {
        return false;
    }

    if (status & ST2_REG_MAG_SENSOR_OVERFLOW) {
        return false;
    }

    magData[X] = -parseMag(buf + 0, mag->magGain[X]);
    magData[Y] = -parseMag(buf + 2, mag->magGain[Y]);
    magData[Z] = -parseMag(buf + 4, mag->magGain[Z]);

    return true;
}

bool ak8975Detect(magDev_t *mag)
{
    uint8_t sig = 0;

    extDevice_t *dev = &mag->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = AK8975_MAG_I2C_ADDRESS;
    }

    bool ack = busReadRegisterBuffer(dev, AK8975_MAG_REG_WIA, &sig, 1);

    if (!ack || sig != AK8975_DEVICE_ID) { // 0x48 / 01001000 / 'H'
        return false;
    }

    mag->init = ak8975Init;
    mag->read = ak8975Read;

    return true;
}
#endif
