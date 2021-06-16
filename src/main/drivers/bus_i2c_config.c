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
 * Created by jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/io.h"

#include "pg/bus_i2c.h"

void i2cHardwareConfigure(const i2cConfig_t *i2cConfig)
{
    for (int index = 0 ; index < I2CDEV_COUNT ; index++) {
        const i2cHardware_t *hardware = &i2cHardware[index];

        if (!hardware->reg) {
            continue;
        }

        I2CDevice device = hardware->device;
        i2cDevice_t *pDev = &i2cDevice[device];

        memset(pDev, 0, sizeof(*pDev));

        for (int pindex = 0 ; pindex < I2C_PIN_SEL_MAX ; pindex++) {
            if (i2cConfig[device].ioTagScl == hardware->sclPins[pindex].ioTag) {
                pDev->scl = IOGetByTag(i2cConfig[device].ioTagScl);
#if defined(STM32F4) || defined(STM32H7) || defined(STM32G4)
                pDev->sclAF = hardware->sclPins[pindex].af;
#endif
            }
            if (i2cConfig[device].ioTagSda == hardware->sdaPins[pindex].ioTag) {
                pDev->sda = IOGetByTag(i2cConfig[device].ioTagSda);
#if defined(STM32F4) || defined(STM32H7) || defined(STM32G4)
                pDev->sdaAF = hardware->sdaPins[pindex].af;
#endif
            }
        }

        if (pDev->scl && pDev->sda) {
            pDev->hardware = hardware;
            pDev->reg = hardware->reg;
            pDev->pullUp = i2cConfig[device].pullUp;
            pDev->clockSpeed = i2cConfig[device].clockSpeed;
        }
    }
}

#endif // defined(USE_I2C) && !defined(USE_SOFT_I2C)
