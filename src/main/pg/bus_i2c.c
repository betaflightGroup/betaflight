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
#include <stddef.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "common/utils.h"

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/bus_i2c.h"

typedef struct i2cDefaultConfig_s {
    I2CDevice device;
    ioTag_t ioTagScl, ioTagSda;
    bool pullUp;
    uint16_t clockSpeed;
} i2cDefaultConfig_t;

static const i2cDefaultConfig_t i2cDefaultConfig[] = {
#ifdef USE_I2C_DEVICE_1
    { I2CDEV_1, IO_TAG(I2C1_SCL), IO_TAG(I2C1_SDA), I2C1_PULLUP, I2C1_CLOCKSPEED },
#endif
#ifdef USE_I2C_DEVICE_2
    { I2CDEV_2, IO_TAG(I2C2_SCL), IO_TAG(I2C2_SDA), I2C2_PULLUP, I2C2_CLOCKSPEED },
#endif
#ifdef USE_I2C_DEVICE_3
    { I2CDEV_3, IO_TAG(I2C3_SCL), IO_TAG(I2C3_SDA), I2C3_PULLUP, I2C3_CLOCKSPEED },
#endif
#ifdef USE_I2C_DEVICE_4
    { I2CDEV_4, IO_TAG(I2C4_SCL), IO_TAG(I2C4_SDA), I2C4_PULLUP, I2C4_CLOCKSPEED },
#endif
};

void pgResetFn_i2cConfig(i2cConfig_t *i2cConfig)
{
    memset(i2cConfig, 0, sizeof(*i2cConfig));

    for (size_t index = 0 ; index < ARRAYLEN(i2cDefaultConfig) ; index++) {
        const i2cDefaultConfig_t *defconf = &i2cDefaultConfig[index];
        int device = defconf->device;
        i2cConfig[device].ioTagScl = defconf->ioTagScl;
        i2cConfig[device].ioTagSda = defconf->ioTagSda;
        i2cConfig[device].pullUp = defconf->pullUp;
        i2cConfig[device].clockSpeed = defconf->clockSpeed;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(i2cConfig_t, I2CDEV_COUNT, i2cConfig, PG_I2C_CONFIG, 1);

#endif // defined(USE_I2C) && !defined(USE_SOFT_I2C)
