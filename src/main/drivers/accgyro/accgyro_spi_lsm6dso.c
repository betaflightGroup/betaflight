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

#ifdef USE_ACCGYRO_LSM6DSO

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_lsm6dso.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

#if defined(USE_GYRO_EXTI) && defined(USE_MPU_DATA_READY_SIGNAL)
void lsm6dsoExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif

bool lsm6dsoAccRead(accDev_t *acc)
{
    enum {
        IDX_ACCEL_XOUT_L,
        IDX_ACCEL_XOUT_H,
        IDX_ACCEL_YOUT_L,
        IDX_ACCEL_YOUT_H,
        IDX_ACCEL_ZOUT_L,
        IDX_ACCEL_ZOUT_H,
        BUFFER_SIZE,
    };

    uint8_t lsm6dso_rx_buf[BUFFER_SIZE];

    extDevice_t *dev = &acc->gyro->dev;
    busReadRegisterBuffer(dev, LSM6DSO_REG_OUTX_L_A, lsm6dso_rx_buf, BUFFER_SIZE);

    acc->ADCRaw[X] = (int16_t)((lsm6dso_rx_buf[IDX_ACCEL_XOUT_H] << 8) | lsm6dso_rx_buf[IDX_ACCEL_XOUT_L]);
    acc->ADCRaw[Y] = (int16_t)((lsm6dso_rx_buf[IDX_ACCEL_YOUT_H] << 8) | lsm6dso_rx_buf[IDX_ACCEL_YOUT_L]);
    acc->ADCRaw[Z] = (int16_t)((lsm6dso_rx_buf[IDX_ACCEL_ZOUT_H] << 8) | lsm6dso_rx_buf[IDX_ACCEL_ZOUT_L]);

    return true;
}

bool lsm6dsoGyroRead(gyroDev_t *gyro)
{
    enum {
        IDX_GYRO_XOUT_L,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_ZOUT_L,
        IDX_GYRO_ZOUT_H,
        BUFFER_SIZE,
    };

    uint8_t lsm6dso_rx_buf[BUFFER_SIZE];

    extDevice_t *dev = &gyro->dev;
    busReadRegisterBuffer(dev, LSM6DSO_REG_OUTX_L_G, lsm6dso_rx_buf, BUFFER_SIZE);

    gyro->gyroADCRaw[X] = (int16_t)((lsm6dso_rx_buf[IDX_GYRO_XOUT_H] << 8) | lsm6dso_rx_buf[IDX_GYRO_XOUT_L]);
    gyro->gyroADCRaw[Y] = (int16_t)((lsm6dso_rx_buf[IDX_GYRO_YOUT_H] << 8) | lsm6dso_rx_buf[IDX_GYRO_YOUT_L]);
    gyro->gyroADCRaw[Z] = (int16_t)((lsm6dso_rx_buf[IDX_GYRO_ZOUT_H] << 8) | lsm6dso_rx_buf[IDX_GYRO_ZOUT_L]);

    return true;
}
#endif
