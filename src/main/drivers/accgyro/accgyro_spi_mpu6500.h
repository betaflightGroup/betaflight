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

#pragma once

#include "drivers/bus.h"

uint8_t mpu6500SpiDetect(const extDevice_t *dev);

bool mpu6500SpiAccDetect(accDev_t *acc);
bool mpu6500SpiGyroDetect(gyroDev_t *gyro);

void mpu6500SpiGyroInit(gyroDev_t *gyro);
void mpu6500SpiAccInit(accDev_t *acc);
