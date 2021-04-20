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

bool icm42605AccDetect(accDev_t *acc);
bool icm42605GyroDetect(gyroDev_t *gyro);

void icm42605AccInit(accDev_t *acc);
void icm42605GyroInit(gyroDev_t *gyro);

uint8_t icm42605SpiDetect(const extDevice_t *dev);

bool icm42605SpiAccDetect(accDev_t *acc);
bool icm42605SpiGyroDetect(gyroDev_t *gyro);
