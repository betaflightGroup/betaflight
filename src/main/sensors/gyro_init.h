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

#include "pg/gyrodev.h"
#include "sensors/gyro.h"

void gyroSetTargetLooptime(uint8_t pidDenom);
void gyroPreInit(void);
bool gyroInit(void);
void gyroInitFilters(void);
void gyroInitSensor(gyroSensor_t *gyroSensor, const gyroDeviceConfig_t *config);
gyroDetectionFlags_t getGyroDetectionFlags(void);
gyroDev_t *gyroActiveDev(void);
struct mpuDetectionResult_s;
const struct mpuDetectionResult_s *gyroMpuDetectionResult(void);
int16_t gyroRateDps(int axis);
uint8_t gyroReadRegister(uint8_t whichSensor, uint8_t reg);
