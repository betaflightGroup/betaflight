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

bool i2cBusWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool i2cBusWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool i2cBusReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t i2cBusReadRegister(const extDevice_t *dev, uint8_t reg);
bool i2cBusReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool i2cBusBusy(const extDevice_t *dev, bool *error);
// Associate a device with an I2C bus
bool i2cBusSetInstance(const extDevice_t *dev, uint32_t device);
void i2cBusDeviceRegister(const extDevice_t *dev);
