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

typedef enum bjf4HardwareRevision_t {
    UNKNOWN = 0,
    BJF4_REV1,       // Flash
    BJF4_REV2,       // SDCard
    BJF4_REV3,       // SDCard + Flash
    BJF4_MINI_REV3A, // Flash (20x20 mini format)
    BJF4_REV4        // SDCard only - improved UART1 inversion
} bjf4HardwareRevision_e;

extern uint8_t hardwareRevision;

void updateHardwareRevision(void);
void detectHardwareRevision(void);
