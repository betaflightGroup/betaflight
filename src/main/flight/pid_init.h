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

void pidInit(const pidProfile_t *pidProfile);
void pidInitFilters(const pidProfile_t *pidProfile);
void pidInitConfig(const pidProfile_t *pidProfile);
void pidSetItermAccelerator(float newItermAccelerator);
void pidInitFeedforwardLpf(uint16_t filterCutoff, uint8_t debugAxis);
void pidUpdateFeedforwardLpf(uint16_t filterCutoff);
void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex);
