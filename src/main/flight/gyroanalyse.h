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

#define DYN_NOTCH_COUNT_MAX 5

typedef struct gyroAnalyseState_s {

    // accumulator for oversampled data => no aliasing and less noise
    uint8_t sampleCount;
    uint8_t maxSampleCount;
    float maxSampleCountRcp;
    float oversampledGyroAccumulator[XYZ_AXIS_COUNT];

    // downsampled gyro data for frequency analysis
    float downsampledGyroData[XYZ_AXIS_COUNT];

    // update state machine step information
    uint8_t updateTicks;
    uint8_t updateStep;
    uint8_t updateAxis;

    float centerFreq[XYZ_AXIS_COUNT][DYN_NOTCH_COUNT_MAX];

} gyroAnalyseState_t;

void gyroDataAnalyseInit(gyroAnalyseState_t *state, uint32_t targetLooptimeUs);
void gyroDataAnalysePush(gyroAnalyseState_t *state, const uint8_t axis, const float sample);
void gyroDataAnalyse(gyroAnalyseState_t *state);
uint16_t getMaxFFT(void);
void resetMaxFFT(void);
