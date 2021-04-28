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

#include <math.h>
#include <stdbool.h>

#include "platform.h"

#include "common/maths.h"
#include "common/sdft.h"

#define SDFT_R 0.9999f  // damping factor for guaranteed SDFT stability (r < 1.0f) 

static FAST_DATA_ZERO_INIT float     rPowerN;  // SDFT_R to the power of SDFT_SAMPLE_SIZE
static FAST_DATA_ZERO_INIT bool      isInitialized;
static FAST_DATA_ZERO_INIT complex_t twiddle[SDFT_BIN_COUNT];

static void applySqrt(const sdft_t *sdft, float *data);


void sdftInit(sdft_t *sdft, const uint8_t startBin, const uint8_t endBin, const uint8_t numBatches)
{
    if (!isInitialized) {
        rPowerN = powf(SDFT_R, SDFT_SAMPLE_SIZE);
        const float c = 2.0f * M_PIf / (float)SDFT_SAMPLE_SIZE;
        float phi = 0.0f;
        for (uint8_t i = 0; i < SDFT_BIN_COUNT; i++) {
            phi = c * i;
            twiddle[i] = SDFT_R * (cos_approx(phi) + _Complex_I * sin_approx(phi));
        }
        isInitialized = true;
    }

    sdft->idx = 0;
    sdft->startBin = startBin;
    sdft->endBin = endBin;
    sdft->numBatches = numBatches;
    sdft->batchSize = (sdft->endBin - sdft->startBin + 1) / sdft->numBatches + 1;

    for (uint8_t i = 0; i < SDFT_SAMPLE_SIZE; i++) {
        sdft->samples[i] = 0.0f;
    }

    for (uint8_t i = 0; i < SDFT_BIN_COUNT; i++) {
        sdft->data[i] = 0.0f;
    }
}


// Add new sample to frequency spectrum
FAST_CODE void sdftPush(sdft_t *sdft, const float *sample)
{
    const float delta = *sample - rPowerN * sdft->samples[sdft->idx];
    
    sdft->samples[sdft->idx] = *sample;
    sdft->idx = (sdft->idx + 1) % SDFT_SAMPLE_SIZE;

    for (uint8_t i = sdft->startBin; i <= sdft->endBin; i++) {
        sdft->data[i] = twiddle[i] * (sdft->data[i] + delta);
    }
}


// Add new sample to frequency spectrum in parts
FAST_CODE void sdftPushBatch(sdft_t* sdft, const float *sample, const uint8_t *batchIdx)
{
    const uint8_t batchStart = sdft->batchSize * *batchIdx;
    uint8_t batchEnd = batchStart;

    const float delta = *sample - rPowerN * sdft->samples[sdft->idx];

    if (*batchIdx == sdft->numBatches - 1) {
        sdft->samples[sdft->idx] = *sample;
        sdft->idx = (sdft->idx + 1) % SDFT_SAMPLE_SIZE;
        batchEnd += sdft->endBin - batchStart + 1;
    } else {
        batchEnd += sdft->batchSize;
    }

    for (uint8_t i = batchStart; i < batchEnd; i++) {
        sdft->data[i] = twiddle[i] * (sdft->data[i] + delta);
    }
}


// Get squared magnitude of frequency spectrum
FAST_CODE void sdftMagSq(const sdft_t *sdft, float *output)
{
    float re;
    float im;

    for (uint8_t i = sdft->startBin; i <= sdft->endBin; i++) {
        re = crealf(sdft->data[i]);
        im = cimagf(sdft->data[i]);
        output[i] = re * re + im * im;
    }
}


// Get magnitude of frequency spectrum (slower)
FAST_CODE void sdftMagnitude(const sdft_t *sdft, float *output)
{
    sdftMagSq(sdft, output);
    applySqrt(sdft, output);
}


// Get squared magnitude of frequency spectrum with Hann window applied
// Hann window in frequency domain: X[k] = -0.25 * X[k-1] +0.5 * X[k] -0.25 * X[k+1]
FAST_CODE void sdftWinSq(const sdft_t *sdft, float *output)
{
    complex_t val;
    float re;
    float im;

    for (uint8_t i = (sdft->startBin + 1); i < sdft->endBin; i++) {
        val = sdft->data[i] - 0.5f * (sdft->data[i - 1] + sdft->data[i + 1]); // multiply by 2 to save one multiplication
        re = crealf(val);
        im = cimagf(val);
        output[i] = re * re + im * im;
    }
}


// Get magnitude of frequency spectrum with Hann window applied (slower)
FAST_CODE void sdftWindow(const sdft_t *sdft, float *output)
{
    sdftWinSq(sdft, output);
    applySqrt(sdft, output);
}


// Apply square root to the whole sdft range
static FAST_CODE void applySqrt(const sdft_t *sdft, float *data)
{
    for (uint8_t i = sdft->startBin; i <= sdft->endBin; i++) {
        data[i] = sqrtf(data[i]);
    }
}
