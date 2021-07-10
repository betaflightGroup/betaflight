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

/* original work by Rav
 * 
 * 2018_07 updated by ctzsnooze to post filter, wider Q, different peak detection
 * coding assistance and advice from DieHertz, Rav, eTracer
 * test pilots icr4sh, UAV Tech, Flint723
 * 
 * 2021_02 updated by KarateBrot: switched FFT with SDFT, multiple notches per axis
 * test pilots: Sugar K, bizmar
 */

#include <math.h>

#include "platform.h"

#ifdef USE_DYN_NOTCH_FILTER
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/sdft.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/time.h"

#include "fc/core.h"

#include "pg/pg_ids.h"

#include "sensors/gyro.h"

#include "dyn_notch_filter.h"

// SDFT_SAMPLE_SIZE defaults to 72 (common/sdft.h).
// We get 36 frequency bins from 72 consecutive data values, called SDFT_BIN_COUNT (common/sdft.h)
// Bin 0 is DC and can't be used.
// Only bins 1 to 35 are usable.

// A gyro sample is collected every PID loop.
// sampleCount recent gyro values are accumulated and averaged
// to ensure that 72 samples are collected at the right rate for the required SDFT bandwidth.

// For an 8k PID loop, at default 600hz max, 6 sequential gyro data points are averaged, SDFT runs 1333Hz.
// Upper limit of SDFT is half that frequency, eg 666Hz by default.
// At 8k, if user sets a max of 300Hz, int(8000/600) = 13, sdftSampleRateHz = 615Hz, range 307Hz.
// Note that lower max requires more samples to be averaged, increasing precision but taking longer to get enough samples.
// For Bosch at 3200Hz gyro, max of 600, int(3200/1200) = 2, sdftSampleRateHz = 1600, range to 800hz.
// For Bosch on XClass, better to set a max of 300, int(3200/600) = 5, sdftSampleRateHz = 640, range to 320Hz.

// When sampleIndex reaches sampleCount, the averaged gyro value is put into the corresponding SDFT.
// At 8k, with 600Hz max, sampleCount = 6, this happens every 6 * 0.125us, or every 0.75ms.
// Hence to completely replace all 72 samples of the SDFT input buffer with clean new data takes 54ms.

// The SDFT code is split into steps. It takes 4 PID loops to calculate the SDFT, track peaks and update the filters for one axis.
// Since there are three axes, it takes 12 PID loops to completely update all axes.
// At 8k, any one axis gets updated at 8000 / 12 or 666hz or every 1.5ms
// In this time, 2 points in the SDFT buffer will have changed.
// At 4k, it takes twice as long to update an axis, i.e. each axis updates only every 3ms.
// Four points in the buffer will have changed in that time, and each point will be the average of three samples.
// Hence output jitter at 4k is about four times worse than at 8k. At 2k output jitter is quite bad.

// Each SDFT output bin has width sdftSampleRateHz/72, ie 18.5Hz per bin at 1333Hz.
// Usable bandwidth is half this, ie 666Hz if sdftSampleRateHz is 1333Hz, i.e. bin 1 is 18.5Hz, bin 2 is 37.0Hz etc.

#define DYN_NOTCH_SMOOTH_HZ        4
#define DYN_NOTCH_CALC_TICKS       (XYZ_AXIS_COUNT * STEP_COUNT) // 3 axes and 4 steps per axis
#define DYN_NOTCH_OSD_MIN_THROTTLE 20

typedef enum {

    STEP_WINDOW,
    STEP_DETECT_PEAKS,
    STEP_CALC_FREQUENCIES,
    STEP_UPDATE_FILTERS,
    STEP_COUNT

} step_e;

typedef struct peak_s {

    int bin;
    float value;

} peak_t;

typedef struct state_s {

    // state machine step information
    int tick;
    int step;
    int axis;

} state_t;

typedef struct dynNotch_s {

    float q;
    float minHz;
    float maxHz;
    int count;

    uint16_t maxCenterFreq;
    float centerFreq[XYZ_AXIS_COUNT][DYN_NOTCH_COUNT_MAX];
    
    uint32_t looptimeUs;
    biquadFilter_t notch[XYZ_AXIS_COUNT][DYN_NOTCH_COUNT_MAX];

} dynNotch_t;

// dynamic notch instance
static FAST_DATA_ZERO_INIT dynNotch_t dynNotch;

// accumulator for oversampled data => no aliasing and less noise
static FAST_DATA_ZERO_INIT int   sampleIndex;
static FAST_DATA_ZERO_INIT int   sampleCount;
static FAST_DATA_ZERO_INIT float sampleCountRcp;
static FAST_DATA_ZERO_INIT float sampleAccumulator[XYZ_AXIS_COUNT];

// downsampled data for frequency analysis
static FAST_DATA_ZERO_INIT float sampleAvg[XYZ_AXIS_COUNT];

// parameters for peak detection and frequency analysis
static FAST_DATA_ZERO_INIT state_t state;
static FAST_DATA_ZERO_INIT sdft_t  sdft[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT peak_t  peaks[DYN_NOTCH_COUNT_MAX];
static FAST_DATA_ZERO_INIT float   sdftData[SDFT_BIN_COUNT];
static FAST_DATA_ZERO_INIT float   sdftSampleRateHz;
static FAST_DATA_ZERO_INIT float   sdftResolutionHz;
static FAST_DATA_ZERO_INIT int     sdftStartBin;
static FAST_DATA_ZERO_INIT int     sdftEndBin;
static FAST_DATA_ZERO_INIT float   sdftMeanSq;
static FAST_DATA_ZERO_INIT float   gain;

PG_REGISTER_WITH_RESET_FN(dynNotchConfig_t, dynNotchConfig, PG_DYN_NOTCH_CONFIG, 0);

void pgResetFn_dynNotchConfig(dynNotchConfig_t *config)
{
    config->dyn_notch_min_hz = 150;
    config->dyn_notch_max_hz = 600;
    config->dyn_notch_q = 300;
    config->dyn_notch_count = 3;
}

void dynNotchInit(const dynNotchConfig_t *config, const uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    dynNotch.q = config->dyn_notch_q / 100.0f;
    dynNotch.minHz = config->dyn_notch_min_hz;
    dynNotch.maxHz = MAX(2 * dynNotch.minHz, config->dyn_notch_max_hz);
    dynNotch.count = config->dyn_notch_count;
    dynNotch.looptimeUs = targetLooptimeUs;
    dynNotch.maxCenterFreq = 0;

    // dynNotchUpdate() is running at looprateHz (which is PID looprate aka. 1e6f / gyro.targetLooptime)
    const float looprateHz = 1.0f / dynNotch.looptimeUs * 1e6f;
    sampleCount = MAX(1, looprateHz / (2 * dynNotch.maxHz)); // 600hz, 8k looptime, 6.00
    sampleCountRcp = 1.0f / sampleCount;

    sdftSampleRateHz = looprateHz / sampleCount;
    // eg 8k, user max 600hz, int(8000/1200) = 6 (6.666), sdftSampleRateHz = 1333hz, range 666Hz
    // eg 4k, user max 600hz, int(4000/1200) = 3 (3.333), sdftSampleRateHz = 1333hz, range 666Hz
    // eg 2k, user max 600hz, int(2000/1200) = 1 (1.666) sdftSampleRateHz = 2000hz, range 1000Hz
    // eg 2k, user max 400hz, int(2000/800) = 2 (2.5) sdftSampleRateHz = 1000hz, range 500Hz
    // eg 1k, user max 600hz, int(1000/1200) = 1 (max(1,0.8333)) sdftSampleRateHz = 1000hz, range 500Hz
    // the upper limit of DN is always going to be the Nyquist frequency (= sampleRate / 2)

    sdftResolutionHz = sdftSampleRateHz / SDFT_SAMPLE_SIZE; // 18.5hz per bin at 8k and 600Hz maxHz
    sdftStartBin = MAX(2, dynNotch.minHz / sdftResolutionHz + 0.5f); // can't use bin 0 because it is DC.
    sdftEndBin = MIN(SDFT_BIN_COUNT - 1, dynNotch.maxHz / sdftResolutionHz + 0.5f); // can't use more than SDFT_BIN_COUNT bins.
    gain = pt1FilterGain(DYN_NOTCH_SMOOTH_HZ, DYN_NOTCH_CALC_TICKS / looprateHz); // minimum PT1 k value

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        sdftInit(&sdft[axis], sdftStartBin, sdftEndBin, sampleCount);
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int p = 0; p < dynNotch.count; p++) {
            // any init value is fine, but evenly spreading centerFreqs across frequency range makes notch filters stick to peaks quicker
            dynNotch.centerFreq[axis][p] = (p + 0.5f) * (dynNotch.maxHz - dynNotch.minHz) / (float)dynNotch.count + dynNotch.minHz;
            biquadFilterInit(&dynNotch.notch[axis][p], dynNotch.centerFreq[axis][p], dynNotch.looptimeUs, dynNotch.q, FILTER_NOTCH);
        }
    }
}

// Collect gyro data, to be downsampled and analysed in dynNotchUpdate() function
FAST_CODE void dynNotchPush(const int axis, const float sample)
{
    sampleAccumulator[axis] += sample;
}

static void dynNotchProcess(void);

// Downsample and analyse gyro data
FAST_CODE void dynNotchUpdate(void)
{
    // samples should have been pushed by `dynNotchPush`
    // if gyro sampling is > 1kHz, accumulate and average multiple gyro samples
    if (sampleIndex == sampleCount) {
        sampleIndex = 0;

        // calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            sampleAvg[axis] = sampleAccumulator[axis] * sampleCountRcp;
            sampleAccumulator[axis] = 0;
            if (axis == gyro.gyroDebugAxis) {
                DEBUG_SET(DEBUG_FFT, 2, lrintf(sampleAvg[axis]));
            }
        }

        // We need DYN_NOTCH_CALC_TICKS ticks to update all axes with newly sampled value
        // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
        // at 8kHz PID loop rate this means 8kHz / 4 / 3 = 666Hz => update every 1.5ms
        // at 4kHz PID loop rate this means 4kHz / 4 / 3 = 333Hz => update every 3ms
        state.tick = DYN_NOTCH_CALC_TICKS;
    }

    // 2us @ F722
    // SDFT processing in batches to synchronize with incoming downsampled data
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        sdftPushBatch(&sdft[axis], sampleAvg[axis], sampleIndex);
    }
    sampleIndex++;

    // Find frequency peaks and update filters
    if (state.tick > 0) {
        dynNotchProcess();
        --state.tick;
    }
}

// Find frequency peaks and update filters
static FAST_CODE_NOINLINE void dynNotchProcess(void)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_FFT_TIME) {
        startTime = micros();
    }

    DEBUG_SET(DEBUG_FFT_TIME, 0, state.step);

    switch (state.step) {
    
        case STEP_WINDOW: // 6us @ F722
        {
            sdftWinSq(&sdft[state.axis], sdftData);
            
            // Calculate mean square over frequency range (= average power of vibrations)
            sdftMeanSq = 0.0f;
            for (int bin = (sdftStartBin + 1); bin < sdftEndBin; bin++) {   // don't use startBin or endBin because they are not windowed properly
                sdftMeanSq += sdftData[bin];                                // sdftData is already squared (see sdftWinSq)
            }
            sdftMeanSq /= sdftEndBin - sdftStartBin - 1;

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_DETECT_PEAKS: // 6us @ F722
        {
            // Get memory ready for new peak data on current axis
            for (int p = 0; p < dynNotch.count; p++) {
                peaks[p].bin = 0;
                peaks[p].value = 0.0f;
            }

            // Search for N biggest peaks in frequency spectrum
            for (int bin = (sdftStartBin + 1); bin < sdftEndBin; bin++) {
                // Check if bin is peak
                if ((sdftData[bin] > sdftData[bin - 1]) && (sdftData[bin] > sdftData[bin + 1])) {
                    // Check if peak is big enough to be one of N biggest peaks.
                    // If so, insert peak and sort peaks in descending height order
                    for (int p = 0; p < dynNotch.count; p++) {
                        if (sdftData[bin] > peaks[p].value) {
                            for (int k = dynNotch.count - 1; k > p; k--) {
                                peaks[k] = peaks[k - 1];
                            }
                            peaks[p].bin = bin;
                            peaks[p].value = sdftData[bin];
                            break;
                        }
                    }
                    bin++; // If bin is peak, next bin can't be peak => jump it
                }
            }

            // Sort N biggest peaks in ascending bin order (example: 3, 8, 25, 0, 0, ..., 0)
            for (int p = dynNotch.count - 1; p > 0; p--) {
                for (int k = 0; k < p; k++) {
                    // Swap peaks but ignore swapping void peaks (bin = 0). This leaves
                    // void peaks at the end of peaks array without moving them
                    if (peaks[k].bin > peaks[k + 1].bin && peaks[k + 1].bin != 0) {
                        peak_t temp = peaks[k];
                        peaks[k] = peaks[k + 1];
                        peaks[k + 1] = temp;
                    }
                }
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_CALC_FREQUENCIES: // 4us @ F722
        {
            for (int p = 0; p < dynNotch.count; p++) {

                // Only update dynNotch.centerFreq if there is a peak (ignore void peaks) and if peak is above noise floor
                if (peaks[p].bin != 0 && peaks[p].value > sdftMeanSq) {

                    float meanBin = peaks[p].bin;

                    // Height of peak bin (y1) and shoulder bins (y0, y2)
                    const float y0 = sdftData[peaks[p].bin - 1];
                    const float y1 = sdftData[peaks[p].bin];
                    const float y2 = sdftData[peaks[p].bin + 1];

                    // Estimate true peak position aka. meanBin (fit parabola y(x) over y0, y1 and y2, solve dy/dx=0 for x)
                    const float denom = 2.0f * (y0 - 2 * y1 + y2);
                    if (denom != 0.0f) {
                        meanBin += (y0 - y2) / denom;
                    }

                    // Convert bin to frequency: freq = bin * binResoultion (bin 0 is 0Hz)
                    const float centerFreq = constrainf(meanBin * sdftResolutionHz, dynNotch.minHz, dynNotch.maxHz);

                    // PT1 style smoothing moves notch center freqs rapidly towards big peaks and slowly away, up to 8x faster 
                    // DYN_NOTCH_SMOOTH_HZ = 4 & gainMultiplier = 1 .. 8  =>  PT1 -3dB cutoff frequency = 4Hz .. 41Hz
                    const float gainMultiplier = constrainf(peaks[p].value / sdftMeanSq, 1.0f, 8.0f);

                    // Finally update notch center frequency p on current axis
                    dynNotch.centerFreq[state.axis][p] += gain * gainMultiplier * (centerFreq - dynNotch.centerFreq[state.axis][p]);
                }
            }

            if(calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
                for (int p = 0; p < dynNotch.count; p++) {
                    dynNotch.maxCenterFreq = MAX(dynNotch.maxCenterFreq, dynNotch.centerFreq[state.axis][p]);
                }
            }

            if (state.axis == gyro.gyroDebugAxis) {
                for (int p = 0; p < dynNotch.count && p < 3; p++) {
                    DEBUG_SET(DEBUG_FFT_FREQ, p, lrintf(dynNotch.centerFreq[state.axis][p]));
                }
                DEBUG_SET(DEBUG_DYN_LPF, 1, lrintf(dynNotch.centerFreq[state.axis][0]));
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_UPDATE_FILTERS: // 7us @ F722
        {
            for (int p = 0; p < dynNotch.count; p++) {
                // Only update notch filter coefficients if the corresponding peak got its center frequency updated in the previous step
                if (peaks[p].bin != 0 && peaks[p].value > sdftMeanSq) {
                    biquadFilterUpdate(&dynNotch.notch[state.axis][p], dynNotch.centerFreq[state.axis][p], dynNotch.looptimeUs, dynNotch.q, FILTER_NOTCH);
                }
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            state.axis = (state.axis + 1) % XYZ_AXIS_COUNT;
        }
    }

    state.step = (state.step + 1) % STEP_COUNT;
}

FAST_CODE float dynNotchFilter(const int axis, float value) 
{
    for (uint8_t p = 0; p < dynNotch.count; p++) {
        value = biquadFilterApplyDF1(&dynNotch.notch[axis][p], value);
    }

    return value;
}

bool isDynamicFilterActive(void)
{
    return featureIsEnabled(FEATURE_DYNAMIC_FILTER);
}

uint16_t getMaxFFT(void)
{
    return dynNotch.maxCenterFreq;
}

void resetMaxFFT(void)
{
    dynNotch.maxCenterFreq = 0;
}

#endif // USE_DYN_NOTCH_FILTER
