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

#include "platform.h"
#ifdef USE_SERIAL_RX
#if defined(USE_SPEKTRUM_REAL_RSSI) || defined(USE_SPEKTRUM_FAKE_RSSI)

#include "config/feature.h"
#include "common/utils.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/spektrum.h"
#include "io/spektrum_rssi.h"

// Number of fade outs counted as a link loss when using USE_SPEKTRUM_REAL_RSSI
#define SPEKTRUM_RSSI_LINK_LOSS_FADES 5

#ifdef USE_SPEKTRUM_FAKE_RSSI
// Spektrum Rx type. Determined by bind method.
static bool spektrumSatInternal = true; // Assume internal,bound by BF.

// Variables used for calculating a signal strength from satellite fade.
//  This is time-variant and computed every second based on the fade
//  count over the last second.
static uint32_t spek_fade_last_sec = 0; // Stores the timestamp of the last second.
static uint16_t spek_fade_last_sec_count = 0; // Stores the fade count at the last second.
#endif

// Linear mapping and interpolation function
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef USE_SPEKTRUM_RSSI_PERCENT_CONVERSION

// Conversion table from dBm to a percentage scale aproximating a more linear RSSI vs Distance curve.

static const dbm_table_t dbmTable[] = {
    {SPEKTRUM_RSSI_MAX, 101},
    {-49,100},
    {-56, 98},
    {-61, 95},
    {-66, 89},
    {-69, 83},
    {-71, 78},
    {-73, 72},
    {-74, 69},
    {-75, 66},
    {-76, 63},
    {-77, 60},
/*
    {-78, 56}, // Linear part of the table, can be interpolated
    {-79, 52},
    {-80, 48},
    {-81, 44},
    {-82, 40},
    {-83, 36},
    {-84, 32},
    {-85, 28},
    {-86, 24},
    {-87, 20}, // Beta Flight default RSSI % alatm point
    {-88, 16},
    {-89, 12},
    {-90,  8}, // Failsafe usually hits here
    {-91,  4}, // Linear part of the table end
*/
    {SPEKTRUM_RSSI_MIN, 0}};

// Convert dBm to Range %
static int8_t dBm2range (int8_t dBm) {
    int8_t  retval = dbmTable[0].reportAs;

    dBm = constrain(dBm, SPEKTRUM_RSSI_MIN, SPEKTRUM_RSSI_MAX);
    for ( uint8_t i = 1; i < ARRAYLEN(dbmTable); i++ ) {
        if (dBm >= dbmTable[i].dBm) {
            // Linear interpolation between table points.
            retval = map(dBm, dbmTable[i-1].dBm, dbmTable[i].dBm, dbmTable[i-1].reportAs, dbmTable[i].reportAs);
            break;
        }
    }

    retval = constrain(retval, 0, 100);
    return retval;
}
#endif

void spektrumHandleRSSI(rxRuntimeState_t *rxRuntimeState, volatile uint8_t spekFrame[]) {
#ifdef USE_SPEKTRUM_REAL_RSSI
    static int8_t spek_last_rssi = SPEKTRUM_RSSI_MAX;
    static uint8_t spek_fade_count = 0;

    // Fetch RSSI
    if (srxlEnabled) {
        // Real RSSI reported only by SRXL Telemetry Rx, in dBm.
        int8_t rssi = spekFrame[0];

        if (rssi <= SPEKTRUM_RSSI_FADE_LIMIT ) {
            // If Rx reports -100 dBm or less, it is a fade out and frame
            // loss.
            // If it is a temporary fade, real RSSI will come back in the
            // next frame, in that case
            // we should not report 0% back as OSD keeps a "minimum RSSI"
            // value. Instead keep last good report.
            // If it is a total link loss, failsafe will kick in.
            // The number of fades are counted and if it is equal or above
            // SPEKTRUM_RSSI_LINK_LOSS_FADES a link loss is assumed and
            // RSSI is set to Spektrums minimum RSSI value.

            spek_fade_count++;
            if (spek_fade_count < SPEKTRUM_RSSI_LINK_LOSS_FADES) {
                // Ignore report and keep last known good value
                rssi = spek_last_rssi;
            } else {
                // Link loss assumed, set RSSI to minimum value
                rssi = SPEKTRUM_RSSI_MIN;
            }
        } else {
            spek_fade_count = 0;
        }

        if(rssi_channel != 0) {
#ifdef USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
            // Do an dBm to percent conversion with an approxatelly linear distance
            // and map the percentage to RSSI RC channel range
            rxRuntimeState->channelXData[rssi_channel] = map(dBm2range (rssi),
                                                       0, 100,
                                                       0, resolution);
#else
            // Do a direkt dBm to percent mapping, keeping the non-linear dBm logarithmic curve.
            rxRuntimeState->channelXData[rssi_channel] = map(rssi),
                                                       SPEKTRUM_RSSI_MIN, SPEKTRUM_RSSI_MAX,
                                                       0, resolution);
#endif
        }
        spek_last_rssi = rssi;
    }

#ifdef USE_SPEKTRUM_FAKE_RSSI
    else
#endif
#endif // USE_SPEKTRUM_REAL_RSSI

#ifdef USE_SPEKTRUM_FAKE_RSSI
    {
        // Fake RSSI value computed from fades

        const uint32_t current_secs = micros() / 1000 / (1000 / SPEKTRUM_FADE_REPORTS_PER_SEC);
        uint16_t fade;
        uint8_t system;

        // Get fade count, different format depending on Rx rype and how Rx is bound. Initially assumed Internal
        if (spektrumSatInternal) {
            // Internal Rx, bind values 3, 5, 7, 9
            fade = (uint16_t) spekFrame[0];
            system = spekFrame[1];

            // Try to detect system type by assuming Internal until we find ANY frame telling otherwise.
            if ( !( (system == SPEKTRUM_DSM2_22) |
                    (system == SPEKTRUM_DSM2_11) |
                    (system == SPEKTRUM_DSMX_22) |
                    (system == SPEKTRUM_DSMX_11) ) ){
                spektrumSatInternal =false; // Nope, this is an externally bound Sat Rx
            }
        } else {
            // External Rx, bind values 4, 6, 8, 10
            fade = ((spekFrame[0] << 8) + spekFrame[1]);
        }

        if (spek_fade_last_sec == 0) {
            // This is the first frame status received.
            spek_fade_last_sec_count = fade;
            spek_fade_last_sec = current_secs;
        } else if (spek_fade_last_sec != current_secs) {
            // If the difference is > 1, then we missed several seconds worth of frames and
            // should just throw out the fade calc (as it's likely a full signal loss).
            if ((current_secs - spek_fade_last_sec) == 1) {
                if (rssi_channel != 0) {
                    rxRuntimeState->channelXData[rssi_channel] = map(fade - spek_fade_last_sec_count,
                                                                   SPEKTRUM_MAX_FADE_PER_SEC / SPEKTRUM_FADE_REPORTS_PER_SEC, 0,
                                                                   0, resolution);
                }
            }
            spek_fade_last_sec_count = fade;
            spek_fade_last_sec = current_secs;
        }
    }
#endif // USE_SPEKTRUM_FAKE_RSSI
}
#endif // USE_SPEKTRUM_REAL_RSSI || USE_SPEKTRUM_FAKE_RSSI
#endif // USE_SERIAL_RX
