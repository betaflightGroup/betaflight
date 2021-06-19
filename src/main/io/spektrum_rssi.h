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

// Spektrum RSSI signal strength range, in dBm
#define SPEKTRUM_RSSI_MAX         (-42)
#define SPEKTRUM_RSSI_MIN         (-92)

// Spektrum RSSI reported value limit at or below which signals a fade instead of a real RSSI
#define SPEKTRUM_RSSI_FADE_LIMIT (-100)

#define SPEKTRUM_MAX_FADE_PER_SEC       40
#define SPEKTRUM_FADE_REPORTS_PER_SEC   2

#include "rx/rx.h"

typedef struct dbm_table_s
{
    int8_t  dBm;
    uint8_t reportAs;
} dbm_table_t;


void spektrumHandleRSSI(rxRuntimeState_t *rxRuntimeState, volatile uint8_t spekFrame[]);
