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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SBUS_CHANNELS

#include "common/utils.h"

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus_channels.h"

#define DEBUG_SBUS_FRAME_INTERVAL 3

#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

uint8_t sbusChannelsDecode(rxRuntimeState_t *rxRuntimeState, const sbusChannels_t *channels)
{
    uint16_t *sbusChannelData = rxRuntimeState->channelData;
    sbusChannelData[0] = channels->chan0;
    sbusChannelData[1] = channels->chan1;
    sbusChannelData[2] = channels->chan2;
    sbusChannelData[3] = channels->chan3;
    sbusChannelData[4] = channels->chan4;
    sbusChannelData[5] = channels->chan5;
    sbusChannelData[6] = channels->chan6;
    sbusChannelData[7] = channels->chan7;
    sbusChannelData[8] = channels->chan8;
    sbusChannelData[9] = channels->chan9;
    sbusChannelData[10] = channels->chan10;
    sbusChannelData[11] = channels->chan11;
    sbusChannelData[12] = channels->chan12;
    sbusChannelData[13] = channels->chan13;
    sbusChannelData[14] = channels->chan14;
    sbusChannelData[15] = channels->chan15;

    if (channels->flags & SBUS_FLAG_CHANNEL_17) {
        sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_CHANNEL_18) {
        sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
        // internal failsafe enabled and rx failsafe flag set
        // RX *should* still be sending valid channel data (repeated), so use it.
        return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
    }

    if (channels->flags & SBUS_FLAG_SIGNAL_LOSS) {
        // The received data is a repeat of the last valid data so can be considered complete.
        return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
    }

    return RX_FRAME_COMPLETE;
}

static float sbusChannelsReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    // Linear fitting values read from OpenTX-ppmus and comparing with values received by X4R
    // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
    return (5 * (float)rxRuntimeState->channelData[chan] / 8) + 880;
}

void sbusChannelsInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeState->rcReadRawFn = sbusChannelsReadRawRC;
    for (int b = 0; b < SBUS_MAX_CHANNEL; b++) {
        rxRuntimeState->channelData[b] = (16 * rxConfig->midrc) / 10 - 1408;
    }
}
#endif
