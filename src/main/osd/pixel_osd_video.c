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

/*
 * Author: Dominic Clifton
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_PIXEL_OSD

#include "build/build_config.h"

#include "drivers/spracingpixelosd/spracing_pixel_osd.h"

#include "pixel_osd_video.h"

extern const spracingPixelOSDLibraryVTable_t *spracingPixelOSDLibraryVTable;
extern spracingPixelOSDState_t *pixelOSDState;

FAST_CODE bool taskPixelOSDVideoCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    spracingPixelOSDLibraryVTable->refreshState(currentTimeUs);

    bool isReady = (pixelOSDState->flags & PIXELOSD_FLAG_SERVICE_REQUIRED);

    return isReady;
}

FAST_CODE void taskPixelOSDVideo(timeUs_t currentTimeUs)
{
    // Handle the more frequent operations first

    if (pixelOSDState->flags & PIXELOSD_FLAG_SERVICE_REQUIRED) {
        spracingPixelOSDLibraryVTable->service(currentTimeUs);
    }
}

#endif
