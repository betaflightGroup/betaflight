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
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working Pixel OSD system.
 */
#include <stdint.h>

#include "platform.h"

#include "spracingpixelosd_api.h"

#include "spracing_pixel_osd_library.h"

// provided by linker script
extern const uint8_t __library_descriptor_start;
extern const uint8_t __library_vtable_start;

const spracingPixelOSDLibraryVTable_t * const spracingPixelOSDLibraryVTable = (spracingPixelOSDLibraryVTable_t *)&__library_vtable_start;
const spracingPixelOSDLibraryDescriptor_t * const spracingPixelOSDLibraryDescriptor = (spracingPixelOSDLibraryDescriptor_t *)&__library_descriptor_start;

bool spracingPixelOSDIsLibraryAvailable(void)
{
    return ((spracingPixelOSDLibraryDescriptor->code == SPRACINGPIXELOSD_LIBRARY_CODE) && (spracingPixelOSDLibraryDescriptor->apiVersion == SPRACINGPIXELOSD_LIBRARY_API_VERSION));
}
