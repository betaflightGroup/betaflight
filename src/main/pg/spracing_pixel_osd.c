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
 * Author: Dominic Clifton / Seriously Pro Racing
 */
#include "platform.h"

#ifdef USE_SPRACING_PIXEL_OSD

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/io.h"
#include "drivers/bus_spi.h"

#include "spracing_pixel_osd.h"

PG_REGISTER_WITH_RESET_FN(spracingPixelOSDConfig_t, spracingPixelOSDConfig, PG_SPRACING_PIXEL_OSD_CONFIG, 0);

void pgResetFn_spracingPixelOSDConfig(spracingPixelOSDConfig_t *config)
{
    config->unused = 0;
}
#endif // USE_SPRACING_PIXEL_OSD
